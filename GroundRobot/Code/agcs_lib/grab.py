#!/usr/bin/python3
# coding=utf8
"""夹取：官方 block_fetch.py 的地面定点夹取。

流程：
1. 机械臂到检测位 (0,15,5) pitch=-90；
2. 稳定识别目标；
3. pixel_to_arm_coord 换算机械臂坐标 (x,y)；
4. 可及范围检查；
5. 按官方 block_fetch 的 move() 夹取、抬起、放置、复位；
6. 夹取后验证目标是否消失。
"""
import time

from agcs_lib.motion import go_forward, go_back
from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.sensors import show_status, dist_cm
from agcs_lib.logs import get_logger, action_msg


class Grabber:
    def __init__(self, board, ik, ak, params, K, R, T, detect, display=None, ultrasonic=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.K, self.R, self.T = K, R, T
        self.detect = detect
        self.display = display
        self.ultrasonic = ultrasonic
        self.log = get_logger()

        arm = params['arm']
        walk = params.get('walk', {})
        gc = params.get('grab', {})

        self.detect_pose = [float(v) for v in arm.get('detect_pose', [0, 15, 5])]
        self.pick_z = float(arm.get('pick_z', 5.0))
        self.grab_pitch = int(gc.get('grab_pitch', -90))
        self.gripper_open = int(arm.get('gripper_open', 120))
        self.gripper_close = int(arm.get('gripper_close', 550))
        self.raise_pose = [float(v) for v in arm.get('raise_pose', [12, 24, 5])]
        self.release_pose = [float(v) for v in arm.get('release_pose', [12, 24, -5])]
        self.reach_x = float(walk.get('reach_x', 8.0))
        self.reach_y = float(walk.get('reach_y', 24.0))
        self.attempts = int(gc.get('attempts', 3))
        self.cy_ref = float(gc.get('cy_ref', 240.0))
        self.height_gain = float(gc.get('height_gain', 0.05))
        self.base_pulse = float(gc.get('base_pulse', 220.0))
        self.height_gain_pulse = float(gc.get('height_gain_pulse', 0.05))
        self.area_ref = float(gc.get('area_ref', 3000.0))
        self.area_z_gain = float(gc.get('area_z_gain', 0.0005))
        self.grab_y_max = float(gc.get('grab_y_max', 18.0))
        self.grab_y_min = float(gc.get('grab_y_min', 6.0))
        self.fine_step_mm = int(gc.get('fine_step_mm', 10))
        self.align_iter = int(gc.get('align_iter', 20))
        self.ultra_weight = float(gc.get('ultra_weight', 0.2))
        self.ultra_max_cm = float(gc.get('ultra_max_cm', 50.0))

    def _status(self, v):
        show_status(self.display, v)

    def _stable(self, frames=80, need=5, jitter=5):
        """连续多帧目标坐标稳定才返回 dict；否则 None。"""
        stable = 0
        old = None
        last = None
        for _ in range(frames):
            r = self.detect()
            if r is None:
                stable = 0
                old = None
                time.sleep(0.05)
                continue
            c = r['center']
            if old is not None and abs(c[0] - old[0]) < jitter and abs(c[1] - old[1]) < jitter:
                stable += 1
            else:
                stable = 0
            old = c
            last = r
            if stable >= need:
                return last
            time.sleep(0.05)
        return None

    def _coord(self, center):
        return pixel_to_arm_coord(self.K, self.R, self.T, center,
                                  initial_coord=(self.detect_pose[0], self.detect_pose[1]))

    def _rescan(self):
        """目标突然丢失时，用 24 号上下扫重新找回。"""
        self.log.info('[grab] %s', action_msg('重扫找回目标', action='21=500，24上下扫'))
        self.x_dis = 500
        best = None
        best_y = 0
        best_score = 10 ** 9
        for y in range(0, 1001, 20):
            self.y_dis = y
            self.board.bus_servo_set_position(0.03, [[24, y], [21, self.x_dis]])
            time.sleep(0.03)
            r = self.detect()
            if r is not None:
                cx, cy = r['center']
                score = abs(cx - 320) + abs(cy - 240)
                if score < best_score:
                    best = r
                    best_y = y
                    best_score = score
        if best is not None:
            self.y_dis = best_y
            self.board.bus_servo_set_position(0.3, [[24, best_y], [21, self.x_dis]])
            time.sleep(0.3)
        return best

    def _target_xyz(self, center, area=None):
        """当前先用手眼标定 K/R/T 换算 x,y，z 用 pick_z；后续可替换为更准的手眼标定。"""
        x, y = self._coord(center)
        d = dist_cm(self.ultrasonic)
        if 0 < d < self.ultra_max_cm:
            y = y * (1.0 - self.ultra_weight) + d * self.ultra_weight
            self.log.debug('[grab] %s', action_msg('超声波协同', action='视觉y=%.1f 超声=%.1f 融合y=%.1f' % (y, d, y)))
        z = self.pick_z + self.height_gain_pulse * (self.y_dis - self.base_pulse)
        if area is not None:
            z += self.area_z_gain * (area - self.area_ref)
        return x, y, z

    def _grab_once(self, x, y, z):
        """官方 block_fetch.py 的 move() 动作。"""
        if self.ak.setPitchRangeMoving((x, y, z), self.grab_pitch, -90, 100, 1) is False:
            return False
        time.sleep(3)

        self.board.bus_servo_set_position(0.5, [[25, self.gripper_close]])
        time.sleep(2)

        self.ak.setPitchRangeMoving(tuple(self.raise_pose), self.grab_pitch, -90, 100, 1.5)
        time.sleep(1.5)
        self.ak.setPitchRangeMoving(tuple(self.release_pose), self.grab_pitch, -90, 100, 1)
        time.sleep(1)

        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)

        self.ak.setPitchRangeMoving(tuple(self.raise_pose), self.grab_pitch, -90, 100, 1)
        time.sleep(1)
        self.ak.setPitchRangeMoving(tuple(self.detect_pose), self.grab_pitch, -90, 100, 1.5)
        time.sleep(1.5)
        return True

    def _verify(self):
        """夹取后目标消失才认为成功。"""
        for _ in range(5):
            r = self.detect()
            if r is None:
                return True
            time.sleep(0.1)
        return False

    def run(self, cy=None, x_dis=None, y_dis=None):
        self.y_dis = int(y_dis if y_dis is not None else 260)
        self.x_dis = int(x_dis if x_dis is not None else 500)
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        # 进入夹取时不再用 IK 切检测位，保持寻路/24扫描给出的相机角度
        self.board.bus_servo_set_position(0.5, [[24, self.y_dis], [21, self.x_dis]])
        time.sleep(1)

        for attempt in range(self.attempts):
            self.log.debug('[grab] %s', action_msg('夹取尝试', action='第 %d/%d 次' % (attempt + 1, self.attempts)))
            self._status(2)

            r = self._stable()
            if r is None:
                self.log.info('[grab] %s', action_msg('目标丢失，进入重扫'))
                r = self._rescan()
                if r is None:
                    self.log.info('[grab] %s', action_msg('重扫后仍未找到目标'))
                    continue

            x, y = self._coord(r['center'])
            self.log.debug('[grab] %s', action_msg('坐标计算', action='像素=%s x=%.1fcm y=%.1fcm' % (r['center'], x, y)))

            x, y, z = self._target_xyz(r['center'], r.get('area', 0))
            self.log.debug('[grab] %s', action_msg('目标xyz', action='x=%.1f y=%.1f z=%.1f' % (x, y, z)))

            # 真实六足微调：y 太大就前进，y 太小就后退，直到 y 进入 [grab_y_min, grab_y_max]
            for _ in range(self.align_iter):
                if self.grab_y_min <= y <= self.grab_y_max:
                    break
                r2 = self.detect()
                if r2 is None:
                    break
                x, y, z = self._target_xyz(r2['center'], r2.get('area', 0))
                if y > self.grab_y_max:
                    self.log.debug('[grab] %s', action_msg('六足微调', reason='y=%.1f>%.1f 偏远' % (y, self.grab_y_max), action='前进 %dmm' % self.fine_step_mm))
                    go_forward(self.ik, self.fine_step_mm, 50)
                elif y < self.grab_y_min:
                    self.log.debug('[grab] %s', action_msg('六足微调', reason='y=%.1f<%.1f 偏近' % (y, self.grab_y_min), action='后退 %dmm' % self.fine_step_mm))
                    go_back(self.ik, self.fine_step_mm, 50)
                time.sleep(0.4)

            if not self._grab_once(x, y, z):
                self.log.info('[grab] %s', action_msg('夹取失败', reason='逆运动学无解'))
                continue
            self._status(3)
            self.log.info('[grab] %s', action_msg('夹取完成', reason='已执行 IK 夹取动作'))
            return True

        return False
