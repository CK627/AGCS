#!/usr/bin/python3
# coding=utf8
"""夹取算法 v2：双信号距离 + 高度分路（地面/高处）+ 距离中位数验证。

与 v1 的区别：
- 高度分路：-90 检测位能看到 → 地面目标，像素转坐标夹取；
            -90 看不到 → 高处目标，抬头 + 固定夹取点；
- 距离用超声波 + 像素转坐标双信号；
- 验证用 x/y 距离中位数 / 目标消失位移。
"""
import time

from common.pid import PID
from agcs_lib.motion import turn_left, turn_right
from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.sensors import show_status, dist_cm
from agcs_lib.logs import get_logger, action_msg


class GrabberV2:
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

        gc = params.get('grab', {})
        gf = params.get('gimbal_fetch', {})
        arm = params['arm']
        walk = params.get('walk', {})

        self.align_x_cm = float(gc.get('align_x_cm', 0.5))
        self.align_y_cm = float(gc.get('align_y_cm', 2.0))
        self.y_ref = float(gc.get('y_ref', 15.0))
        self.move_tol_px = int(gc.get('move_tol_px', 40))
        self.pos_ratio = float(gc.get('pos_ratio', 1.5))
        self.attempts = int(gc.get('attempts', 3))
        self.pick_z = float(arm.get('pick_z', -5))
        self.gripper_open = int(arm.get('gripper_open', 120))
        self.gripper_close = int(arm.get('gripper_close', 600))
        self.reach_x = float(walk.get('reach_x', 8.0))
        self.reach_y = float(walk.get('reach_y', 24.0))

        # 高处目标：抬头 + 固定夹取点
        self.tilt_pose = [float(v) for v in gc.get('tilt_pose', [0, 15, 30])]
        self.tilt_pitch = int(gc.get('tilt_pitch', 0))
        self.tilt_cx_min = int(gc.get('tilt_cx_min', 250))
        self.tilt_cx_max = int(gc.get('tilt_cx_max', 390))
        self.tilt_cy_min = int(gc.get('tilt_cy_min', 240))
        self.tilt_cy_max = int(gc.get('tilt_cy_max', 400))
        self.tilt_turn_deg = int(gc.get('tilt_turn_deg', 4))
        self.tilt_rounds = int(gc.get('tilt_rounds', 12))
        self.fixed_grab = [float(v) for v in gc.get('fixed_grab', [0, 25])]
        self.fixed_grab_z = float(gc.get('fixed_grab_z', 5))
        self.turn_sign = int(gf.get('turn_sign', 1))

    def _status(self, v):
        show_status(self.display, v)

    def _stable(self, frames=60, need=5, jitter=5):
        stable = 0
        old = None
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
            if stable >= need:
                return r
            time.sleep(0.05)
        return None

    def _coord(self, center):
        return pixel_to_arm_coord(self.K, self.R, self.T, center, initial_coord=(0, 15, 5))

    def _turn_body(self, deg):
        if deg == 0:
            return
        if self.turn_sign > 0:
            (turn_left if deg > 0 else turn_right)(self.ik, abs(deg), 60)
        else:
            (turn_right if deg > 0 else turn_left)(self.ik, abs(deg), 60)

    def _grab_at(self, x, y, z):
        """伸臂夹取（官方 intelligent_fetch：y+2 前伸、z 向下、夹爪闭合）。"""
        res = self.ak.setPitchRangeMoving((x, y + 2.0, z), -90, -90, 100, 2)
        if res is False:
            return False
        time.sleep(2)
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_close]])
        time.sleep(0.5)
        self.ak.setPitchRangeMoving((x, y, 8), -90, -90, 100, 1)
        time.sleep(1)
        self.ak.setPitchRangeMoving((12, 24, -5), -90, -90, 100, 1.5)
        time.sleep(1.5)
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)
        self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 1.5)
        time.sleep(1.5)
        return True

    def _verify(self, before):
        """夹取后判定：多次检测取 x/y 中位数；目标消失/明显位移/贴到爪边 → 夹到。"""
        xs, ys = [], []
        for _ in range(5):
            r = self.detect()
            if r is None:
                return True
            xs.append(r['center'][0])
            ys.append(r['center'][1])
            time.sleep(0.1)
        if not xs:
            return False
        mx = sorted(xs)[len(xs) // 2]
        my = sorted(ys)[len(ys) // 2]
        radius = 20
        tol = max(self.move_tol_px, int(self.pos_ratio * radius))
        if abs(mx - before[0]) > tol or abs(my - before[1]) > tol:
            return True
        if my >= 480 - 40:
            return True
        return False

    def run(self, center=None, cy=None):
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)

        # 先 -90 检测位尝试（地面目标）
        self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
        time.sleep(2)
        r = self._stable()
        if r is not None:
            x, y = self._coord(r['center'])
            ultra = dist_cm(self.ultrasonic) if self.ultrasonic else -1.0
            self.log.info('[grab] %s', action_msg(
                '地面目标', action='x=%.1fcm y=%.1fcm 超声波=%.1fcm' % (x, y, ultra)))
            if abs(x) <= self.reach_x and 6 <= y <= self.reach_y:
                if self._grab_at(x, y, self.pick_z):
                    if self._verify(r['center']):
                        self.log.info('[grab] %s', action_msg('夹取成功', reason='目标消失或明显位移'))
                        return True
            else:
                self.log.info('[grab] %s', action_msg('超出可及范围', reason='x=%.1fcm y=%.1fcm' % (x, y)))

        # -90 检测位看不到 → 抬头夹取（高处目标）
        self.log.info('[grab] %s', action_msg('转抬头夹取', reason='-90检测位未检测到或超出可及'))
        return self._run_tilted()

    def _run_tilted(self):
        self.ak.setPitchRangeMoving(tuple(self.tilt_pose), self.tilt_pitch, -90, 100, 2)
        time.sleep(2)
        self.log.info('[grab] %s', action_msg(
            '抬头检测', action='切抬头位 %s pitch=%d°' % (self.tilt_pose, self.tilt_pitch)))
        for round_no in range(self.tilt_rounds):
            r = self._stable(frames=30, need=3)
            if r is None:
                self.log.debug('[grab] %s', action_msg('抬头未检测到', action='转身 %d° 重找' % self.tilt_turn_deg))
                self._turn_body(self.tilt_turn_deg)
                time.sleep(0.6)
                continue
            cx, cy = r['center']
            self.log.debug('[grab] %s', action_msg(
                '抬头定位', action='第 %d/%d 轮 cx=%dpx cy=%dpx' % (round_no + 1, self.tilt_rounds, cx, cy)))
            if cx < self.tilt_cx_min:
                self._turn_body(self.tilt_turn_deg)
                time.sleep(0.4)
                continue
            if cx > self.tilt_cx_max:
                self._turn_body(-self.tilt_turn_deg)
                time.sleep(0.4)
                continue
            if self.tilt_cy_min <= cy <= self.tilt_cy_max:
                fx, fy = self.fixed_grab
                self.log.info('[grab] %s', action_msg(
                    '抬头夹取', action='固定夹取点 (%.1fcm, %.1fcm, %.1fcm)' % (fx, fy, self.fixed_grab_z)))
                if self._grab_at(fx, fy, self.fixed_grab_z):
                    self.log.info('[grab] %s', action_msg('夹取成功', reason='抬头目标进入窗口'))
                    return True
        return False
