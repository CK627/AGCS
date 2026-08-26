#!/usr/bin/python3
# coding=utf8
"""夹取算法（二开）：体态升降 + 24舵机参与 + 重扫锁定 + 纯视觉夹取 + 判定 + 重试。"""
import time

from common.pid import PID
from agcs_lib.motion import move_body, go_forward, go_back
from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.sensors import show_status
from agcs_lib.logs import get_logger, action_msg


class Grabber:
    """detect: callable，返回 detect_color 的 dict 或 None。"""

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
        self.body_step = int(gc.get('body_lift_step', 5))
        self.body_max = int(gc.get('body_lift_max', 30))
        self.height_iter = int(gc.get('height_max_iter', 12))
        self.cy_target = int(gc.get('cy_target', 240))
        self.cy_tol = int(gc.get('cy_tol', 50))
        self.cy_sign = int(gc.get('cy_sign', 1))
        self.height_enabled = bool(gc.get('height_enabled', True))
        self.arm_tilt_step = int(gc.get('arm_tilt_step', 10))

        self.pick_z = float(arm.get('pick_z', -5))
        self.gripper_open = int(arm.get('gripper_open', 120))
        self.gripper_close = int(arm.get('gripper_close', 600))
        self.reach_x = float(walk.get('reach_x', 8.0))
        self.reach_y = float(walk.get('reach_y', 24.0))
        self.body_z = 0

        # 摄像头舵机（21 水平 / 24 俯仰），用于调整后的重扫锁定
        self.x_dis = 500
        self.y_dis = 260
        self.x_pid = PID(P=0.1, I=0.001, D=0.008)
        self.y_pid = PID(P=0.1, I=0.02, D=0.008)
        self.pan_step = int(gf.get('pan_step', 40))
        self.settle = float(gf.get('settle_ms', 280)) / 1000.0
        self.v_scan = [260, 200, 320, 140, 380, 80, 440]

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
                return c
            time.sleep(0.05)
        return None

    def set_body(self, dz, reason=None):
        dz = max(-self.body_max, min(self.body_max, int(dz)))
        move_body(self.ik, dz)
        self.body_z = dz
        self.log.debug('[grab] %s', action_msg('机身调整', reason=reason, action='%+dmm' % dz))
        time.sleep(0.3)
        return dz

    def _cy_ok(self, cy):
        return abs(cy - self.cy_target) <= self.cy_tol

    def _cam(self):
        self.board.bus_servo_set_position(0.02, [[24, int(self.y_dis)], [21, int(self.x_dis)]])

    def _set_cam(self, x, y):
        self.x_dis = max(0, min(1000, int(x)))
        self.y_dis = max(0, min(1000, int(y)))
        self._cam()
        time.sleep(self.settle)

    def _track(self, center):
        cx, cy = center
        self.x_pid.SetPoint = 320
        self.x_pid.update(cx)
        self.y_pid.SetPoint = 240
        self.y_pid.update(cy)
        self._set_cam(self.x_dis + self.x_pid.output, self.y_dis + self.y_pid.output)
        return abs(cx - 320) < 30 and abs(cy - 240) < 30

    def _rescan(self):
        """21+24 双轴扫描找回目标。"""
        pans = list(range(500 + self.pan_step, 1001, self.pan_step)) + \
               list(range(1000, -1, -self.pan_step)) + \
               list(range(0, 500, self.pan_step))
        for y in self.v_scan:
            self._set_cam(500, y)
            for x in pans:
                self._set_cam(x, y)
                r = self.detect()
                if r is not None:
                    return r
        return None

    def adjust_height(self, cy):
        """体态(1-20)优先，到极限用 24 舵机；每次调整后重扫并锁定。返回 (ok, alarm)。"""
        self.log.info('[grab] %s', action_msg(
            '开始高度自适应', reason='cy=%dpx 偏离目标 %dpx' % (cy, self.cy_target),
            action='cy=%dpx 目标cy=%dpx' % (cy, self.cy_target)))
        cur = cy
        for i in range(self.height_iter):
            if self._cy_ok(cur):
                return True, False
            step = self.body_step * self.cy_sign
            if cur < self.cy_target:
                if self.body_z + step <= self.body_max:
                    self.set_body(self.body_z + step, reason='cy=%dpx 低于目标（目标偏高/远），抬高机身' % cur)
                else:
                    self._set_cam(self.x_dis, self.y_dis + self.arm_tilt_step)
            else:
                if self.body_z - step >= -self.body_max:
                    self.set_body(self.body_z - step, reason='cy=%dpx 高于目标（目标偏低/近），降低机身' % cur)
                else:
                    self._set_cam(self.x_dis, self.y_dis - self.arm_tilt_step)
            r = self.detect()
            if r is None:
                self._set_cam(self.found_x, self.found_y)
                r = self.detect()
            if r is None:
                r = self._rescan()
            if r is None:
                return False, True
            self._track(r['center'])
            cur = r['center'][1]
            time.sleep(0.25)
        return False, True

    def _coord(self, center):
        return pixel_to_arm_coord(self.K, self.R, self.T, center, initial_coord=(0, 15, 5))

    def _aligned(self, x, y):
        return abs(x) <= self.align_x_cm and abs(y - self.y_ref) <= self.align_y_cm

    def _grab_once(self, x, y):
        res = self.ak.setPitchRangeMoving((x, y + 2.0, self.pick_z), -90, -90, 100, 1)
        if res is False:
            return False
        time.sleep(1.5)
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_close]])
        time.sleep(1.5)
        self.ak.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1.5)
        time.sleep(1.5)
        self.ak.setPitchRangeMoving((12, 24, -5), -90, -90, 100, 1)
        time.sleep(1)
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)
        self.ak.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1)
        time.sleep(1)
        self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 1.5)
        time.sleep(1.5)
        return True

    def _verify(self, before):
        for _ in range(5):
            r = self.detect()
            if r is None:
                return True
            c = r['center']
            radius = max(int(r.get('radius', 20)), 1)
            tol = max(self.move_tol_px, int(self.pos_ratio * radius))
            if abs(c[0] - before[0]) > tol or abs(c[1] - before[1]) > tol:
                return True
            time.sleep(0.1)
        return False

    def run(self, cy=None, x_dis=None, y_dis=None):
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        if x_dis is not None:
            self.x_dis = x_dis
        if y_dis is not None:
            self.y_dis = y_dis
        self.found_x = self.x_dis
        self.found_y = self.y_dis

        if self.height_enabled and cy is not None and not self._cy_ok(cy):
            ok, alarm = self.adjust_height(cy)
            if alarm:
                self.log.info('[grab] %s', action_msg(
                    '高度未达标', reason='cy=%dpx 目标cy=%dpx' % (cy, self.cy_target),
                    action='保持当前体态继续夹取'))

        # 切回检测位（相机朝下 pitch -90），保证 pixel_to_arm_coord 的标定姿态一致
        self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
        time.sleep(2)

        for attempt in range(self.attempts):
            self.log.debug('[grab] %s', action_msg('夹取尝试', action='第 %d/%d 次' % (attempt + 1, self.attempts)))
            self._status(2)
            center = self._stable()
            if center is None:
                self.log.info('[grab] %s', action_msg('未检测到目标', reason='稳定检测未找到目标'))
                continue

            x, y = self._coord(center)
            self.log.debug('[grab] %s', action_msg('坐标计算', action='像素=%s x=%.1fcm y=%.1fcm' % (center, x, y)))

            # 对齐微调：y 偏离 y_ref 则前进/后退
            for _ in range(20):
                if self._aligned(x, y):
                    break
                r = self.detect()
                if r is None:
                    break
                x, y = self._coord(r['center'])
                if y > self.y_ref + self.align_y_cm:
                    self.log.debug('[grab] %s', action_msg('对齐', reason='y=%.1fcm 偏远' % y, action='前进 12mm'))
                    go_forward(self.ik, 12, 50)
                elif y < self.y_ref - self.align_y_cm:
                    self.log.debug('[grab] %s', action_msg('对齐', reason='y=%.1fcm 偏近' % y, action='后退 12mm'))
                    go_back(self.ik, 12, 50)
                time.sleep(0.4)

            if abs(x) > self.reach_x or y > self.reach_y or y < 0:
                self.log.info('[grab] %s', action_msg('超出可及范围', reason='x=%.1fcm y=%.1fcm' % (x, y), action='中止本次'))
                continue

            if not self._grab_once(x, y):
                self.log.info('[grab] %s', action_msg('夹取失败', reason='逆运动学无解', action='中止本次'))
                continue

            if self._verify(center):
                self._status(3)
                self.log.info('[grab] %s', action_msg('夹取成功', reason='目标消失或明显位移'))
                return True
            self.log.info('[grab] %s', action_msg('未夹到', reason='目标未移动', action='回到靠近位置'))
            self.set_body(0)
            self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
            time.sleep(1.5)

        return False
