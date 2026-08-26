#!/usr/bin/python3
# coding=utf8
"""夹取（纯机械臂，官方 block_fetch / intelligent_fetch 逻辑）。

只动 21-25，不走路：像素转坐标 → 可及检查 → 伸臂夹取 → 验证。
"""
import time

from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.sensors import show_status
from agcs_lib.logs import get_logger, action_msg


class Grabber:
    """纯机械臂夹取。"""

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
        arm = params['arm']
        walk = params.get('walk', {})

        self.attempts = int(gc.get('attempts', 3))
        self.move_tol_px = int(gc.get('move_tol_px', 40))
        self.pos_ratio = float(gc.get('pos_ratio', 1.5))
        self.pick_z = float(arm.get('pick_z', -5))
        self.gripper_open = int(arm.get('gripper_open', 120))
        self.gripper_close = int(arm.get('gripper_close', 550))
        self.reach_x = float(walk.get('reach_x', 8.0))
        self.reach_y = float(walk.get('reach_y', 24.0))

    def _status(self, v):
        show_status(self.display, v)

    def _detail(self, msg):
        get_logger().debug('[grab] %s', msg)

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

    def _grab_once(self, x, y, z=None):
        """伸臂夹取（官方 intelligent_fetch 逻辑：y+2 前伸、z 向下、夹爪闭合）。"""
        if z is None:
            z = self.pick_z
        res = self.ak.setPitchRangeMoving((x, y + 2.0, z), -90, -90, 100, 1)
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
        """夹取后判定：目标消失或明显位移 → 夹到了。"""
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

    def run(self, center=None):
        """像素转坐标 → 可及检查 → 伸臂夹取 → 验证。只动 21-25，不走路。"""
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)

        for attempt in range(self.attempts):
            self.log.debug('[grab] %s', action_msg('夹取尝试', action='第 %d/%d 次' % (attempt + 1, self.attempts)))
            self._status(2)

            # 夹前复检：优先用 search 给的中心，再复检一次取最新
            if center is not None:
                r = self.detect()
                if r is not None:
                    center = r['center']
            if center is None:
                r = self._stable()
                if r is None:
                    self.log.info('[grab] %s', action_msg('夹取失败', reason='目标不可见'))
                    return False
                center = r['center']

            x, y = self._coord(center)
            self.log.debug('[grab] %s', action_msg('坐标计算', action='x=%.1fcm y=%.1fcm z=%.1fcm' % (x, y, self.pick_z)))

            if abs(x) > self.reach_x or y > self.reach_y or y < 6:
                self.log.info('[grab] %s', action_msg('夹取失败', reason='超出可及范围 x=%.1fcm y=%.1fcm' % (x, y)))
                return False

            if not self._grab_once(x, y, self.pick_z):
                self.log.info('[grab] %s', action_msg('夹取失败', reason='逆运动学无解'))
                return False

            if self._verify(center):
                self._status(3)
                self.log.info('[grab] %s', action_msg('夹取成功'))
                return True

            self.log.info('[grab] %s', action_msg('未夹到', reason='目标未移动', action='重试伸臂 第 %d/%d 次' % (attempt + 1, self.attempts)))
            self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 1.5)
            time.sleep(1.0)

        self._status(0)
        self.log.info('[grab] %s', action_msg('夹取失败', reason='%d 次尝试均未夹到' % self.attempts))
        return False
