#!/usr/bin/python3
# coding=utf8
"""夹取（纯机械臂）：只动 21-25（臂 21-24 + 夹爪 25），绝不动 1-20。

前置条件由 bgas（Before-Grab-After-Search）保证，并按策略执行：
- strategy='mapped'（目标在 -90 检测位可见，地面/低处）：
  稳定检测 → 地面标定映射算 (x,y,z) → 伸臂夹取 → -90 位验证；
- strategy='fixed'（目标在 -90 不可见，高处平台）：
  稳定检测（倾斜位）→ 确认 cx/cy 在就绪窗口 → 伸臂到固定点 → 切回倾斜位验证。

失败时不动腿，返回 False 交由上层决定（重新 bgas 或放弃）。
"""
import os
import time

from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.sensors import show_status
from agcs_lib.logs import get_logger


class Grabber:
    """纯机械臂夹取。"""

    def __init__(self, board, ik, ak, params, K, R, T, detect, display=None, ultrasonic=None):
        self.board = board
        self.ik = ik   # 保留引用（签名兼容），grab 不使用 1-20
        self.ak = ak
        self.params = params
        self.K, self.R, self.T = K, R, T
        self.detect = detect
        self.display = display
        self.ultrasonic = ultrasonic  # 仅兼容旧调用，grab 不使用

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

        self.body_z = 0  # 由 bgas 传入，仅用于日志

    # ---------- 工具 ----------
    def _status(self, v):
        show_status(self.display, v)

    def _detail(self, msg):
        """详细原因，只写日志文件（debug 级别，不打印终端）。"""
        get_logger().debug('[grab] %s', msg)

    def _stable(self, frames=60, need=5, jitter=5):
        """稳定检测，返回 detect dict 或 None。"""
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

    # ---------- 夹取动作（只动 21-25） ----------
    def _grab_once(self, x, y, z=None):
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

    # ---------- 主流程 ----------
    def run(self, context=None, body_z=0):
        """纯机械臂夹取。context 为 bgas.run() 返回的可抓取上下文（可为 None）。

        只做：稳定检测 → 坐标（映射或固定点）→ 伸臂夹取 → 验证；不走路、不升降、不找回。
        返回 True=夹取成功；False=失败（不动 1-20，交由上层重新 bgas）。
        """
        self.body_z = int(body_z or 0)
        strategy = 'mapped'
        if context is not None:
            strategy = context.get('strategy', 'mapped')
        self._detail('grab 开始：strategy=%s 机身=%+dmm；只动 21-25，不调整 1-20' % (strategy, self.body_z))
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)

        for attempt in range(self.attempts):
            print('[grab] 第 %d/%d 次（%s）' % (attempt + 1, self.attempts, strategy))
            self._status(2)
            r = self._stable()
            if r is None:
                print('[grab] 目标不可见，grab 失败', flush=True)
                self._detail('grab 失败：稳定检测未找到目标；grab 不调整 1-20，交由上层重新 bgas')
                return False
            center = r['center']

            if strategy == 'mapped':
                x, y = self._coord(center)
                z_grab = self.pick_z
                print('[grab] 像素=%s -> x=%.1f cm y=%.1f cm（机身=%+dmm cy=%d z=%.1f）'
                      % (center, x, y, self.body_z, center[1], z_grab))
                self._detail('坐标计算：地面标定映射 pixel_to_arm_coord 得 x/y=%.1f/%.1f，z=pick_z=%.1f'
                             % (x, y, z_grab))
                if abs(x) > self.reach_x or y > self.reach_y or y < 0:
                    print('[grab] 超出可及范围 x=%.1f cm y=%.1f cm，grab 失败' % (x, y))
                    self._detail('grab 失败：x=%.1f y=%.1f 超出可及范围（reach_x=%.0f reach_y=%.0f）；'
                                 'grab 不走路，交由上层重新 bgas' % (x, y, self.reach_x, self.reach_y))
                    return False
            else:
                x = float(context.get('x', 0.0))
                y = float(context.get('y', 15.0))
                z_grab = float(context.get('z', self.pick_z))
                cx_min = int(context.get('cx_min', 250))
                cx_max = int(context.get('cx_max', 390))
                cy_min = int(context.get('cy_min', 330))
                cy_max = int(context.get('cy_max', 430))
                cx, cy = center
                if not (cx_min <= cx <= cx_max and cy_min <= cy <= cy_max):
                    print('[grab] 目标 cx=%d cy=%d 不在倾斜位就绪窗口内，grab 失败' % (cx, cy))
                    self._detail('grab 失败（fixed）：cx=%d cy=%d 不在窗口 [%d-%d, %d-%d]，'
                                 'bgas 后目标漂移；grab 不走路，交由上层重新 bgas'
                                 % (cx, cy, cx_min, cx_max, cy_min, cy_max))
                    return False
                print('[grab] fixed：目标 cx=%d cy=%d 在就绪窗口，伸臂到固定点 (%.1f, %.1f, %.1f)'
                      % (cx, cy, x, y, z_grab))
                self._detail('坐标：固定夹取点 (%.1f, %.1f, %.1f)（bgas 倾斜闭环已把目标对准窗口）'
                             % (x, y, z_grab))

            self._detail('伸出机械臂：setPitchRangeMoving 到 (%.1f, %.1f, %.1f) 夹取'
                         % (x, y + 2.0, z_grab))
            if not self._grab_once(x, y, z_grab):
                print('[grab] 逆运动学无解，grab 失败')
                self._detail('grab 失败：目标点 (%.1f, %.1f, %.1f) 逆运动学无解，交由上层重新 bgas'
                             % (x, y, z_grab))
                return False

            # 验证：fixed 路线需把相机切回倾斜位才能看到方块原位置
            tilt_pose = (0.0, 15.0, 18.0)
            tilt_pitch = -35
            if strategy == 'fixed':
                tilt_pose = tuple(float(v) for v in context.get('tilt_pose', [0, 15, 18]))
                tilt_pitch = int(context.get('tilt_pitch', -35))
                self._detail('夹取后切回倾斜位 pose=%s pitch=%d 做验证（相机需看见方块原位置）'
                             % (tilt_pose, tilt_pitch))
                self.ak.setPitchRangeMoving(tilt_pose, tilt_pitch, -90, 100, 1.5)
                time.sleep(1.5)

            if self._verify(center):
                self._status(3)
                print('[grab] 夹取成功')
                self._detail('夹取验证通过：夹取后目标消失/明显位移，判定已夹到')
                return True

            print('[grab] 目标未移动，未夹到，重试伸臂（第 %d/%d 次）' % (attempt + 1, self.attempts))
            self._detail('夹取验证失败：目标仍在原位；grab 只重试伸臂，不调整 1-20')
            if strategy == 'fixed':
                self.ak.setPitchRangeMoving(tilt_pose, tilt_pitch, -90, 100, 1.5)
            else:
                self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 1.5)
            time.sleep(1.0)

        self._status(0)
        print('[grab] %d 次尝试均未夹到，grab 失败' % self.attempts)
        self._detail('grab 失败：%d 次伸臂重试均未夹到，交由上层重新 bgas' % self.attempts)
        return False
