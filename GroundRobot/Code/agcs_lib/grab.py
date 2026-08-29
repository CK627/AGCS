#!/usr/bin/python3
# coding=utf8
"""夹取：红外粗定位 + 面积阈值精夹取。

流程：
1. 机械臂缩回（抬头），红外测距算目标高度 z（粗定位，目标不动只测一次）；
2. 机械臂切检测位（pitch=-90），pixel_to_arm_coord 算 x,y（地面假设）；
3. 机械臂 IK 直接到 (x,y,z) 目标附近；
4. 面积阈值精夹取：面积 < 阈值就下调 z（机械臂下降靠近），面积到阈值就夹取。
"""
import time
import math

from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.sensors import show_status
from agcs_lib.logs import get_logger, action_msg


class Grabber:
    """detect: callable，返回 detect_color 的 dict 或 None。"""

    def __init__(self, board, ik, ak, params, K, R, T, detect, display=None, ultrasonic=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.detect = detect
        self.display = display
        self.log = get_logger()

        gc = params.get('grab', {})
        arm = params['arm']

        # 精夹取面积阈值（320x240 检测）
        self.servo_area_threshold = float(gc.get('servo_area_threshold', 1500.0))
        self.servo_z_step = float(gc.get('servo_z_step', 0.5))       # 精夹取下调 z 步长 cm
        self.servo_z_tries = int(gc.get('servo_z_tries', 10))         # 精夹取最大下调次数

        self.pick_z = float(arm.get('pick_z', -4))
        self.gripper_open = int(arm.get('gripper_open', 120))
        self.gripper_close = int(arm.get('gripper_close', 550))
        self.raise_pose = [float(v) for v in arm.get('raise_pose', [12, 24, 5])]
        self.detect_pose = [float(v) for v in arm.get('detect_pose', [0, 15, 5])]
        self.K, self.R, self.T = K, R, T

        # 红外算 z 参数
        self.tof_height = float(gc.get('tof_height', 13.5))     # 红外离地高度 cm
        self.tof_forward = float(gc.get('tof_forward', 30.0))   # 红外到夹爪水平距离 cm
        self.tof = None
        try:
            import board as adafruit_board
            import busio
            import adafruit_vl53l0x
            i2c = busio.I2C(adafruit_board.SCL, adafruit_board.SDA)
            self.tof = adafruit_vl53l0x.VL53L0X(i2c)
            self.log.info('[grab] %s', action_msg('红外初始化成功'))
        except Exception as e:
            self.log.info('[grab] %s', action_msg('红外初始化失败', reason=str(e)))

    def _status(self, v):
        show_status(self.display, v)

    def _stable(self, frames=60, need=5, jitter=5):
        """连续多帧目标坐标稳定才返回检测结果，否则 None。"""
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

    def _rescan(self):
        """24 号舵机 200-800 平滑扫描找目标（20 步长平滑，21 号保持不动）。"""
        self.log.info('[grab] %s', action_msg('重扫找回目标', action='24号 200-800 平滑扫描'))
        y = 200
        for target in (200, 400, 600, 800):
            step = 20
            direction = 1 if target >= y else -1
            while y != target:
                y += direction * step
                if direction > 0:
                    y = min(y, target)
                else:
                    y = max(y, target)
                self.board.bus_servo_set_position(0.05, [[24, y]])
                r = self.detect()
                if r is not None:
                    return r
                time.sleep(0.05)
        return None

    def _coord(self, center):
        return pixel_to_arm_coord(self.K, self.R, self.T, center, initial_coord=(0, 15, 5))

    def _tof_distance_cm(self):
        """红外测距（cm），失败/超范围返回 None。"""
        if self.tof is None:
            return None
        try:
            d_mm = self.tof.range
            if 0 < d_mm < 8000:
                return d_mm / 10.0
        except Exception:
            pass
        return None

    def run(self, cy=None, x_dis=None, y_dis=None):
        # 1. 红外粗定位：机械臂缩回（抬头，不挡红外），测 d 算目标高度 z
        self.ak.setPitchRangeMoving((10, 15, 30), 0, -90, 100, 1)
        time.sleep(1)
        d = self._tof_distance_cm()
        if d is not None:
            inner = d * d - self.tof_forward * self.tof_forward
            z = self.tof_height - math.sqrt(inner if inner > 0 else 0)
        else:
            z = self.pick_z  # 红外失败（地面目标），用 pick_z
        self.log.info('[grab] %s', action_msg(
            '红外粗定位', action='红外=%.1fcm 目标高度z=%.1fcm' % (d if d is not None else -1, z)))

        # 2. 保持抬头姿态（不切 -90，避免 24 号朝下看不到前方目标），检测目标算 x,y
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)

        for attempt in range(3):
            self._status(2)
            r = self._stable()
            if r is None:
                self.log.info('[grab] %s', action_msg('未检测到目标', action='24号重扫找回'))
                r = self._rescan()
                if r is None:
                    continue
            center = r['center']
            x, y = self._coord(center)
            self.log.info('[grab] %s', action_msg(
                '目标坐标', action='像素=%s x=%.1fcm y=%.1fcm z=%.1fcm' % (center, x, y, z)))

            # 3. 机械臂到目标附近（粗定位）
            res = self.ak.setPitchRangeMoving((x, y, z), -90, -90, 100, 1)
            if res is False:
                self.log.info('[grab] %s', action_msg('粗定位失败', reason='逆运动学无解'))
                continue
            time.sleep(2)

            # 4. 面积阈值精夹取：面积不够就下调 z，面积到阈值夹取
            for i in range(self.servo_z_tries):
                r2 = self.detect()
                if r2 is None:
                    continue
                area = r2.get('area', 0)
                self.log.debug('[grab] %s', action_msg('精夹取', action='面积=%d 阈值=%d z=%.1f' % (area, self.servo_area_threshold, z)))
                if area >= self.servo_area_threshold:
                    break
                z -= self.servo_z_step
                self.ak.setPitchRangeMoving((x, y, z), -90, -90, 100, 0.5)
                time.sleep(0.5)

            # 夹取动作 + 全程采样判定：目标跟着夹爪走（抬起→放下全程可见）才算夹中
            samples = []

            def sample(duration):
                end_time = time.time() + duration
                while time.time() < end_time:
                    r = self.detect()
                    samples.append(r['center'] if r is not None else None)
                    time.sleep(0.15)

            self.board.bus_servo_set_position(0.5, [[25, self.gripper_close]])  # 夹爪闭合
            time.sleep(1.0)
            self.ak.setPitchRangeMoving(tuple(self.raise_pose), -90, -90, 100, 1.5)  # 抬起
            sample(1.5)
            self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])  # 松开
            sample(0.8)
            self.ak.setPitchRangeMoving(tuple(self.detect_pose), -90, -90, 100, 1.5)  # 复位
            sample(1.5)

            seen = [s for s in samples if s is not None]
            self.log.info('[grab] %s', action_msg(
                '夹取过程采样', action='样本=%d 可见=%d' % (len(samples), len(seen))))
            # 目标从夹取到放下全程可见 → 夹中；任何一帧丢失 → 失败
            if len(seen) >= 5 and len(seen) == len(samples):
                self._status(3)
                self.log.info('[grab] %s', action_msg('夹取成功', reason='目标从夹取到放下全程可见'))
                return True
            self.log.info('[grab] %s', action_msg('夹取失败', reason='目标在夹取-放下过程中丢失'))
            continue

        return False
