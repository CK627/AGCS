#!/usr/bin/python3
# coding=utf8
"""夹取：search 定位目标，grab 前伸 + 画面占比判断夹取（alpha=0）。

search 到位后目标已在画面中心（云台对准），grab 不重新检测。夹持器水平
（alpha=0°）时，摄像头朝前下方看，目标在地面出现在画面下部。机械臂前伸，
摄像头靠近目标时目标在画面下部越来越大（摄像头能朝下看，目标不会消失）；
目标占画面比例达标 = 夹爪已到目标上方，直接夹取。

流程：
1. 前伸到起始位置（alpha=0，末端朝前）；
2. 动态前伸：占比离阈值越远步长越大、越近越小；占比达标 → 停止前伸 → 闭合夹爪；
3. 前伸到 IK 极限仍不够 → 直接夹取（目标可能超可及范围，已最靠近）；
4. 抬起（复位由调用方处理）。
"""
import time

from agcs_lib.sensors import show_status
from agcs_lib.logs import get_logger, action_msg


class Grabber:
    """detect: callable，返回 dict(center=(cx,cy), radius, area, color, contour) 或 None。"""

    def __init__(self, board, ik, ak, params, K, R, T, detect, display=None, ultrasonic=None, tof=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.detect = detect
        self.display = display
        self.tof = tof  # 红外(VL53L0X)，读垂直距离（mm）
        self.log = get_logger()

        gc = params.get('grab', {})
        arm = params['arm']

        # 前伸夹取参数
        self.coarse_z = float(gc.get('coarse_z', 15.0))
        self.reach_y_start = float(gc.get('reach_y_start', 28.0))
        self.grab_area_ratio = float(gc.get('grab_area_ratio', 0.30))
        self.reach_gain = float(gc.get('reach_gain', 20.0))          # 前伸增益 cm/占比
        self.reach_step_min = float(gc.get('reach_step_min', 0.5))   # 占比接近阈值时的小步
        self.reach_step_max = float(gc.get('reach_step_max', 3.0))   # 看不到/目标远时的大步
        self.lift_pulse = int(gc.get('lift_pulse', 50))              # 夹住后 22 号肩舵机上抬脉宽
        self.tof_grab_cm = float(gc.get('tof_grab_cm', 4.0))         # 红外读数 <= 此值（夹爪到色块顶面）夹取
        self.descend_step_cm = float(gc.get('descend_step_cm', 0.5))  # 下降步长 cm
        self.min_z = float(gc.get('min_z', 3.0))                     # 下降 z 下限 cm
        self.descend_max_steps = int(gc.get('descend_max_steps', 30))
        self.frame_area = 320 * 240  # detect() 在 320x240 上算面积，占比 = area / frame_area
        self.descend_move_ms = int(gc.get('descend_move_ms', 120))
        self.descend_settle_ms = int(gc.get('descend_settle_ms', 80))
        self.attempts = int(gc.get('attempts', 3))

        # 夹取动作参数
        self.gripper_open = int(arm.get('gripper_open', 120))
        self.gripper_close = int(arm.get('gripper_close', 550))
        self.raise_pose = tuple(float(v) for v in arm.get('raise_pose', [12, 24, 5]))
        self.detect_pose = tuple(float(v) for v in arm.get('detect_pose', [0, 15, 5]))
        self.K, self.R, self.T = K, R, T

        self._z = self.coarse_z  # 当前末端 z（cm），下降过程维护

    def _status(self, v):
        show_status(self.display, v)

    def _move(self, coord, move_ms=None, settle_ms=None):
        """末端移到 coord=(x,y,z)，alpha=0°（夹持器水平 → 红外在夹爪下方垂直朝下）。IK 无解或俯仰降级返回 False。"""
        if move_ms is None:
            move_ms = self.descend_move_ms
        if settle_ms is None:
            settle_ms = self.descend_settle_ms
        res = self.ak.setPitchRangeMoving(coord, 0, -90, 100, move_ms / 1000.0)  # ms → 秒
        time.sleep(settle_ms / 1000.0)
        if res is False:
            return False  # 逆运动学无解
        # 强制夹持器水平：alpha=0 无解时 IK 会降级成斜俯仰（alpha != 0），
        # 斜了红外就不垂直朝下（打到机械臂自己/地面偏了），这里一律判失败
        if abs(res[1]) > 0.5:
            self.log.info('[grab] %s', action_msg(
                '俯仰角降级', reason='目标%s 夹持器水平无解，IK 选了 alpha=%.1f°' % (tuple(coord), res[1])))
            return False
        return res[0]  # servos dict（含 servo22 当前脉宽，夹住后 +lift_pulse 上抬）

    def _tof_height_cm(self):
        """红外测夹爪离地高度（cm），中位数采样，失败/超范围返回 None。"""
        if self.tof is None:
            return None
        vals = []
        for _ in range(5):
            try:
                d = self.tof.range
                if 0 < d < 8000:
                    vals.append(d)
            except Exception:
                pass
            time.sleep(0.01)
        if not vals:
            return None
        vals.sort()
        return vals[len(vals) // 2] / 10.0

    def _descend(self, y):
        """前伸到位后下降：红外测夹爪离地高度，降到色块顶面附近返回 True。"""
        z = self.coarse_z
        for _ in range(self.descend_max_steps):
            d = self._tof_height_cm()
            if d is not None and d <= self.tof_grab_cm:
                self.log.info('[grab] %s', action_msg(
                    '下降到夹取高度', action='红外离地 %.1fcm <= %.1fcm' % (d, self.tof_grab_cm)))
                return True
            z -= self.descend_step_cm
            if z < self.min_z:
                break
            if not self._move((0.0, y, z), move_ms=self.descend_move_ms, settle_ms=self.descend_settle_ms):
                break
        return False

    def _reach(self):
        """前伸 + 画面占比闭环：占比离阈值越远步长越大、越近越小；占比达标即夹。返回是否夹取。"""
        # 先前伸到扫描起始位置（alpha=0，末端朝前），避免从收拢位大幅跳变
        last_servos = self._move((0.0, self.reach_y_start, self.coarse_z), move_ms=800, settle_ms=500)
        y = self.reach_y_start
        while True:
            r = self.detect()
            if r is None:
                area, ratio, cx, cy = 0, 0.0, 0, 0
            else:
                area = int(r.get('area', 0))
                ratio = area / self.frame_area
                cx, cy = r.get('center', (0, 0))
                if ratio >= self.grab_area_ratio:
                    # 目标占画面比例达标 = 夹爪已到目标上方，先下降到色块高度，再夹取
                    self.log.info('[grab] %s', action_msg(
                        '目标占比达标', reason='占比 %.1f%% >= %.1f%%' % (ratio * 100, self.grab_area_ratio * 100),
                        action='下降到色块高度再夹取'))
                    self._descend(y)
                    self.board.bus_servo_set_position(0.5, [[25, self.gripper_close]])
                    time.sleep(1.0)
                    if last_servos:
                        self.board.bus_servo_set_position(0.5, [[22, last_servos["servo22"] + self.lift_pulse]])
                        time.sleep(0.5)
                    self.log.info('[grab] %s', action_msg('夹取执行完成'))
                    return True
            # 根据画面占比动态前伸：占比离阈值越远步长越大、越近越小（逐渐逼近，不伸过头）
            gap = max(0.0, self.grab_area_ratio - ratio)
            step = max(self.reach_step_min, min(self.reach_step_max, gap * self.reach_gain))
            self.log.info('[grab] %s', action_msg(
                '前伸扫描', action='y=%.1fcm 面积=%d 占比=%.1f%% cy=%d → 前伸 %.1fcm' % (y, area, ratio * 100, cy, step)))
            y += step
            res = self._move((0.0, y, self.coarse_z), move_ms=400, settle_ms=300)
            if res is False:
                # 前伸到极限（IK 无解），已经最靠近目标，直接夹取
                self.log.info('[grab] %s', action_msg('前伸到极限', action='y=%.1fcm 直接夹取' % y))
                self.board.bus_servo_set_position(0.5, [[25, self.gripper_close]])
                time.sleep(1.0)
                if last_servos:
                    self.board.bus_servo_set_position(0.5, [[22, last_servos["servo22"] + self.lift_pulse]])
                    time.sleep(0.5)
                self.log.info('[grab] %s', action_msg('夹取执行完成'))
                return True
            last_servos = res

    def run(self):
        self.board.bus_servo_set_position(0.5, [[25, self.gripper_open]])
        time.sleep(0.5)

        for attempt in range(self.attempts):
            self._status(2)
            self.log.info('[grab] %s', action_msg(
                '前伸夹取', action='第 %d/%d 次' % (attempt + 1, self.attempts)))

            # 前伸 + 面积占比判断，目标消失直接夹取（_reach 内闭合夹爪 + 22 号上抬）
            if self._reach():
                self._status(3)
                return True
            self.log.info('[grab] %s', action_msg('前伸未定位目标'))

        return False
