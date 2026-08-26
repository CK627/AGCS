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
from agcs_lib.motion import move_body, go_forward, go_back, turn_left, turn_right
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
        self.body_step = int(gc.get('body_lift_step', 5))
        self.body_max = int(gc.get('body_lift_max', 70))
        self.body_min = int(gc.get('body_lift_min', 0))
        self.height_iter = int(gc.get('height_max_iter', 12))
        self.height_enabled = bool(gc.get('height_enabled', True))
        self.body_z = 0

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

    def set_body(self, dz, reason=None):
        """体态（1-20）升降，dz 为相对初始站姿的高度偏移 mm，正=升高。"""
        dz = max(self.body_min, min(self.body_max, int(dz)))
        move_body(self.ik, dz)
        self.body_z = dz
        self.log.info('[grab] %s', action_msg('机身调整', reason=reason, action='%+dmm' % dz))
        time.sleep(0.4)
        return dz

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

    def _align_to_reach(self, x, y, max_iter=12):
        """可及范围闭环微调：y 偏远→前进、y 偏近→后退、|x| 偏远→转身。

        保持 -90 检测位定点夹取，不轻易转抬头固定点。返回 (ok, x, y)。
        """
        for _ in range(max_iter):
            if abs(x) <= self.reach_x and 6 <= y <= self.reach_y:
                return True, x, y
            # 先处理横向偏远（转身），再处理前后距离（前进/后退）
            if abs(x) > self.reach_x:
                deg = 4 if x > 0 else -4
                self.log.info('[grab] %s', action_msg(
                    '横向偏远', reason='x=%.1fcm' % x, action='转身 %+d°' % deg))
                self._turn_body(deg)
            elif y > self.reach_y:
                self.log.info('[grab] %s', action_msg(
                    '偏远', reason='y=%.1fcm 超出可及上限 %.1fcm' % (y, self.reach_y), action='前进 12mm'))
                go_forward(self.ik, 12, 50)
            elif y < 6:
                self.log.info('[grab] %s', action_msg(
                    '偏近', reason='y=%.1fcm' % y, action='后退 12mm'))
                go_back(self.ik, 12, 50)
            time.sleep(0.6)
            r = self._stable(frames=20, need=2)
            if r is None:
                return False, x, y
            x, y = self._coord(r['center'])
        return (abs(x) <= self.reach_x and 6 <= y <= self.reach_y), x, y

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

        # 复位体态（若之前抬过）
        if self.body_z != 0:
            self.set_body(0, reason='复位体态到初始站姿')

        # 切检测位（官方 block_fetch 定点夹取的标定姿态）
        self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
        time.sleep(2)

        for attempt in range(self.attempts):
            self.log.info('[grab] %s', action_msg('夹取尝试', action='第 %d/%d 次' % (attempt + 1, self.attempts)))

            # 稳定检测（-90 检测位）
            r = self._stable()
            if r is None:
                # 检测位看不到：先体态抬升找回，仍无则放弃本次
                self.log.info('[grab] %s', action_msg('检测位未检测到目标', reason='目标可能在高处', action='体态抬升找回'))
                r = self._height_search()
                if r is None:
                    continue
                # 抬身找回后复位体态再检测，保证像素转坐标标定准确；
                # 复位后仍看不到说明是高处目标，交给抬头夹取。
                self.set_body(0, reason='找回后复位体态')
                r2 = self._stable(frames=20, need=2)
                if r2 is None:
                    self.log.info('[grab] %s', action_msg('复位后仍看不到目标', reason='目标在高处', action='转抬头夹取'))
                    return self._run_tilted()
                r = r2

            x, y = self._coord(r['center'])
            ultra = dist_cm(self.ultrasonic) if self.ultrasonic else -1.0
            self.log.info('[grab] %s', action_msg(
                '目标坐标', action='x=%.1fcm y=%.1fcm 超声波=%.1fcm' % (x, y, ultra)))

            # 可及范围闭环微调（保持定点夹取，不轻易转抬头固定点）
            ok, x, y = self._align_to_reach(x, y)
            if not ok:
                self.log.info('[grab] %s', action_msg('未进入可及范围', reason='x=%.1fcm y=%.1fcm' % (x, y)))
                continue

            # 定点夹取
            if not self._grab_at(x, y, self.pick_z):
                self.log.info('[grab] %s', action_msg('夹取失败', reason='逆运动学无解'))
                continue

            if self._verify(r['center']):
                self.log.info('[grab] %s', action_msg('夹取成功', reason='目标消失或明显位移'))
                return True
            self.log.info('[grab] %s', action_msg('未夹到', reason='目标未移动', action='复位重试'))
            self.set_body(0)
            self.ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
            time.sleep(1.5)

        # 兜底：抬头夹取（高处目标）
        self.log.info('[grab] %s', action_msg('转抬头夹取', reason='检测位多次未夹到，目标可能在高处'))
        return self._run_tilted()

    def _height_search(self):
        """检测位看不到目标时，逐步抬高机身找回目标。返回检测结果或 None。"""
        if not self.height_enabled:
            return None
        for _ in range(self.height_iter):
            if self.body_z + self.body_step > self.body_max:
                break
            self.set_body(self.body_z + self.body_step, reason='检测位看不到目标，抬高机身')
            r = self._stable(frames=20, need=2)
            if r is not None:
                return r
        return None

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
