#!/usr/bin/python3
# coding=utf8
"""寻路：找目标 -> 对准 -> 逼近。

职责边界：
- 只动六足（1-20）和云台（21/24）；
- 搜索：先 24 上下扫，再 21 从中间向两侧扩到 0~1000；
- 逼近：只用图片面积估算距离，超声波仅做避障；
- 不做像素坐标换算。像素转机械臂坐标是 grab 在 -90 检测位才该做的事。

日志通过 agcs_lib.logs 输出，保留结构化中文日志。
"""
import threading
import time

from agcs_lib.motion import turn_left, turn_right, go_forward
from agcs_lib.sensors import show_status, dist_cm
from agcs_lib.logs import get_logger, action_msg
from agcs_lib.tracker import ColorTracker


class Searcher:
    """detect: callable，返回 detect_color 的 dict 或 None。"""

    def __init__(self, board, ik, ak, params, detect, ultrasonic=None, display=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.detect = detect
        self.ultrasonic = ultrasonic
        self.display = display
        self.log = get_logger()

        sc = params.get('search', {})
        gf = params.get('gimbal_fetch', {})
        obs = params.get('obstacle', {})

        # 扫描顺序：24 上下扫；21 由中间向两侧扩到 0~1000
        self.pan_pulses = [int(v) for v in sc.get(
            'pan_pulses', [0, 200, 400, 600, 800, 1000])]
        self.tilt_pulses = [int(v) for v in sc.get(
            'tilt_pulses', [0, 200, 400, 600, 800, 1000])]
        self.settle = float(gf.get('settle_ms', 280)) / 1000.0
        self.tilt_scan_step = int(sc.get('tilt_scan_step', 20))
        self.scan_move_ms = float(gf.get('scan_move_ms', 50)) / 1000.0
        self.detect_interval = float(gf.get('detect_interval_ms', 30)) / 1000.0
        self.pan_scan_step = int(gf.get('pan_scan_step', 20))
        self.pan_move_ms = float(gf.get('pan_move_ms', 1)) / 1000.0
        self.pan_detect_interval = float(gf.get('pan_detect_interval_ms', 30)) / 1000.0

        # 追踪/逼近参数
        self.edge_margin = int(gf.get('edge_margin', 30))
        self.px_per_deg = float(gf.get('px_per_deg', 10.7))
        self.turn_deg = int(gf.get('turn_deg', 10))
        self.turn_sign = int(gf.get('turn_sign', 1))
        self.max_approach = int(gf.get('max_approach', 12))
        self.near_radius = float(gf.get('near_radius', 65.0))
        self.base_pulse = float(gf.get('base_pulse', 220.0))
        self.radius_per_pulse = float(gf.get('radius_per_pulse', 0.0273))
        self.stop_pulse_min = float(gf.get('stop_pulse_min', 120.0))
        self.slow_radius = float(gf.get('slow_radius', 35.0))
        self.stop_y = float(gf.get('stop_y', 20.0))
        self.pan_band = int(gf.get('pan_band', 80))
        self.pan_turn_deg = int(gf.get('pan_turn_deg', 5))
        self.fast_walk_mm = int(gf.get('fast_walk_mm', 70))
        self.fast_speed = int(gf.get('fast_speed', 90))
        self.walk_mm = int(gf.get('walk_mm', 40))
        self.walk_speed = int(gf.get('walk_speed', 50))
        self.body_turn_speed = int(sc.get('body_turn_speed', 80))

        # 超声波只用于避障；近距离读数乱跳，不用作距离判定
        self.obstacle_threshold = float(obs.get('threshold', 35.0))
        self.obstacle_disable_radius = float(obs.get('target_radius_gate', 30.0))

        # 云台当前脉宽（21 水平 / 24 俯仰）
        self.x_dis = 500
        self.y_dis = 260
        self.track_dead_x = int(gf.get('track_dead_cx', 40))
        self.track_dead_y = int(gf.get('track_dead_cy', 60))
        self.tracker = ColorTracker(
            board, detect,
            dead_x=self.track_dead_x, dead_y=self.track_dead_y,
            start_x=self.x_dis, start_y=self.y_dis)
        self._stop_event = threading.Event()
        self._obstacle_thread = None

    def _status(self, v):
        show_status(self.display, v)

    def _home(self):
        """恢复官方 color_track.py 的初始位置，再直接开始扫描。"""
        self.ak.setPitchRangeMoving((10, 15, 30), 0, -90, 100, 1)
        time.sleep(1)
        self.x_dis, self.y_dis = 500, 260
        self.board.bus_servo_set_position(0.5, [[24, self.y_dis], [21, self.x_dis]])
        time.sleep(self.settle)
        self.log.info('[search] %s', action_msg('恢复官方初始位置', action='云台21=%d 云台24=%d' % (self.x_dis, self.y_dis)))

    def _cam(self, duration=0.02):
        self.board.bus_servo_set_position(duration, [[24, int(self.y_dis)], [21, int(self.x_dis)]])

    def _set_cam(self, x, y):
        old_x, old_y = self.x_dis, self.y_dis
        self.x_dis = max(0, min(1000, int(x)))
        self.y_dis = max(0, min(1000, int(y)))
        self._cam()
        time.sleep(self.settle)
        if (old_x, old_y) != (self.x_dis, self.y_dis):
            self.log.debug('[search] %s', action_msg(
                '云台调整', action='舵机21 %d->%d脉宽，舵机24 %d->%d脉宽'
                % (old_x, self.x_dis, old_y, self.y_dis)))

    def _confirm(self, tries=5, need_hits=2):
        """目标连续出现若干帧才确认，避免把瞬时噪声当目标。"""
        hits = 0
        last = None
        for _ in range(tries):
            r = self.detect()
            if r is not None:
                cx, cy = r['center']
                if (self.edge_margin <= cx <= 640 - self.edge_margin and
                        self.edge_margin <= cy <= 480 - self.edge_margin):
                    hits += 1
                    last = r
            time.sleep(0.08)
        return last if hits >= need_hits else None

    def _turn_body(self, angle):
        if angle == 0:
            return
        self.log.info('[search] %s', action_msg('转身', action='%+.0f°' % angle))
        if self.turn_sign > 0:
            (turn_left if angle > 0 else turn_right)(self.ik, abs(angle), self.body_turn_speed)
        else:
            (turn_right if angle > 0 else turn_left)(self.ik, abs(angle), self.body_turn_speed)

    def _blocked(self):
        """前方有障碍物则返回 True；超声波无效时不判定。"""
        d = dist_cm(self.ultrasonic)
        return 0 < d < self.obstacle_threshold

    def _smooth_tilt_to(self, x, target_y):
        """把 21 移到 x，然后让 24 以小步平滑移动到 target_y，途中持续检测。"""
        self.x_dis = max(0, min(1000, int(x)))
        self.board.bus_servo_set_position(0.05, [[21, self.x_dis]])

        step = self.tilt_scan_step
        direction = 1 if target_y >= self.y_dis else -1
        while self.y_dis != target_y:
            next_y = self.y_dis + direction * step
            if direction > 0:
                next_y = min(next_y, target_y)
            else:
                next_y = max(next_y, target_y)
            self.y_dis = next_y
            self.board.bus_servo_set_position(
                self.scan_move_ms, [[24, self.y_dis], [21, self.x_dis]])

            # 舵机移动的同时反复取帧检测，避免“跳一下停一下”
            end_time = time.time() + self.scan_move_ms
            while time.time() < end_time:
                r = self.detect()
                if r is not None:
                    self.log.info('[search] %s', action_msg(
                        '检测到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽' % (self.x_dis, self.y_dis)))
                    cr = self._confirm()
                    if cr is not None:
                        return cr
                    self.log.debug('[search] %s', action_msg('检测到但未确认', action='继续扫描'))
                time.sleep(self.detect_interval)

            if next_y == target_y:
                break
        return None

    def _smooth_pan_to(self, target_x):
        """让 21 号以小步平滑移动到 target_x，途中持续检测；找到返回 dict。"""
        target_x = max(0, min(1000, int(target_x)))
        step = self.pan_scan_step
        direction = 1 if target_x >= self.x_dis else -1
        while self.x_dis != target_x:
            next_x = self.x_dis + direction * step
            if direction > 0:
                next_x = min(next_x, target_x)
            else:
                next_x = max(next_x, target_x)
            self.x_dis = next_x
            self.board.bus_servo_set_position(self.pan_move_ms, [[21, self.x_dis]])

            end_time = time.time() + self.pan_detect_interval
            while time.time() < end_time:
                r = self.detect()
                if r is not None:
                    self.log.info('[search] %s', action_msg(
                        '检测到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽' % (self.x_dis, self.y_dis)))
                    cr = self._confirm()
                    if cr is not None:
                        return cr
                    self.log.debug('[search] %s', action_msg('检测到但未确认', action='继续扫描'))
                time.sleep(self.pan_detect_interval)

            if next_x == target_x:
                break
        return None

    def _obstacle_monitor(self):
        """独立线程持续监测障碍物，发现后及时请求停止寻路。"""
        blocked_frames = 0
        while not self._stop_event.is_set():
            latest = self.tracker.latest()
            if latest is not None and int(latest.get('radius', 0)) >= self.obstacle_disable_radius:
                # 目标已经比较大，说明前方更可能是我们要夹的目标，而不是障碍物
                blocked_frames = 0
                time.sleep(0.05)
                continue
            if self._blocked():
                blocked_frames += 1
                if blocked_frames >= 2:
                    self.log.info('[search] %s', action_msg('检测到障碍物', action='请求停止寻路'))
                    self._stop_event.set()
                    return
            else:
                blocked_frames = 0
            time.sleep(0.1)

    def _vertical_sweep(self, x):
        """固定 21 号脉宽为 x，让 24 号平滑扫一遍；找到返回 dict，否则 None。"""
        for y in self.tilt_pulses:
            r = self._smooth_tilt_to(x, y)
            if r is not None:
                return r
        return None

    def search(self):
        """第一次扫描：先 24 上下，再 21 从中间向两侧扩到 0~1000。"""
        self.board.bus_servo_set_position(0.5, [[25, 120]])
        time.sleep(0.5)
        self._home()

        # 复位位置先看一眼；没找到才进入 24 号舵机扫描
        r = self.detect()
        if r is not None:
            cr = self._confirm()
            if cr is not None:
                return cr

        self.log.info('[search] %s', action_msg('开始扫描', action='21号固定500，24号上下扫描'))
        r = self._vertical_sweep(500)
        if r is not None:
            return r

        for x in self.pan_pulses:
            if x == 500:
                continue
            self.log.info('[search] %s', action_msg('扫描', action='21号脉宽=%d，24号上下扫描' % x))
            r = self._smooth_pan_to(x)
            if r is not None:
                return r
            r = self._vertical_sweep(x)
            if r is not None:
                return r
        return None

    def _approach(self, det):
        """找到目标后：先转身体对准，再启动跟踪线程持续锁定并逼近。"""
        cx, cy = det['center']
        self.log.info('[search] %s', action_msg(
            '找到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 面积=%d像素'
            % (self.x_dis, self.y_dis, cx, cy, det['area'])))

        # 按云台水平偏移 + 画面水平偏移，把六足身体转到目标方向
        target_dir = (self.x_dis - 500) / 4.1667 + (320 - cx) / self.px_per_deg
        deg = target_dir * self.turn_sign
        remaining = abs(deg)
        while remaining > 3:
            step = min(self.turn_deg, remaining)
            self._turn_body(step if deg > 0 else -step)
            remaining -= step
            time.sleep(0.5)

        # 身体对准后，21 回中；24 保持找到目标时的俯仰，交给跟踪线程持续锁定
        self._set_cam(500, self.y_dis)
        self.tracker.start(500, self.y_dis)
        self._stop_event.clear()
        self._obstacle_thread = threading.Thread(
            target=self._obstacle_monitor, name='obstacle-monitor', daemon=True)
        self._obstacle_thread.start()

        for step in range(self.max_approach):
            if self._stop_event.is_set():
                self.log.info('[search] %s', action_msg('停止寻路', reason='检测到障碍物'))
                return None, None
            r = self.tracker.latest()
            if r is None:
                if self.tracker.lost_frames() >= 3:
                    self.log.info('[search] %s', action_msg('连续丢失目标'))
                    return None, None
                self.log.debug('[search] %s', action_msg('目标暂不可见', action='跟踪线程继续寻找'))
                time.sleep(0.05)
                continue

            cx, cy = r['center']
            radius = max(int(r.get('radius', 20)), 1)
            pulse = int(r.get('y_dis', self.base_pulse))
            dx = int(r['x_dis']) - 500
            target_radius = max(5.0, self.near_radius - self.radius_per_pulse * (pulse - self.base_pulse))

            self.log.info('[search] %s', action_msg(
                '追踪 #%d' % (step + 1),
                action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 半径=%dpx 目标半径=%.1fpx 21偏移=%+d'
                % (r['x_dis'], pulse, cx, cy, radius, target_radius, dx)))

            # 地面目标逼近到脚下时，24 号会往下压；低于阈值就停，避免走过丢目标
            if pulse < self.stop_pulse_min:
                self.log.info('[search] %s', action_msg(
                    '到位(24过低)', reason='pulse=%d < %d' % (pulse, self.stop_pulse_min),
                    action='停止逼近'))
                return (cx, cy), cy

            d = dist_cm(self.ultrasonic)
            if 0 < d <= self.stop_y:
                self.log.info('[search] %s', action_msg(
                    '到位(超声y)', reason='y=%.1fcm <= %.1fcm' % (d, self.stop_y),
                    action='停止逼近'))
                return (cx, cy), cy

            # 21 号偏离 500 太多：动六足把摄像头方向拉回朝前，而不是让云台顶到边界
            if dx > self.pan_band:
                self.log.info('[search] %s', action_msg('摄像头偏右', action='身体右转 %d° 让21回中' % self.pan_turn_deg))
                self._turn_body(-self.pan_turn_deg)
                time.sleep(0.4)
                continue
            if dx < -self.pan_band:
                self.log.info('[search] %s', action_msg('摄像头偏左', action='身体左转 %d° 让21回中' % self.pan_turn_deg))
                self._turn_body(self.pan_turn_deg)
                time.sleep(0.4)
                continue

            if radius >= target_radius:
                return (cx, cy), cy

            if radius >= self.slow_radius:
                self.log.debug('[search] %s', action_msg(
                    '接近中', reason='半径 %dpx >= %dpx 慢速' % (radius, self.slow_radius),
                    action='前进 %dmm x1' % self.walk_mm))
                go_forward(self.ik, self.walk_mm, self.walk_speed, 1)
            else:
                self.log.debug('[search] %s', action_msg(
                    '接近中', reason='半径 %dpx < %dpx 快速' % (radius, self.slow_radius),
                    action='前进 %dmm x2' % self.fast_walk_mm))
                go_forward(self.ik, self.fast_walk_mm, self.fast_speed, 2)
            time.sleep(0.1)

        return None, None

    def run(self):
        self._status(1)
        det = self.search()
        if det is None:
            self.log.info('[search] %s', action_msg('目标未在附近', reason='21/24 全范围扫描后未发现目标'))
            return None, None
        result = self._approach(det)
        self._stop_event.set()
        if self._obstacle_thread is not None and self._obstacle_thread.is_alive():
            self._obstacle_thread.join(timeout=1.0)
        self.tracker.stop()
        return result
