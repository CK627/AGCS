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

    def __init__(self, board, ik, ak, params, detect, ultrasonic=None, display=None, tof=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.detect = detect
        self.ultrasonic = ultrasonic
        self.display = display
        self.tof = tof  # 红外(VL53L0X)，寻路到位用
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
        self.scan_settle_ms = float(gf.get('scan_settle_ms', 60)) / 1000.0  # 每步到位等待（防运动模糊）
        self.detect_interval = float(gf.get('detect_interval_ms', 30)) / 1000.0
        self.pan_scan_step = int(gf.get('pan_scan_step', 20))
        self.pan_move_ms = float(gf.get('pan_move_ms', 1)) / 1000.0
        self.pan_settle_ms = float(gf.get('pan_settle_ms', 60)) / 1000.0   # 每步到位等待（防运动模糊）
        self.pan_detect_interval = float(gf.get('pan_detect_interval_ms', 30)) / 1000.0

        # 追踪/逼近参数
        self.edge_margin = int(gf.get('edge_margin', 10))
        self.px_per_deg = float(gf.get('px_per_deg', 10.7))
        self.turn_deg = int(gf.get('turn_deg', 10))
        self.turn_sign = int(gf.get('turn_sign', 1))
        self.max_approach = int(gf.get('max_approach', 12))
        self.near_radius = float(gf.get('near_radius', 65.0))
        self.base_pulse = float(gf.get('base_pulse', 220.0))
        self.radius_per_pulse = float(gf.get('radius_per_pulse', 0.0273))
        self.stop_pulse_min = float(gf.get('stop_pulse_min', 120.0))
        self.slow_radius = float(gf.get('slow_radius', 35.0))
        # 逼近停止面积阈值：面积到该值视为目标够近，停止逼近交给夹取
        self.approach_area_threshold = float(params.get('grab', {}).get('approach_area_threshold', 1400.0))
        # 逼近到位红外距离：红外到目标 ≤ 此值（cm）停止逼近（优先用红外）
        self.tof_stop_cm = float(params.get('grab', {}).get('tof_stop_cm', 28.0))
        self.stop_y = float(gf.get('stop_y', 20.0))
        self.pan_band = int(gf.get('pan_band', 80))
        self.pan_band_fine = int(gf.get('pan_band_fine', 20))  # 到位后 21号回中的严格阈值
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

    def _confirm(self, tries=4, need_hits=2):
        """目标连续出现若干帧、且在画面中部，才确认，避免把边缘/噪声当目标。

        确认失败时把原因打到 INFO，便于现场排查是“检测不到”还是“中心不在画面中部”。
        """
        hits = 0
        last = None
        for _ in range(tries):
            r = self.detect()
            if r is not None:
                cx, cy = r['center']
                self.log.debug('[search] %s', action_msg(
                    '确认检测', action='中心x=%dpx 中心y=%dpx 面积=%d' % (cx, cy, r.get('area', 0))))
                # 目标中心要在画面中部（不贴边）才计数：顶部/底部/左右边缘都不算，
                # 避免目标还在边缘（快移出画面）就确认，导致追踪时目标已丢失
                if 60 <= cx <= 580 and 100 <= cy <= 380:
                    hits += 1
                    last = r
            time.sleep(0.08)
        if last is None or hits < need_hits:
            if last is not None:
                self.log.info('[search] %s', action_msg(
                    '目标未锁定', reason='%d/%d 帧在画面中部，最后中心x=%dpx y=%dpx 面积=%d'
                    % (hits, tries, last['center'][0], last['center'][1], last.get('area', 0)),
                    action='继续扫描'))
            return None
        return last

    def _lock_on(self, det):
        """检测到目标后先把云台转向目标居中，再确认。居中即锁定，不再继续扫。

        方向约定与 ColorTracker 一致：dx = 320-cx、dy = 240-cy，比例转向，
        目标偏右/偏下就按对应脉宽方向修正，直到目标进入画面中部再交给 _confirm。
        """
        for _ in range(12):
            cx, cy = det['center']
            if 60 <= cx <= 580 and 100 <= cy <= 380:
                return self._confirm()
            self.log.info('[search] %s', action_msg(
                '锁定目标', action='目标中心x=%dpx y=%dpx 偏离画面中心，云台转向居中' % (cx, cy)))
            self.x_dis = max(0, min(1000, int(self.x_dis + 0.2 * (320 - cx))))
            self.y_dis = max(0, min(1000, int(self.y_dis + 0.2 * (240 - cy))))
            self._cam()
            time.sleep(self.settle)
            det = self.detect()
            if det is None:
                return None
        return self._confirm()

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
        """把 21 移到 x，然后让 24 以小步平滑移动到 target_y，途中持续检测。

        修复：每步 移动 → 等舵机真正到位 → 再检测。旧实现发移动指令后立即取帧，
        舵机实际还在运动，画面模糊、目标中心跳动，导致锁定/确认失败。
        """
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
            # 等舵机真正到位再取帧：移动时间 + 到位余量
            time.sleep(self.scan_move_ms + self.scan_settle_ms)

            r = self.detect()
            if r is not None:
                self.log.info('[search] %s', action_msg(
                    '检测到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽' % (self.x_dis, self.y_dis)))
                cr = self._lock_on(r)
                if cr is not None:
                    return cr
                self.log.info('[search] %s', action_msg(
                    '检测到但未锁定', reason='中心x=%dpx y=%dpx 面积=%d'
                    % (r['center'][0], r['center'][1], r.get('area', 0)),
                    action='继续扫描'))

            if next_y == target_y:
                break
        return None

    def _smooth_pan_to(self, target_x):
        """让 21 号以小步平滑移动到 target_x，途中持续检测；找到返回 dict。

        修复：与 _smooth_tilt_to 相同，每步等舵机到位后再检测。
        """
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
            # 等舵机真正到位再取帧：移动时间 + 到位余量
            time.sleep(self.pan_move_ms + self.pan_settle_ms)

            r = self.detect()
            if r is not None:
                self.log.info('[search] %s', action_msg(
                    '检测到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽' % (self.x_dis, self.y_dis)))
                cr = self._lock_on(r)
                if cr is not None:
                    return cr
                self.log.info('[search] %s', action_msg(
                    '检测到但未锁定', reason='中心x=%dpx y=%dpx 面积=%d'
                    % (r['center'][0], r['center'][1], r.get('area', 0)),
                    action='继续扫描'))

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
        """固定 21 号脉宽为 x，24 号从 200 平滑扫到 1000（步长 200 档）。"""
        # 先把 24 号复位到起始档位，保证每次扫描都从同一起点开始
        self.y_dis = self.tilt_pulses[0]
        self.board.bus_servo_set_position(0.3, [[24, self.y_dis], [21, int(x)]])
        time.sleep(self.settle)
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
            cr = self._lock_on(r)
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
            # 转 21 号之前，先把 24 号复位到起始档位，避免带着上一次的 1000 一起转
            self.y_dis = self.tilt_pulses[0]
            self.board.bus_servo_set_position(0.3, [[24, self.y_dis], [21, int(self.x_dis)]])
            time.sleep(self.settle)
            r = self._smooth_pan_to(x)
            if r is not None:
                return r
            r = self._vertical_sweep(x)
            if r is not None:
                return r
        return None

    def _approach(self, det):
        """找到目标后：追踪线程贯穿转身 + 逼近，面积到阈值停止交给夹取。"""
        cx, cy = det['center']
        self.log.info('[search] %s', action_msg(
            '找到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 面积=%d像素'
            % (self.x_dis, self.y_dis, cx, cy, det['area'])))

        # 先启动追踪线程（贯穿转身 + 逼近），让 21/24 持续追踪目标居中
        self.tracker.start(self.x_dis, self.y_dis)
        self._stop_event.clear()
        self._obstacle_thread = threading.Thread(
            target=self._obstacle_monitor, name='obstacle-monitor', daemon=True)
        self._obstacle_thread.start()

        # 转身对准：追踪线程让 21 号跟着目标，转身体让 21 号回中（身体朝目标）
        for _ in range(self.max_approach):
            if self._stop_event.is_set():
                self.log.info('[search] %s', action_msg('停止寻路', reason='检测到障碍物'))
                return None, None
            r = self.tracker.latest()
            if r is None:
                time.sleep(0.05)
                continue
            dx = int(r['x_dis']) - 500
            if abs(dx) <= self.pan_band:
                break  # 21 接近 500，身体已对准目标
            if dx > 0:
                self._turn_body(self.pan_turn_deg)
            else:
                self._turn_body(-self.pan_turn_deg)
            time.sleep(0.4)

        # 逼近：前进 + 面积到阈值停止
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
            area = int(r.get('area', 0))
            dx = int(r['x_dis']) - 500

            self.log.info('[search] %s', action_msg(
                '追踪 #%d' % (step + 1),
                action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 半径=%dpx 21偏移=%+d 面积=%d'
                % (r['x_dis'], r.get('y_dis', self.base_pulse), cx, cy, radius, dx, area)))

            # 面积到阈值：目标够近，停止逼近交给夹取
            # （红外在 search 阶段机械臂非垂直朝下，读的是夹爪离地高度而非到目标距离，不可作到位判定）
            if area >= self.approach_area_threshold:
                self.log.info('[search] %s', action_msg(
                    '到位(面积阈值)', reason='面积=%d >= %d' % (area, self.approach_area_threshold),
                    action='停止逼近交给夹取'))
                # 到位后强制 21号回中（转身体让 21号=500、目标居中），
                # 避免 grab 前伸 IK 强制 21号=500 时目标偏离画面中心
                for _ in range(self.max_approach):
                    r2 = self.tracker.latest()
                    if r2 is None:
                        break
                    dx2 = int(r2['x_dis']) - 500
                    if abs(dx2) <= self.pan_band_fine:
                        break
                    self.log.info('[search] %s', action_msg(
                        '回中微调', action='21偏移=%+d 身体%s转 %d°' % (
                            dx2, '左' if dx2 > 0 else '右', self.pan_turn_deg)))
                    self._turn_body(self.pan_turn_deg if dx2 > 0 else -self.pan_turn_deg)
                    time.sleep(0.4)
                return (cx, cy), cy

            # 21 号偏离 500 太多：转身体把摄像头方向拉回朝前（方向与转身对准循环一致）
            if dx > self.pan_band:
                self.log.info('[search] %s', action_msg('摄像头偏右', action='身体左转 %d° 让21回中' % self.pan_turn_deg))
                self._turn_body(self.pan_turn_deg)
                time.sleep(0.4)
                continue
            if dx < -self.pan_band:
                self.log.info('[search] %s', action_msg('摄像头偏左', action='身体右转 %d° 让21回中' % self.pan_turn_deg))
                self._turn_body(-self.pan_turn_deg)
                time.sleep(0.4)
                continue

            # 前进逼近
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
        # 追踪线程和避障线程贯穿到夹取阶段，这里不 stop（由上层在夹取结束后清理）
        return result

    def reset_pose(self):
        """恢复官方初始机器姿态：云台回中（21=500/24=260）。

        机械臂复位与立正由调用方负责，这里只负责云台，避免寻路结束/失败后
        摄像头停留在扫描末尾角度。
        """
        self.x_dis, self.y_dis = 500, 260
        self.board.bus_servo_set_position(0.5, [[24, self.y_dis], [21, self.x_dis]])
        time.sleep(self.settle)
        self.log.info('[search] %s', action_msg(
            '恢复云台初始位置', action='云台21=%d 云台24=%d' % (self.x_dis, self.y_dis)))

    def stop(self):
        """夹取结束后清理：停止追踪线程和避障线程。"""
        self._stop_event.set()
        if self._obstacle_thread is not None and self._obstacle_thread.is_alive():
            self._obstacle_thread.join(timeout=1.0)
        self.tracker.stop()
