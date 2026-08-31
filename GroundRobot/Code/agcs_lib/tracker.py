#!/usr/bin/python3
# coding=utf8
"""目标跟踪线程：复刻官方 color_track.py 的 PID 云台跟踪。

官方 color_track.py 的跟踪部分就是两个 PID 分别控制舵机 21/24，
把目标锁到画面中心。这里把它放进独立线程，连续跟踪并保存最新位置，
主线程通过 latest() 取数据，不会被检测/PID 阻塞。
"""
import threading
import time

from common.pid import PID
from agcs_lib.logs import get_logger, action_msg


class ColorTracker:
    def __init__(self, board, detect, dead_x=40, dead_y=60,
                 pan_min=0, pan_max=1000, tilt_min=0, tilt_max=1000,
                 start_x=500, start_y=260, interval=0.03, tilt_fixed=False,
                 tilt_sign=1, settle=0.12):
        self.board = board
        self.detect = detect
        self.dead_x = int(dead_x)
        self.dead_y = int(dead_y)
        self.pan_min = int(pan_min)
        self.pan_max = int(pan_max)
        self.tilt_min = int(tilt_min)
        self.tilt_max = int(tilt_max)
        self.interval = float(interval)
        self.tilt_fixed = tilt_fixed  # True=24 号固定不追踪俯仰，只动 21 号水平转
        self.tilt_sign = int(tilt_sign)  # 俯仰方向符号：本机实测为 -1（上下反）
        self.settle = float(settle)      # 舵机移动后到位等待，避免运动模糊丢帧

        self.x_dis = int(start_x)
        self.y_dis = int(start_y)
        # 纯比例控制：目标静止色块、慢速逼近，不需要 I（积分累积会抬头过冲）
        # 也不需要 D（clear 后 last_error=0 导致微分突跳）。只留 P。
        self.x_pid = PID(P=0.1, I=0.0, D=0.0)
        self.y_pid = PID(P=0.1, I=0.0, D=0.0)

        self._lock = threading.Lock()
        self._latest = None
        self._lost_frames = 0
        self._stop = threading.Event()
        self._thread = None
        self.log = get_logger('tracker')

    def _update(self, r):
        cx, cy = r['center']
        old_x, old_y = self.x_dis, self.y_dis
        # 21 号水平追踪（死区内清积分，否则更新）
        if abs(cx - 320) < self.dead_x:
            self.x_pid.clear()
        else:
            self.x_pid.SetPoint = 320
            self.x_pid.update(cx)
            self.x_dis = max(self.pan_min, min(self.pan_max, self.x_dis + int(self.x_pid.output)))
        # 24 号俯仰：tilt_fixed 时固定（保持摆位后的水平朝前），否则正常追踪
        if not self.tilt_fixed:
            if abs(cy - 240) < self.dead_y:
                self.y_pid.clear()
            else:
                self.y_pid.SetPoint = 240
                self.y_pid.update(cy)
                # tilt_sign：修正方向按本机实际俯仰方向取反
                self.y_dis = max(self.tilt_min, min(
                    self.tilt_max, self.y_dis + int(self.tilt_sign * self.y_pid.output)))
        moved = old_x != self.x_dis or old_y != self.y_dis
        if moved:
            self.log.debug('[track] %s', action_msg(
                '云台调整', action='21号 %d->%d，24号 %d->%d' % (old_x, self.x_dis, old_y, self.y_dis)))
            self.board.bus_servo_set_position(0.02, [[24, self.y_dis], [21, self.x_dis]])

        with self._lock:
            self._latest = {
                'center': (cx, cy),
                'area': r.get('area', 0),
                'radius': r.get('radius', 20),
                'x_dis': self.x_dis,
                'y_dis': self.y_dis,
            }
            self._lost_frames = 0
        return moved

    def _run(self):
        while not self._stop.is_set():
            r = self.detect()
            if r is None:
                with self._lock:
                    self._latest = None
                    self._lost_frames += 1
            else:
                moved = self._update(r)
                if moved:
                    # 舵机刚被移动：等到位再取下一帧，避免运动模糊导致误丢目标
                    time.sleep(self.settle)
            time.sleep(self.interval)

    def start(self, x_dis=None, y_dis=None):
        if self._thread is not None and self._thread.is_alive():
            return
        if x_dis is not None:
            self.x_dis = int(x_dis)
        if y_dis is not None:
            self.y_dis = int(y_dis)
        self._latest = None
        self._lost_frames = 0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='color-track', daemon=True)
        self._thread.start()

    def latest(self):
        with self._lock:
            if self._latest is None:
                return None
            return dict(self._latest)

    def lost_frames(self):
        with self._lock:
            return self._lost_frames

    def stop(self, timeout=1.0):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout)
