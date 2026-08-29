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
                 start_x=500, start_y=260, interval=0.03):
        self.board = board
        self.detect = detect
        self.dead_x = int(dead_x)
        self.dead_y = int(dead_y)
        self.pan_min = int(pan_min)
        self.pan_max = int(pan_max)
        self.tilt_min = int(tilt_min)
        self.tilt_max = int(tilt_max)
        self.interval = float(interval)

        self.x_dis = int(start_x)
        self.y_dis = int(start_y)
        self.x_pid = PID(P=0.1, I=0.001, D=0.008)
        self.y_pid = PID(P=0.1, I=0.02, D=0.008)

        self._lock = threading.Lock()
        self._latest = None
        self._lost_frames = 0
        self._stop = threading.Event()
        self._thread = None
        self.log = get_logger()

    def _update(self, r):
        cx, cy = r['center']
        old_x, old_y = self.x_dis, self.y_dis
        # 死区：目标已居中就不更新 PID，并清积分，避免积分累积过冲把目标追出画面
        if abs(cx - 320) < self.dead_x and abs(cy - 240) < self.dead_y:
            self.x_pid.clear()
            self.y_pid.clear()
        else:
            self.x_pid.SetPoint = 320
            self.x_pid.update(cx)
            self.y_pid.SetPoint = 240
            self.y_pid.update(cy)
            self.x_dis = max(self.pan_min, min(self.pan_max, self.x_dis + int(self.x_pid.output)))
            self.y_dis = max(self.tilt_min, min(self.tilt_max, self.y_dis + int(self.y_pid.output)))
        if old_x != self.x_dis or old_y != self.y_dis:
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

    def _run(self):
        while not self._stop.is_set():
            r = self.detect()
            if r is None:
                with self._lock:
                    self._latest = None
                    self._lost_frames += 1
            else:
                self._update(r)
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
