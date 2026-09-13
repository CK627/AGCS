#!/usr/bin/python3
# coding=utf8
"""比赛步骤 2.2 自动寻路：平滑扫描找目标 → 连续追踪线程居中 → 转身对准 → 小步逼近 → 深度判距到位。

完全独立：只 import 标准库 + pip 库（cv2/numpy/flask）+ 官方 SDK（common）。
不依赖 agcs_lib。深度走 OpenNI2、彩色走 /dev/video0。无避障。

上报 status：state / position_m / heading_deg / last_result / message。

用法（先 sudo systemctl stop spiderpi）：
    cd /home/pi/spiderpi/CompetitionUse
    python3 AutoPathfinding.py --color yellow
"""
import os
import sys
import time
import math
import ctypes
import threading
import argparse

import cv2
import numpy as np

from common.ros_robot_controller_sdk import Board
from common import kinematics

# ---- LAB 颜色阈值（与 VisualTracking 一致，重新标定后改这里）----
LAB = {
    'red':    {'min': (0, 130, 115), 'max': (255, 170, 145)},
    'yellow': {'min': (150, 105, 158), 'max': (255, 128, 185)},
    'green':  {'min': (0, 105, 125), 'max': (255, 128, 156)},
    'blue':   {'min': (97, 122, 50), 'max': (255, 153, 104)},
}

# ---- 扫描参数 ----
PAN_PULSES = [0, 200, 400, 600, 800, 1000]     # 21 号水平扫描档位
TILT_PULSES = [100, 180, 260, 340, 420, 500]   # 24 号俯仰扫描档位（相机朝下）
TILT_SCAN_STEP = 20      # 24 号平滑扫描步长
PAN_SCAN_STEP = 20       # 21 号平滑扫描步长
SCAN_MOVE_MS = 0.05      # 24 号每步移动时间
SCAN_SETTLE_MS = 0.06    # 每步到位等待
PAN_MOVE_MS = 0.001      # 21 号每步移动时间
PAN_SETTLE_MS = 0.06     # 每步到位等待

# ---- 追踪/逼近参数 ----
P_GAIN = 0.1
DEAD_X, DEAD_Y = 40, 60
FRAME_CX, FRAME_CY = 320, 240
TRACK_INTERVAL = 0.03
PAN_BAND = 80           # 21 号偏离 500 的转身阈值
PAN_TURN_DEG = 5        # 身体每次转身角度
BODY_TURN_SPEED = 80
WALK_MM = 40            # 每步前进 mm
WALK_SPEED = 50
MAX_APPROACH = 12       # 最多逼近步数
STOP_DEPTH_CM = 25      # 深度判距停止距离 cm
CENTER_WAIT = 0.8       # 每步走后等云台重新居中的时间窗
LOST_LIMIT = 15         # 连续丢帧超过该值才放弃逼近


# ---------------- 颜色检测（内联）----------------
def detect_color(frame, color, min_area=50):
    img = frame.copy()
    h0, w0 = img.shape[:2]
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    ch = list(cv2.split(ycrcb))
    cv2.equalizeHist(ch[0], ch[0])
    cv2.merge(ch, ycrcb)
    img = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_NEAREST)
    img = cv2.GaussianBlur(img, (5, 5), 5)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lo, hi = LAB[color]['min'], LAB[color]['max']
    mask = cv2.inRange(lab, lo, hi)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    best, best_area = None, 0
    for c in contours:
        a = cv2.contourArea(c)
        if a >= min_area and a > best_area:
            best, best_area = c, a
    if best is None:
        return None
    ((cx, cy), radius) = cv2.minEnclosingCircle(best)
    cx = int(cx * w0 / 320)
    cy = int(cy * h0 / 240)
    radius = int(radius * w0 / 320)
    cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
    cv2.putText(frame, color, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return {'center': (cx, cy), 'radius': radius, 'area': float(best_area)}


# ---------------- OpenNI2 深度（内联）----------------
class _OniFrame(ctypes.Structure):
    _fields_ = [
        ('dataSize', ctypes.c_int), ('data', ctypes.c_void_p),
        ('sensorType', ctypes.c_int), ('timestamp', ctypes.c_uint64),
        ('frameIndex', ctypes.c_int), ('width', ctypes.c_int), ('height', ctypes.c_int),
        ('videoMode_pixelFormat', ctypes.c_int), ('videoMode_resX', ctypes.c_int),
        ('videoMode_resY', ctypes.c_int), ('videoMode_fps', ctypes.c_int),
        ('croppingEnabled', ctypes.c_int), ('cropOriginX', ctypes.c_int),
        ('cropOriginY', ctypes.c_int), ('stride', ctypes.c_int),
    ]


class _OniDeviceInfo(ctypes.Structure):
    _fields_ = [
        ('uri', ctypes.c_char * 256), ('vendor', ctypes.c_char * 256),
        ('name', ctypes.c_char * 256), ('usbVendorId', ctypes.c_uint16),
        ('usbProductId', ctypes.c_uint16),
    ]


class DepthCam:
    def __init__(self, lib='/home/pi/orbbec_sdk/libOpenNI2.so'):
        self.lib = ctypes.CDLL(lib)
        self._device = ctypes.c_void_p()
        self._stream = ctypes.c_void_p()
        L = self.lib
        L.oniInitialize.argtypes = [ctypes.c_int]; L.oniInitialize.restype = ctypes.c_int
        L.oniShutdown.argtypes = []; L.oniShutdown.restype = None
        L.oniGetDeviceList.argtypes = [ctypes.POINTER(ctypes.POINTER(_OniDeviceInfo)),
                                       ctypes.POINTER(ctypes.c_int)]
        L.oniGetDeviceList.restype = ctypes.c_int
        L.oniReleaseDeviceList.argtypes = [ctypes.POINTER(_OniDeviceInfo)]
        L.oniReleaseDeviceList.restype = ctypes.c_int
        L.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
        L.oniDeviceOpen.restype = ctypes.c_int
        L.oniDeviceClose.argtypes = [ctypes.c_void_p]; L.oniDeviceClose.restype = ctypes.c_int
        L.oniDeviceCreateStream.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                            ctypes.POINTER(ctypes.c_void_p)]
        L.oniDeviceCreateStream.restype = ctypes.c_int
        L.oniStreamStart.argtypes = [ctypes.c_void_p]; L.oniStreamStart.restype = ctypes.c_int
        L.oniStreamStop.argtypes = [ctypes.c_void_p]; L.oniStreamStop.restype = None
        L.oniStreamDestroy.argtypes = [ctypes.c_void_p]; L.oniStreamDestroy.restype = None
        L.oniStreamReadFrame.argtypes = [ctypes.c_void_p,
                                         ctypes.POINTER(ctypes.POINTER(_OniFrame))]
        L.oniStreamReadFrame.restype = ctypes.c_int
        L.oniFrameRelease.argtypes = [ctypes.POINTER(_OniFrame)]
        L.oniFrameRelease.restype = None

    def open(self):
        if self.lib.oniInitialize(2002) != 0:
            raise RuntimeError('oniInitialize 失败')
        devs = ctypes.POINTER(_OniDeviceInfo)()
        n = ctypes.c_int(0)
        self.lib.oniGetDeviceList(ctypes.byref(devs), ctypes.byref(n))
        if n.value == 0:
            raise RuntimeError('未找到 Orbbec 深度设备')
        uri = bytes(devs[0].uri)
        self.lib.oniReleaseDeviceList(devs)
        if self.lib.oniDeviceOpen(uri, ctypes.byref(self._device)) != 0:
            raise RuntimeError('打开深度设备失败')
        if self.lib.oniDeviceCreateStream(self._device, 3, ctypes.byref(self._stream)) != 0:
            raise RuntimeError('创建深度流失败')
        if self.lib.oniStreamStart(self._stream) != 0:
            raise RuntimeError('启动深度流失败')

    def read(self, timeout_ms=100):
        frame = ctypes.POINTER(_OniFrame)()
        if self.lib.oniStreamReadFrame(self._stream, ctypes.byref(frame)) != 0:
            return None
        try:
            f = frame.contents
            raw = ctypes.string_at(f.data, f.dataSize)
            arr = np.frombuffer(raw, dtype=np.uint16)
            arr = arr[:f.stride // 2 * f.height].reshape(f.height, f.stride // 2)
            return arr[:, :f.width].copy()
        finally:
            self.lib.oniFrameRelease(frame)

    def close(self):
        if self._stream:
            self.lib.oniStreamDestroy(self._stream)
        if self._device:
            self.lib.oniDeviceClose(self._device)
        self.lib.oniShutdown()


# ---------------- Flask 状态/推流（内联）----------------
STATUS = {'state': 'IDLE', 'position_m': {'x': 0.0, 'y': 0.0}, 'heading_deg': 0.0,
          'last_result': None, 'message': ''}
_LATEST_JPEG = None
_JPEG_LOCK = threading.Lock()


def set_status(**kw):
    STATUS.update(kw)


def publish_frame(frame):
    global _LATEST_JPEG
    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    if ok:
        with _JPEG_LOCK:
            _LATEST_JPEG = jpg.tobytes()


def start_server():
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    import flask.cli
    flask.cli.show_server_banner = lambda *a, **k: None

    from flask import Flask, Response, jsonify
    app = Flask(__name__)

    @app.route('/status')
    def status():
        return jsonify(STATUS)

    @app.route('/video.mjpeg')
    def video():
        def gen():
            while True:
                with _JPEG_LOCK:
                    jpg = _LATEST_JPEG
                if jpg is None:
                    time.sleep(0.05)
                    continue
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                time.sleep(0.1)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000,
                                             'debug': False, 'threaded': True},
                     daemon=True).start()


def lan_ip():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    finally:
        s.close()


# ---------------- 摄像头后台线程（连续取帧，线程安全）----------------
class Camera:
    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self.frame = f
            time.sleep(0.01)

    def read(self):
        with self._lock:
            return self.frame

    def stop(self):
        self._stop.set()


# ---------------- 连续追踪线程（复刻 ColorTracker 纯 P 控制）----------------
class Tracker:
    def __init__(self, board, detect):
        self.board = board
        self.detect = detect
        self.x_dis, self.y_dis = 500, 260
        self.latest = None
        self.lost_frames = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def _update(self, r):
        cx, cy = r['center']
        if abs(cx - FRAME_CX) >= DEAD_X:
            self.x_dis += int(P_GAIN * (FRAME_CX - cx))
            self.x_dis = max(0, min(1000, self.x_dis))
        if abs(cy - FRAME_CY) >= DEAD_Y:
            self.y_dis += int(P_GAIN * (FRAME_CY - cy))
            self.y_dis = max(0, min(1000, self.y_dis))
        self.board.bus_servo_set_position(0.02, [[24, self.y_dis], [21, self.x_dis]])
        with self._lock:
            self.latest = {'center': (cx, cy), 'radius': r.get('radius', 20),
                           'area': r.get('area', 0), 'x_dis': self.x_dis, 'y_dis': self.y_dis}
            self.lost_frames = 0

    def _run(self):
        while not self._stop.is_set():
            r = self.detect()
            if r is None:
                with self._lock:
                    self.latest = None
                    self.lost_frames += 1
            else:
                self._update(r)
            time.sleep(TRACK_INTERVAL)

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get(self):
        with self._lock:
            return self.latest

    def lost(self):
        with self._lock:
            return self.lost_frames

    def stop(self):
        self._stop.set()


# ---------------- 寻路主体 ----------------
class Pathfinder:
    def __init__(self, board, ik, cam, depth, color):
        self.board = board
        self.ik = ik
        self.cam = cam
        self.depth = depth
        self.color = color
        self.x_dis, self.y_dis = 500, 260
        self.tracker = None

    def set_cam(self, x, y):
        self.x_dis = max(0, min(1000, int(x)))
        self.y_dis = max(0, min(1000, int(y)))
        self.board.bus_servo_set_position(0.02, [[24, self.y_dis], [21, self.x_dis]])

    def detect(self):
        f = self.cam.read()
        if f is None:
            return None
        publish_frame(f)
        return detect_color(f, self.color)

    def confirm(self, tries=4, need_hits=2):
        """目标连续若干帧且居中才算确认，避免边缘/噪声误判。"""
        hits, last = 0, None
        for _ in range(tries):
            r = self.detect()
            if r is not None:
                cx, cy = r['center']
                if 200 <= cx <= 440 and 120 <= cy <= 380:
                    hits += 1
                    last = r
            time.sleep(0.08)
        return last if (last is not None and hits >= need_hits) else None

    def lock_on(self, det):
        """检测到目标后云台转向居中，再确认。"""
        for _ in range(12):
            cx, cy = det['center']
            if 200 <= cx <= 440 and 120 <= cy <= 380:
                return self.confirm()
            self.x_dis = max(0, min(1000, int(self.x_dis + 0.2 * (320 - cx))))
            self.y_dis = max(0, min(1000, int(self.y_dis + 0.2 * (240 - cy))))
            self.set_cam(self.x_dis, self.y_dis)
            time.sleep(0.05)
            det = self.detect()
            if det is None:
                return None
        return self.confirm()

    def smooth_tilt(self, x, target_y):
        """21 固定 x，24 平滑移到 target_y，途中持续检测。"""
        self.x_dis = max(0, min(1000, int(x)))
        self.board.bus_servo_set_position(0.05, [[21, self.x_dis]])
        direction = 1 if target_y >= self.y_dis else -1
        while self.y_dis != target_y:
            nxt = self.y_dis + direction * TILT_SCAN_STEP
            nxt = min(nxt, target_y) if direction > 0 else max(nxt, target_y)
            self.y_dis = nxt
            self.board.bus_servo_set_position(SCAN_MOVE_MS, [[24, self.y_dis], [21, self.x_dis]])
            time.sleep(SCAN_MOVE_MS + SCAN_SETTLE_MS)
            r = self.detect()
            if r is not None:
                cr = self.lock_on(r)
                if cr is not None:
                    return cr
            if nxt == target_y:
                break
        return None

    def smooth_pan(self, target_x):
        """21 平滑移到 target_x，途中持续检测。"""
        target_x = max(0, min(1000, int(target_x)))
        direction = 1 if target_x >= self.x_dis else -1
        while self.x_dis != target_x:
            nxt = self.x_dis + direction * PAN_SCAN_STEP
            nxt = min(nxt, target_x) if direction > 0 else max(nxt, target_x)
            self.x_dis = nxt
            self.board.bus_servo_set_position(PAN_MOVE_MS, [[21, self.x_dis]])
            time.sleep(PAN_MOVE_MS + PAN_SETTLE_MS)
            r = self.detect()
            if r is not None:
                cr = self.lock_on(r)
                if cr is not None:
                    return cr
            if nxt == target_x:
                break
        return None

    def vertical_sweep(self, x):
        self.y_dis = TILT_PULSES[0]
        self.board.bus_servo_set_position(0.8, [[24, self.y_dis], [21, int(x)]])
        time.sleep(0.8)
        for y in TILT_PULSES:
            r = self.smooth_tilt(x, y)
            if r is not None:
                return r
        return None

    def search(self):
        self.set_cam(500, 260)
        time.sleep(0.3)
        r = self.detect()
        if r is not None:
            cr = self.lock_on(r)
            if cr is not None:
                return cr

        r = self.vertical_sweep(500)
        if r is not None:
            return r

        for x in PAN_PULSES:
            if x == 500:
                continue
            self.y_dis = TILT_PULSES[0]
            self.board.bus_servo_set_position(0.8, [[24, self.y_dis], [21, int(x)]])
            time.sleep(0.8)
            r = self.smooth_pan(x)
            if r is not None:
                return r
            r = self.vertical_sweep(x)
            if r is not None:
                return r
        return None

    def distance_cm(self, cx, cy):
        if self.depth is None:
            return None
        d = self.depth.read(100)
        if d is None:
            return None
        h, w = d.shape
        px = min(max(int(cx), 0), w - 1)
        py = min(max(int(cy), 0), h - 1)
        z = int(d[py, px])
        return z / 10.0 if z > 0 else None

    def approach(self, det):
        """启动追踪线程，转身对准 + 小步前进逼近，深度判距到位。"""
        cx, cy = det['center']
        print('找到目标 中心=(%d,%d)' % (cx, cy))
        self.tracker = Tracker(self.board, self.detect)
        self.tracker.start()

        # 转身对准：追踪线程让 21 跟着目标，转身体让 21 回中
        for _ in range(MAX_APPROACH):
            r = self.tracker.get()
            if r is None:
                time.sleep(0.05)
                continue
            dx = int(r['x_dis']) - 500
            if abs(dx) <= PAN_BAND:
                break
            ang = PAN_TURN_DEG if dx > 0 else -PAN_TURN_DEG
            (self.ik.turn_left if ang > 0 else self.ik.turn_right)(
                self.ik.initial_pos, 2, abs(ang), BODY_TURN_SPEED, 1)
            time.sleep(0.4)

        # 逼近
        for step in range(MAX_APPROACH):
            deadline = time.time() + CENTER_WAIT
            r = None
            while time.time() < deadline:
                r = self.tracker.get()
                if r is not None:
                    break
                time.sleep(0.03)
            if r is None:
                if self.tracker.lost() >= LOST_LIMIT:
                    set_status(last_result='failed', message='连续丢失目标')
                    print('连续丢失目标，放弃逼近')
                    return None
                continue

            cx, cy = r['center']
            dx = int(r['x_dis']) - 500

            if abs(dx) > PAN_BAND:
                ang = PAN_TURN_DEG if dx > 0 else -PAN_TURN_DEG
                (self.ik.turn_left if ang > 0 else self.ik.turn_right)(
                    self.ik.initial_pos, 2, abs(ang), BODY_TURN_SPEED, 1)
                time.sleep(0.4)
                continue

            d_cm = self.distance_cm(cx, cy)
            wx = (cx - FRAME_CX) / 570.0 * (d_cm / 100.0) if d_cm else 0.0  # 粗略横向位置
            pos = {'x': round(wx, 3), 'y': round(d_cm / 100.0, 3) if d_cm else 0.0}
            heading = round(math.degrees(math.atan2(wx, d_cm / 100.0)), 1) if d_cm else 0.0
            msg = '追踪 #%d 中心=(%d,%d) 距离=%s' % (
                step + 1, cx, cy, ('%.1fcm' % d_cm) if d_cm is not None else '无')
            print(msg)
            set_status(state='NAV', position_m=pos, heading_deg=heading, message=msg)

            if d_cm is not None and d_cm <= STOP_DEPTH_CM:
                print('到位(距离 %.1fcm)' % d_cm)
                set_status(state='DONE', last_result='done', message='寻路到位')
                return (cx, cy)

            self.ik.go_forward(self.ik.initial_pos, 2, WALK_MM, WALK_SPEED, 1)
            time.sleep(0.05)

        set_status(last_result='failed', message='逼近步数用尽')
        return None

    def run(self):
        set_status(state='SEARCH', message='扫描找目标')
        det = self.search()
        if det is None:
            set_status(state='SEARCH', last_result='failed', message='未找到目标')
            return None
        return self.approach(det)


def main():
    parser = argparse.ArgumentParser(description='2.2 自动寻路')
    parser.add_argument('--color', default='yellow',
                        choices=['red', 'green', 'blue', 'yellow'])
    args = parser.parse_args()

    board = Board()
    ik = kinematics.IK(board)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    for _ in range(5):
        cap.read()
    cam = Camera(cap)

    depth = None
    try:
        depth = DepthCam()
        depth.open()
    except Exception as e:
        print('深度相机初始化失败: %s' % e)

    start_server()
    print('推流: http://%s:5000/video.mjpeg' % lan_ip(), flush=True)

    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    pf = Pathfinder(board, ik, cam, depth, args.color)
    try:
        pf.run()
    except KeyboardInterrupt:
        pass
    finally:
        if pf.tracker is not None:
            pf.tracker.stop()
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        ik.stand(ik.initial_pos, t=500)
        cam.stop()
        cap.release()
        if depth is not None:
            depth.close()
        set_status(state='IDLE', message='寻路结束')


if __name__ == '__main__':
    main()
