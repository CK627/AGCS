#!/usr/bin/python3
# coding=utf8
"""比赛步骤 2.2 自动寻路：扫描找目标 → 转身对准 → 小步前进逼近 → 深度判距到位。

完全独立：只 import 标准库 + pip 库（cv2/numpy/flask）+ 官方 SDK（common）。
不依赖 agcs_lib。深度走 OpenNI2、彩色走 /dev/video0。

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

# ---- 扫描参数（相机斜向下，24号越小越朝下，这里扫「朝下→朝前」一段）----
PAN_PULSES = [0, 200, 400, 600, 800, 1000]     # 21 号水平扫描档位
TILT_PULSES = [100, 180, 260, 340, 420, 500]   # 24 号俯仰扫描档位
SETTLE_S = 0.3                                  # 每步到位等待

# ---- 追踪/逼近参数 ----
P_GAIN = 0.1
DEAD_X, DEAD_Y = 40, 60
FRAME_CX, FRAME_CY = 320, 240
PAN_BAND = 80          # 21 号偏离 500 多少算偏（转身阈值）
PAN_TURN_DEG = 5       # 身体每次转身角度
BODY_TURN_SPEED = 80
WALK_MM = 40           # 每步前进 mm
WALK_SPEED = 50
MAX_APPROACH = 15      # 最多逼近步数
STOP_DEPTH_CM = 25     # 深度判距停止距离 cm


# ---------------- 颜色检测（内联）----------------
def detect_color(frame, color, min_area=150):
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
          'message': ''}
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


# ---------------- 寻路逻辑 ----------------
def set_cam(board, x, y):
    board.bus_servo_set_position(0.02, [[24, int(y)], [21, int(x)]])


def read_frame(cap):
    ok, frame = cap.read()
    return frame if ok else None


def search(board, cap, detect, color):
    """扫描找目标：先复位看一眼，再 24 号上下扫 + 21 号左右扫。返回 detect 结果或 None。"""
    set_cam(board, 500, 260)
    time.sleep(SETTLE_S)
    f = read_frame(cap)
    if f is not None:
        publish_frame(f)
        r = detect(f)
        if r is not None:
            return r

    for y in TILT_PULSES:
        set_cam(board, 500, y)
        time.sleep(SETTLE_S)
        f = read_frame(cap)
        if f is not None:
            publish_frame(f)
            r = detect(f)
            if r is not None:
                return r

    for x in PAN_PULSES:
        if x == 500:
            continue
        set_cam(board, x, TILT_PULSES[0])
        time.sleep(SETTLE_S)
        for y in TILT_PULSES:
            set_cam(board, x, y)
            time.sleep(SETTLE_S)
            f = read_frame(cap)
            if f is not None:
                publish_frame(f)
                r = detect(f)
                if r is not None:
                    return r
    return None


def distance_cm(depth, cx, cy):
    """深度相机读 (cx,cy) 处距离(cm)；无有效读数返回 None。"""
    if depth is None:
        return None
    d = depth.read(100)
    if d is None:
        return None
    h, w = d.shape
    px = min(max(int(cx), 0), w - 1)
    py = min(max(int(cy), 0), h - 1)
    z = int(d[py, px])
    if z <= 0:
        return None
    return z / 10.0


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

    depth = None
    try:
        depth = DepthCam()
        depth.open()
    except Exception as e:
        print('深度相机初始化失败: %s' % e)

    start_server()
    print('推流: http://%s:5000/video.mjpeg' % lan_ip(), flush=True)

    # 立正
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)
    set_status(state='SEARCH', message='2.2 自动寻路')

    def detect(frame):
        return detect_color(frame, args.color)

    print('开始扫描找目标...')
    det = search(board, cap, detect, args.color)
    if det is None:
        print('未找到目标')
        set_status(state='SEARCH', message='未找到目标')
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        cap.release()
        if depth is not None:
            depth.close()
        return

    print('找到目标，开始逼近...')
    x_dis, y_dis = 500, 260
    try:
        for step in range(MAX_APPROACH):
            f = read_frame(cap)
            if f is None:
                continue
            publish_frame(f)
            r = detect(f)
            if r is None:
                set_status(state='SEARCH', message='追踪中 未发现目标')
                time.sleep(0.05)
                continue
            cx, cy = r['center']

            # 云台追踪：把目标居中
            if abs(cx - FRAME_CX) >= DEAD_X:
                x_dis += int(P_GAIN * (FRAME_CX - cx))
                x_dis = max(0, min(1000, x_dis))
            if abs(cy - FRAME_CY) >= DEAD_Y:
                y_dis += int(P_GAIN * (FRAME_CY - cy))
                y_dis = max(0, min(1000, y_dis))
            set_cam(board, x_dis, y_dis)

            # 转身对准：21 号偏离 500 就转身体，把云台拉回朝前
            dx = x_dis - 500
            if abs(dx) > PAN_BAND:
                ang = PAN_TURN_DEG if dx > 0 else -PAN_TURN_DEG
                (ik.turn_left if ang > 0 else ik.turn_right)(ik.initial_pos, 2, abs(ang),
                                                             BODY_TURN_SPEED, 1)
                time.sleep(0.4)
                continue

            # 判距到位
            d_cm = distance_cm(depth, cx, cy)
            msg = '追踪 #%d 中心=(%d,%d) 距离=%s' % (step + 1, cx, cy,
                                                    ('%.1fcm' % d_cm) if d_cm is not None else '无')
            print(msg)
            set_status(state='SEARCH', message=msg)
            if d_cm is not None and d_cm <= STOP_DEPTH_CM:
                print('到位(距离 %.1fcm <= %.1fcm)' % (d_cm, STOP_DEPTH_CM))
                set_status(state='DONE', message='寻路到位')
                break

            # 前进一小步
            ik.go_forward(ik.initial_pos, 2, WALK_MM, WALK_SPEED, 1)
            time.sleep(0.1)
        else:
            set_status(state='SEARCH', message='逼近步数用尽')
    except KeyboardInterrupt:
        pass
    finally:
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        ik.stand(ik.initial_pos, t=500)
        cap.release()
        if depth is not None:
            depth.close()
        set_status(state='IDLE', message='寻路结束')


if __name__ == '__main__':
    main()
