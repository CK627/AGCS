#!/usr/bin/python3
# coding=utf8
"""比赛步骤 2.1 视觉追踪（默认黄色）：云台 21/24 PID 跟随色块 + 深度估位置。

完全独立：只 import 标准库 + pip 库（cv2/numpy/flask）+ 官方 SDK（common）。
不依赖 agcs_lib / _common / ColorTracker / communication.task_server。

深度走 OpenNI2(libOpenNI2.so，ctypes 直调)、彩色走 /dev/video0(uvcvideo)。

用法（先 sudo systemctl stop spiderpi）：
    cd /home/pi/spiderpi/CompetitionUse
    python3 VisualTracking.py --color yellow
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

# ---- LAB 颜色阈值（从 config/lab_config.yaml 内联，重新标定后改这里）----
LAB = {
    'red':    {'min': (0, 130, 115), 'max': (255, 170, 145)},
    'yellow': {'min': (150, 105, 158), 'max': (255, 128, 185)},
    'green':  {'min': (0, 105, 125), 'max': (255, 128, 156)},
    'blue':   {'min': (97, 122, 50), 'max': (255, 153, 104)},
}

# ---- 云台追踪参数（复刻官方 color_track.py 的纯 P 控制）----
P_GAIN = 0.1
DEAD_X, DEAD_Y = 40, 60
PAN_MIN, PAN_MAX = 0, 1000
TILT_MIN, TILT_MAX = 0, 1000
START_X, START_Y = 500, 260
FRAME_CX, FRAME_CY = 320, 240


# ---------------- 颜色检测（内联 agcs_lib.vision.detect_color 的 LAB 管线）----------------
def detect_color(frame, color, min_area=50):
    """在 frame(640x480 BGR) 里检测颜色块，返回 dict(center, radius, area) 或 None。"""
    img = frame.copy()
    h0, w0 = img.shape[:2]
    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_NEAREST)
    img = cv2.GaussianBlur(img, (3, 3), 3)
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


# ---------------- OpenNI2 深度（内联 agcs_lib.depth.DepthCamera 的最小部分）----------------
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
    """最小 OpenNI2 深度读取（读一帧 + 像素转世界坐标）。"""

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
        L.oniCoordinateConverterDepthToWorld.argtypes = [
            ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float)]
        L.oniCoordinateConverterDepthToWorld.restype = ctypes.c_int

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
            raise RuntimeError('创建深度流失败')  # 3 = ONI_SENSOR_DEPTH
        if self.lib.oniStreamStart(self._stream) != 0:
            raise RuntimeError('启动深度流失败')

    def read(self, timeout_ms=100):
        """读一帧深度图 (H,W) uint16 mm，超时返回 None。"""
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

    def to_world(self, x, y, z):
        """深度像素 (x,y) + 深度值 z(mm) → 世界 (X,Y,Z) mm。"""
        wx, wy, wz = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        self.lib.oniCoordinateConverterDepthToWorld(
            self._stream, float(x), float(y), float(z),
            ctypes.byref(wx), ctypes.byref(wy), ctypes.byref(wz))
        return wx.value, wy.value, wz.value

    def close(self):
        if self._stream:
            self.lib.oniStreamDestroy(self._stream)
        if self._device:
            self.lib.oniDeviceClose(self._device)
        self.lib.oniShutdown()


# ---------------- 状态/推流（内联 task_server 的最小部分：Flask /status + /video.mjpeg）----------------
STATUS = {'state': 'IDLE', 'position_m': {'x': 0.0, 'y': 0.0}, 'heading_deg': 0.0,
          'message': ''}
_LATEST_JPEG = None
_JPEG_LOCK = threading.Lock()


def set_status(**kw):
    STATUS.update(kw)


def publish_frame(frame, max_fps=10.0):
    global _LATEST_JPEG
    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    if ok:
        with _JPEG_LOCK:
            _LATEST_JPEG = jpg.tobytes()


def start_server():
    import logging
    # 抑制 Flask 开发服务器的启动横幅 + 请求日志（红字 WARNING + 一堆 GET 日志）
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
    import subprocess
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        pass
    # 直连(AP)模式没有外网，退回从网卡枚举本机 IP
    try:
        out = subprocess.check_output(['hostname', '-I']).decode().strip()
        ips = [x for x in out.split()
               if x.startswith(('192.168', '10.', '172.'))]
        return ips[0] if ips else (out.split()[0] if out else '127.0.0.1')
    except Exception:
        return '127.0.0.1'


def main():
    parser = argparse.ArgumentParser(description='2.1 视觉追踪')
    parser.add_argument('--color', default='yellow',
                        choices=['red', 'green', 'blue', 'yellow'])
    args = parser.parse_args()

    board = Board()

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
    set_status(state='TRACKING', message='2.1 视觉追踪')

    x_dis, y_dis = START_X, START_Y
    board.bus_servo_set_position(0.3, [[24, y_dis], [21, x_dis]])

    pos = {'x': 0.0, 'y': 0.0}
    heading = 0.0
    last_depth_t = 0.0

    print('追踪开始（Ctrl+C 退出）')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            r = detect_color(frame, args.color)
            publish_frame(frame)
            if r is not None:
                cx, cy = r['center']
                if abs(cx - FRAME_CX) >= DEAD_X:
                    x_dis += int(P_GAIN * (FRAME_CX - cx))
                    x_dis = max(PAN_MIN, min(PAN_MAX, x_dis))
                if abs(cy - FRAME_CY) >= DEAD_Y:
                    y_dis += int(P_GAIN * (FRAME_CY - cy))
                    y_dis = max(TILT_MIN, min(TILT_MAX, y_dis))
                board.bus_servo_set_position(0.02, [[24, y_dis], [21, x_dis]])

                now = time.time()
                if depth is not None and now - last_depth_t >= 0.3:
                    last_depth_t = now
                    d = depth.read(100)
                    if d is not None:
                        h, w = d.shape
                        px = min(max(cx, 0), w - 1)
                        py = min(max(cy, 0), h - 1)
                        z = int(d[py, px])
                        if z > 0:
                            wx, _, wz = depth.to_world(px, py, float(z))
                            pos = {'x': round(wx / 1000.0, 3), 'y': round(wz / 1000.0, 3)}
                            heading = round(math.degrees(math.atan2(wx, wz)), 1)
                set_status(state='TRACKING', position_m=pos, heading_deg=heading,
                           message='追踪中 中心=(%d,%d)' % (cx, cy))
            else:
                set_status(state='TRACKING', message='追踪中 未发现目标')
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        cap.release()
        if depth is not None:
            depth.close()
        set_status(state='IDLE', message='追踪结束')


if __name__ == '__main__':
    main()
