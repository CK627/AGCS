#!/usr/bin/python3
# coding=utf8
"""stream.py —— 把机器人彩色相机推到 /video.mjpeg，只推流、不动舵机、不检测。

和 2.py 用同一套预处理（rotate + 去畸变 + 高斯模糊），但相机用 VideoCapture(0)
直接开（官方 Camera 用 VideoCapture(-1) 会走 obsensor 后端，在 Pi5 上时好时坏）。

跑法（在机器人上）：
    cd ~/spiderpi/CompetitionUse
    python3 stream.py
本地浏览器/程序打开：http://<机器人IP>:5000/video.mjpeg
"""

import importlib.util
import os
import sys
import threading
import time

import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

# 复用 Auto-capture-1.py 的预处理函数（correct_camera / load_undistort_maps / load_params）
_path = os.path.join(_HERE, 'Auto-capture-1.py')
_spec = importlib.util.spec_from_file_location('no7_for_stream', _path)
_mod = importlib.util.module_from_spec(_spec)
sys.modules['no7_for_stream'] = _mod
_spec.loader.exec_module(_mod)

rotate = _mod.load_params()['vision'].get('camera_rotate', 0)
mapx, mapy = _mod.load_undistort_maps()

cap = cv2.VideoCapture(0)  # 彩色相机 = 设备 0
if not cap.isOpened():
    raise SystemExit('打不开摄像头 /dev/video0')

if _mod.task_server is not None:
    _mod.task_server.start_server()
    print('推流已启动: http://<机器人IP>:5000/video.mjpeg', flush=True)
else:
    print('!! 未安装 Flask / task_server，无法推流', flush=True)

_latest = None
_lock = threading.Lock()


def _reader():
    global _latest
    while True:
        ok, f = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        frame = cv2.remap(_mod.correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        frame = cv2.GaussianBlur(frame, (7, 7), 0)
        with _lock:
            _latest = frame


threading.Thread(target=_reader, daemon=True).start()

try:
    while True:
        with _lock:
            f = _latest
        if f is not None:
            _mod.task_server.publish_frame(f, max_fps=10.0)
        time.sleep(0.03)
except KeyboardInterrupt:
    print('退出', flush=True)
finally:
    cap.release()
