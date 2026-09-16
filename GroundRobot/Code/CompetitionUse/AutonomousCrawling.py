#!/usr/bin/python3
# coding=utf8
"""自动抓取 + 人脸识别递物（固定路线）。

流程：
    状态 Grab → 恢复官方初始位置 → 21 先动 → 22/23 一起动 → 24 到 270
    → 闭合夹爪(25) → 保持夹取 → 恢复机械臂初始位置（夹爪保持闭合）
    → 21 左转 → 识别人脸(OpenCV Haar) → 松开夹爪放到手上 → 恢复。

完全独立：只 import 标准库 + pip 库（cv2/flask）+ 官方 SDK（common）。
上报 /status + 视频流 /video.mjpeg（http://<IP>:5000/video.mjpeg）。

用法（先 sudo systemctl stop spiderpi）：
    python3 AutonomousCrawling.py
"""
import time
import threading

import cv2
import numpy as np

from common.ros_robot_controller_sdk import Board


# 目标舵机脉宽（21/22/23/24）
GRAB = {21: 500, 22: 400, 23: 500, 24: 250}   # 21 向左 +15 脉宽（485->500）
# 官方初始位置（复位，取自 robot_params.yaml arm.reset_pulses）
RESET = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_OPEN = 120    # 25 号张开
GRIPPER_CLOSE = 700   # 25 号闭合（拉满）
HOLD_SEC = 1.0        # 夹住保持时长（秒）
TURN_LEFT_21 = 900    # 夹取后 21 左转目标脉宽（>500 朝左）
LOOK_UP_24 = 500      # 左转后 24 抬头看人脸的脉宽（>330 朝上）
FACE_TIMEOUT = 15.0   # 人脸检测超时（秒）
HAND_WAIT = 2.0       # 识别人脸后等待伸手的固定时长（秒）
HAND_MIN_AREA = 5000  # 手掌肤色区域最小面积（已停用手掌检测，保留备用）
# 递物位姿（21-24，占位：保持臂抬起、24 手腕朝下朝向手掌，需现场调）
HANDOVER = {21: 900, 22: 345, 23: 575, 24: 280}


STATUS = {'state': 'Grab', 'message': '自动抓取', 'last_result': None}
_LATEST_JPEG = None
_JPEG_LOCK = threading.Lock()


def set_status(**kw):
    STATUS.update(kw)


def publish_frame(frame):
    """把一帧压成 JPEG 存共享区，供 /video.mjpeg 推流。"""
    global _LATEST_JPEG
    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    if ok:
        with _JPEG_LOCK:
            _LATEST_JPEG = jpg.tobytes()


def start_server():
    """内联 Flask：/status 状态 + /video.mjpeg 视频流。"""
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


class Camera:
    """后台线程连续取帧（存最新帧）+ 推流（10fps 限流）。"""

    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_pub = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self.frame = f
                now = time.time()
                if now - self._last_pub >= 0.1:  # 10 fps
                    self._last_pub = now
                    publish_frame(f)
            time.sleep(0.01)

    def read(self):
        with self._lock:
            return self.frame

    def stop(self):
        self._stop.set()


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


def move(board, servos, sec):
    """按给定脉宽移动舵机；sec 为移动时长（秒），阻塞到移动完成。"""
    board.bus_servo_set_position(sec, [[sid, p] for sid, p in servos])
    time.sleep(sec + 0.1)


def reset_arm(board):
    """恢复官方初始位置：机械臂复位 + 夹爪张开。"""
    move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]),
                 (24, RESET[24]), (25, GRIPPER_OPEN)], 1.5)


def detect_face(cascade, frame):
    """OpenCV Haar 级联检测人脸，返回是否有脸。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = cascade.detectMultiScale(gray, 1.1, 6, minSize=(80, 80))
    return len(faces) > 0


def detect_hand(frame):
    """肤色(HSV)检测手掌：返回是否有较大的肤色区域（手）。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 20, 70], dtype=np.uint8)
    upper = np.array([20, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > HAND_MIN_AREA:
            return True
    return False


def main():
    board = Board()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    for _ in range(5):
        cap.read()
    cam = Camera(cap)

    start_server()
    set_status(state='Grab', message='固定路线夹取')

    try:
        # 1) 恢复官方初始位置
        reset_arm(board)

        # 2) 21 先动
        move(board, [(21, GRAB[21])], 0.8)

        # 3) 22 和 23 一起动
        move(board, [(22, GRAB[22]), (23, GRAB[23])], 1.2)

        # 4) 24 到 270
        move(board, [(24, GRAB[24])], 0.8)

        # 5) 闭合夹爪（25）
        move(board, [(25, GRIPPER_CLOSE)], 1.0)
        set_status(last_result='done', message='已夹取')
        time.sleep(HOLD_SEC)

        # 6) 恢复机械臂初始位置（21-24 复位，夹爪保持闭合）
        move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]), (24, RESET[24])], 1.5)
        set_status(message='已恢复，准备左转')

        # 7) 21 左转（面向人）
        move(board, [(21, TURN_LEFT_21)], 1.0)
        set_status(message='已左转')

        # 8) 抬头看（24 向上看人脸）
        move(board, [(24, LOOK_UP_24)], 1.0)
        set_status(message='已抬头，识别人脸')

        # 9) 识别人脸（OpenCV Haar 级联）
        face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        found = False
        deadline = time.time() + FACE_TIMEOUT
        while time.time() < deadline:
            f = cam.read()
            if f is not None and detect_face(face_cascade, f):
                found = True
                break
            time.sleep(0.05)
        # 9) 等待伸手（固定时长，不识别手掌）-> 调整机械臂 -> 松开夹爪放到手上
        if found:
            set_status(message='已识别人脸，等待伸手')
            time.sleep(HAND_WAIT)

            move(board, [(21, HANDOVER[21]), (22, HANDOVER[22]),
                         (23, HANDOVER[23]), (24, HANDOVER[24])], 1.5)  # 调整机械臂朝向手掌
            move(board, [(25, GRIPPER_OPEN)], 1.0)  # 松开夹爪
            set_status(last_result='done', message='已放置')
            print('夹取成功', flush=True)
        else:
            set_status(last_result='failed', message='未检测到人脸')

        # 10) 恢复机械臂位置（21-24 复位，夹爪已张开）
        move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]), (24, RESET[24])], 1.5)
        set_status(message='已完成')
    finally:
        cam.stop()
        cap.release()


if __name__ == '__main__':
    main()
