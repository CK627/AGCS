#!/usr/bin/python3
# coding=utf8
"""自动抓取（夹爪逐步靠近 + 模型引导对准）。

流程：
    状态 Grab → 恢复官方初始位置（夹爪张开）→ 相机朝前下看
    → 夹爪分步下降靠近：22/23 从复位位插值到夹取位，每步模型微调 21/24 居中
      + 22/23 高度/前后补偿（框高估距离）
    → 慢慢闭合夹爪 → 保持 → 恢复机械臂初始位置（夹爪保持闭合）。

完全独立：只 import 标准库 + pip 库（cv2/flask/onnxruntime）+ 官方 SDK（common）。
上报 /status + 视频流 /video.mjpeg（http://<IP>:5000/video.mjpeg）。

用法（先 sudo systemctl stop spiderpi）：
    python3 AutonomousCrawling.py
    python3 AutonomousCrawling.py --model ""   # 不检测，纯固定脉宽夹取
"""
import os
import sys
import time
import threading
import argparse

import cv2
import numpy as np

from common.ros_robot_controller_sdk import Board

# spiderpi 根目录（模型在 ~/spiderpi/models/ 下）
if getattr(sys, 'frozen', False):
    _PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[0])))
else:
    _PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# 目标舵机脉宽（21/22/23/24）
GRAB = {21: 500, 22: 400, 23: 500, 24: 250}   # 21 向左 +15 脉宽（485->500）
# 官方初始位置（复位，取自 robot_params.yaml arm.reset_pulses）
RESET = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_OPEN = 120    # 25 号张开
GRIPPER_CLOSE = 700   # 25 号闭合（拉满）
HOLD_SEC = 1.0        # 夹住保持时长（秒）
# 前后（深度）补偿：框高度越小目标越远。NOMINAL_BBOX_H 是目标在正确夹取距离时的框高度
# （像素），现场标定；REACH_GAIN / HEIGHT_GAIN 是补偿增益（正负决定方向，现场调）。0 = 关闭补偿。
NOMINAL_BBOX_H = 0.0  # 目标在正确距离时的框高度（像素），0 表示关闭前后/高度补偿
REACH_GAIN = 0.3      # 框高度每差 1 像素，23（肘）调多少脉宽（前后）
HEIGHT_GAIN = 0.3     # 框高度每差 1 像素，22（肩）调多少脉宽（高度）
# 夹爪逐步靠近参数
APPROACH_STEPS = 10   # 夹爪从复位位分多少步下降到夹取位（步数越多越慢越平滑）
K_PAN = 0.2           # 21 横转增益（让目标在画面水平居中）
K_TILT = 0.2          # 24 俯仰增益（让目标在画面竖直居中）


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
    """后台线程连续取帧（存最新帧）+ 检测画框 + 推流（10fps 限流）。"""

    def __init__(self, cap, model_det=None):
        self.cap = cap
        self.frame = None
        self.model_det = model_det
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_pub = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                if self.model_det is not None:
                    self.model_det.detect(f)  # 画识别框（约 65ms）
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


class ModelDetector:
    """ONNX YOLO 检测器（onnxruntime 本地推理）。detect(frame) 返回 {'center':(cx,cy),'conf':..} 或 None。"""

    NAME = 'fake bug'

    def __init__(self, model_path, conf=0.5):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        self.output_name = self.sess.get_outputs()[0].name
        self.conf = conf
        shp = self.sess.get_inputs()[0].shape
        self.in_h, self.in_w = int(shp[2]), int(shp[3])

    def detect(self, frame):
        h0, w0 = frame.shape[:2]
        ih, iw = self.in_h, self.in_w
        r = min(iw / w0, ih / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        pad_x, pad_y = (iw - new_w) // 2, (ih - new_h) // 2
        canvas = np.full((ih, iw, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(frame, (new_w, new_h))
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run([self.output_name], {self.input_name: blob})[0][0]

        nc = out.shape[0] - 4
        best = None
        for i in range(out.shape[1]):
            scores = out[4:4 + nc, i]
            cls = int(scores.argmax())
            score = float(scores[cls])
            if score < self.conf:
                continue
            cx, cy, w, h = out[0, i], out[1, i], out[2, i], out[3, i]
            x1 = (cx - w / 2 - pad_x) / r
            y1 = (cy - h / 2 - pad_y) / r
            x2 = (cx + w / 2 - pad_x) / r
            y2 = (cy + h / 2 - pad_y) / r
            x1 = max(0, min(w0, x1)); y1 = max(0, min(h0, y1))
            x2 = max(0, min(w0, x2)); y2 = max(0, min(h0, y2))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            if best is None or score > best[4]:
                best = (x1, y1, x2, y2, score)

        if best is None:
            return None
        x1, y1, x2, y2, score = best
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        return {'center': (cx, cy), 'conf': float(score),
                'w': float(x2 - x1), 'h': float(y2 - y1)}


def main():
    parser = argparse.ArgumentParser(description='2.3 自动抓取+递物（夹取前用模型检测对准）')
    parser.add_argument('--model', default='models/v8n.onnx',
                        help='YOLO ONNX 模型路径；传空串 "" 则不检测直接固定脉宽夹')
    parser.add_argument('--conf', type=float, default=0.5, help='YOLO 置信度阈值')
    args = parser.parse_args()

    board = Board()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()

    # YOLO 模型（夹取前检测对准用；加载失败则不检测）
    model_det = None
    if args.model:
        model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
        try:
            model_det = ModelDetector(model_path, args.conf)
            print('YOLO 模型已加载：%s（conf=%.2f）' % (model_path, args.conf), flush=True)
        except Exception as e:
            print('YOLO 模型加载失败：%s，退回固定脉宽夹取' % e, flush=True)
            model_det = None

    cam = Camera(cap, model_det)   # 传模型给 Camera，推流画识别框

    start_server()
    set_status(state='Grab', message='自动抓取（夹取前检测对准）')

    try:
        # 1) 恢复官方初始位置（夹爪张开）
        reset_arm(board)
        time.sleep(0.5)
        # 相机朝前下看
        board.bus_servo_set_position(0.3, [[24, 260]])
        x_dis, y_dis = 500, 260   # 21/24 当前值

        # 2) 夹爪逐步靠近目标：22/23 从复位位插值到夹取位，每步模型微调 21/24/22/23
        for step in range(1, APPROACH_STEPS + 1):
            ratio = step / APPROACH_STEPS
            w22 = int(RESET[22] + (GRAB[22] - RESET[22]) * ratio)
            z23 = int(RESET[23] + (GRAB[23] - RESET[23]) * ratio)
            if model_det is not None:
                f = cam.read()
                if f is not None:
                    r = model_det.detect(f)
                    if r is not None:
                        cx, cy = r['center']
                        h = r.get('h', 0.0)
                        print('靠近 中心=(%.0f,%.0f) 框高=%.0f conf=%.2f'
                              % (cx, cy, h, r.get('conf', 0.0)), flush=True)
                        x_dis = max(0, min(1000, int(x_dis + K_PAN * (320 - cx))))
                        y_dis = max(0, min(1000, int(y_dis + K_TILT * (240 - cy))))
                        if NOMINAL_BBOX_H > 0:
                            # 框小（远）→ 22 更低、23 更伸；框大（近）→ 收回一点
                            w22 = max(0, min(1000, int(w22 + HEIGHT_GAIN * (NOMINAL_BBOX_H - h))))
                            z23 = max(0, min(1000, int(z23 + REACH_GAIN * (NOMINAL_BBOX_H - h))))
            board.bus_servo_set_position(0.2, [[21, x_dis], [24, y_dis], [22, w22], [23, z23]])
            time.sleep(0.25)

        # 3) 慢慢闭合夹爪
        move(board, [(25, GRIPPER_CLOSE)], 1.5)
        set_status(last_result='done', message='已夹取')
        time.sleep(HOLD_SEC)

        # 4) 恢复机械臂初始位置（21-24 复位，夹爪保持闭合）
        move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]), (24, RESET[24])], 1.5)
        set_status(last_result='done', message='已夹取并恢复')
        print('夹取成功', flush=True)
    finally:
        cam.stop()
        cap.release()


if __name__ == '__main__':
    main()
