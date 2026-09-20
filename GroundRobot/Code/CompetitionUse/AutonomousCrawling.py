#!/usr/bin/python3
# coding=utf8
"""2.3 自动抓取：视觉追踪居中 + 持续靠近 + 面积阈值停止。

流程：
    复位（夹爪张开）→ 相机转到初始角度 → 循环：
      模型检测虫子 → 21/24 追踪保持居中 → 22/23 逐步下降靠近
      → 虫子框面积占画面比例达到 --area-ratio 阈值 → 停止 → 闭合夹爪
    → 保持 → 复位（夹爪保持闭合）。

依赖：官方 SDK(common) + cv2 + onnxruntime + flask（推流）。
上报 /status + 视频流 /video.mjpeg（http://<IP>:5000/video.mjpeg）。

用法（先 sudo systemctl stop spiderpi）：
    python3 AutonomousCrawling.py                              # 默认参数
    python3 AutonomousCrawling.py --area-ratio 0.25 --tilt 330 --conf 0.4
    python3 AutonomousCrawling.py --model ""                  # 不检测，纯固定脉宽夹取
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

# ---- 复位位 / 夹爪 ----
RESET = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_OPEN = 120     # 25 号张开
GRIPPER_CLOSE = 700    # 25 号闭合
HOLD_SEC = 1.0         # 夹住保持时长

# ---- 靠近参数 ----
APPROACH_STEPS = 60     # 最多靠近步数（安全上限）
APPROACH_D22 = 10       # 每步 22（肩）下降量
APPROACH_D23 = 10       # 每步 23（肘）伸展量
K_PAN = 0.3             # 21 横转增益（水平居中）
K_TILT = 0.3            # 24 俯仰增益（竖直居中）
AREA_RATIO_THRESHOLD = 0.20   # 面积阈值（默认 20%）
SLEEP_S = 0.12          # 每步间隔（秒）
FRAME_W, FRAME_H = 640, 480


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
    """后台线程连续取帧 + 检测画框 + 推流；主线程经 read() 取「最新帧 + 检测结果」。

    检测只在后台线程做一次（避免与主线程并发调 onnx session），主线程读结果即可。
    """

    def __init__(self, cap, model_det=None):
        self.cap = cap
        self.model_det = model_det
        self.frame = None
        self.detection = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_pub = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                det = None
                if self.model_det is not None:
                    det = self.model_det.detect(f)   # 检测 + 画框（约几十 ms）
                with self._lock:
                    self.frame = f
                    self.detection = det
                now = time.time()
                if now - self._last_pub >= 0.1:      # 10 fps 推流
                    self._last_pub = now
                    publish_frame(f)
            time.sleep(0.01)

    def read(self):
        """返回 (frame, detection)；无检测时 detection 为 None。"""
        with self._lock:
            return self.frame, self.detection

    def stop(self):
        self._stop.set()


def move(board, servos, sec):
    """按给定脉宽移动舵机；sec 为移动时长（秒），阻塞到移动完成。"""
    board.bus_servo_set_position(sec, [[sid, p] for sid, p in servos])
    print('  [动作] 舵机%s 移动 %.1fs' % (servos, sec), flush=True)
    time.sleep(sec + 0.1)


def reset_arm(board):
    """恢复官方初始位置：机械臂复位 + 夹爪张开。"""
    move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]),
                 (24, RESET[24]), (25, GRIPPER_OPEN)], 1.5)


class ModelDetector:
    """ONNX YOLO 检测器。detect(frame) 返回 {'center':(cx,cy),'conf':..,'w':..,'h':..} 或 None。"""

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
    parser = argparse.ArgumentParser(description='2.3 自动抓取（视觉追踪 + 靠近 + 面积阈值）')
    parser.add_argument('--model', default='models/v8n.onnx',
                        help='YOLO ONNX 模型路径；传空串 "" 则不检测')
    parser.add_argument('--conf', type=float, default=0.5, help='YOLO 置信度阈值')
    parser.add_argument('--area-ratio', type=float, default=AREA_RATIO_THRESHOLD,
                        help='面积阈值(0~1)，默认 %.2f' % AREA_RATIO_THRESHOLD)
    parser.add_argument('--tilt', type=int, default=330,
                        help='初始 24 号俯仰脉宽（默认 330=水平朝前看）')
    args = parser.parse_args()

    board = Board()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()

    model_det = None
    if args.model:
        model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
        try:
            model_det = ModelDetector(model_path, args.conf)
            print('YOLO 模型已加载：%s（conf=%.2f）' % (model_path, args.conf), flush=True)
        except Exception as e:
            print('YOLO 模型加载失败：%s，退回固定脉宽夹取' % e, flush=True)
            model_det = None

    cam = Camera(cap, model_det)   # 传模型，视频推流画识别框
    start_server()
    set_status(state='Grab', message='自动抓取（视觉追踪 + 靠近 + 面积阈值）')

    try:
        # 1) 复位 + 相机转到初始角度
        reset_arm(board)
        time.sleep(0.5)
        print('  [动作] 24 号转到 %d（初始角度）' % args.tilt, flush=True)
        board.bus_servo_set_position(0.3, [[24, args.tilt]])
        time.sleep(0.3)

        # 2) 视觉追踪居中 + 持续下降靠近 + 面积阈值停止
        x_dis, y_dis = 500, args.tilt       # 21/24 当前值
        w22, z23 = RESET[22], RESET[23]     # 22/23 从复位位开始
        reached = False
        print('开始靠近（面积阈值 %.1f%%）...' % (args.area_ratio * 100), flush=True)
        for step in range(APPROACH_STEPS):
            _f, r = cam.read()
            if r is not None:
                cx, cy = r['center']
                w, h = r.get('w', 0.0), r.get('h', 0.0)
                ratio = (w * h) / (FRAME_W * FRAME_H)
                # 追踪保持居中：21 左右、24 上下
                x_dis = max(0, min(1000, int(x_dis + K_PAN * (320 - cx))))
                y_dis = max(0, min(1000, int(y_dis + K_TILT * (240 - cy))))
                print('  靠近%02d: 中心=(%.0f,%.0f) 框=%.0fx%.0f 占比=%.1f%% | 21=%d 24=%d 22=%d 23=%d'
                      % (step, cx, cy, w, h, ratio * 100, x_dis, y_dis, w22, z23), flush=True)
                if ratio >= args.area_ratio:
                    reached = True
                    board.bus_servo_set_position(0.15, [[21, x_dis], [24, y_dis], [22, w22], [23, z23]])
                    print('  面积达阈值，停止靠近', flush=True)
                    break
            # 下降一步
            w22 = max(0, min(1000, int(w22 - APPROACH_D22)))
            z23 = max(0, min(1000, int(z23 + APPROACH_D23)))
            board.bus_servo_set_position(SLEEP_S, [[21, x_dis], [24, y_dis], [22, w22], [23, z23]])
            time.sleep(SLEEP_S)

        if not reached:
            print('到步数上限仍未达阈值，用当前位夹取', flush=True)

        # 3) 闭合夹爪 + 保持
        move(board, [(25, GRIPPER_CLOSE)], 1.5)
        set_status(last_result='done', message='已夹取')
        time.sleep(HOLD_SEC)

        # 4) 复位（夹爪保持闭合）
        move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]), (24, RESET[24])], 1.5)
        set_status(last_result='done', message='已夹取并恢复')
        print('夹取成功', flush=True)
    finally:
        cam.stop()
        cap.release()


if __name__ == '__main__':
    main()
