#!/usr/bin/python3
# coding=utf8
"""2.3 自动抓取：先视觉追踪居中，再前进靠近，距离达阈值夹取。

流程（两段式）：
    复位（夹爪张开）→ 相机转到初始角度
    → 阶段一：21/24 追踪把虫子锁在画面中心（连续 CENTER_HOLD 帧 ±CENTER_TOL 内）
    → 阶段二：22/23 展开前进（高度随展开变化），24 联动保持夹爪水平，21 不动
      → 前方距离（=焦距×虫子高度/框高）达到 --distance 阈值 → 闭合夹爪
    → 保持 → 复位（夹爪保持闭合）。

依赖：官方 SDK(common) + cv2 + onnxruntime + flask（推流）。
上报 /status + 视频流 /video.mjpeg（http://<IP>:5000/video.mjpeg）。

用法（先 sudo systemctl stop spiderpi）：
    python3 AutonomousCrawling.py                            # 默认参数
    python3 AutonomousCrawling.py --distance 15 --tilt 330 --conf 0.4
    python3 AutonomousCrawling.py --model ""                # 不检测，纯固定脉宽夹取
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

# ---- 视觉追踪参数（阶段一：居中）----
TRACK_STEPS = 200       # 最多追踪步数（安全上限）
K_PAN = 0.1             # 21 横转增益（水平居中，小步不震荡）
K_TILT = 0.1            # 24 俯仰增益（竖直居中，小步不震荡）
DEADBAND = 30           # 死区：偏差在这个像素内就不纠正（避免过冲震荡）
CENTER_TOL = 30         # 中心判据：|cx-320|<30 且 |cy-240|<30 算居中（像素）
CENTER_HOLD = 5         # 连续多少帧居中才进入前进

# ---- 前进参数（阶段二：靠近）----
APPROACH_STEPS = 60     # 最多前进步数（安全上限）
APPROACH_D22 = 6        # 每步 22（肩）展开量
APPROACH_D23 = 6        # 每步 23（肘）展开量
F_PX = 838.0            # 焦距（像素，640 分辨率）
BUG_HEIGHT_CM = 5.0     # 虫子物理高度(cm)，用于距离估算
DISTANCE_THRESHOLD = 8.0    # 前方距离阈值(cm)，达到就夹
RELIABLE_MAX_H = 450    # 框高超过此值视为被裁（距离不可靠），改用外推
LEVEL_SUM = 1125        # 夹爪水平时 22+23+24 = 1125（alpha=0）

SLEEP_S = 0.25          # 追踪每步间隔（秒），拉长让舵机走完、不震荡
APPROACH_SLEEP = 0.12   # 前进每步间隔（秒），更快
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
                 (24, RESET[24]), (25, GRIPPER_OPEN)], 0.8)


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
    parser = argparse.ArgumentParser(description='2.3 自动抓取（视觉追踪 + 靠近 + 距离阈值）')
    parser.add_argument('--model', default='models/v8n.onnx',
                        help='YOLO ONNX 模型路径；传空串 "" 则不检测')
    parser.add_argument('--conf', type=float, default=0.5, help='YOLO 置信度阈值')
    parser.add_argument('--distance', type=float, default=DISTANCE_THRESHOLD,
                        help='前方距离阈值(cm)，达到就夹，默认 %.1f' % DISTANCE_THRESHOLD)
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
    set_status(state='Grab', message='自动抓取（视觉追踪 + 靠近 + 距离阈值）')

    try:
        # 1) 复位 + 相机转到初始角度
        reset_arm(board)
        time.sleep(0.5)
        print('  [动作] 24 号转到 %d（初始角度）' % args.tilt, flush=True)
        board.bus_servo_set_position(0.3, [[24, args.tilt]])
        time.sleep(0.3)

        # 2) 阶段一：视觉追踪居中（21 左右、24 上下）
        x_dis, y_dis = 500, args.tilt       # 21/24 当前值
        centered = False
        hold = 0
        print('阶段一：追踪居中（±%dpx，连续 %d 帧）...' % (CENTER_TOL, CENTER_HOLD), flush=True)
        for step in range(TRACK_STEPS):
            _f, r = cam.read()
            if r is not None:
                cx, cy = r['center']
                err_x = 320 - cx
                err_y = 240 - cy
                # 死区：偏差在死区内就不纠正，避免过冲震荡
                if abs(err_x) > DEADBAND:
                    x_dis = max(0, min(1000, int(x_dis + K_PAN * err_x)))
                if abs(err_y) > DEADBAND:
                    y_dis = max(0, min(1000, int(y_dis + K_TILT * err_y)))
                if abs(cx - 320) < CENTER_TOL and abs(cy - 240) < CENTER_TOL:
                    hold += 1
                else:
                    hold = 0
                print('  追踪%03d: 中心=(%.0f,%.0f) 偏差=(%+.0f,%+.0f) | 21=%d 24=%d'
                      % (step, cx, cy, cx - 320, cy - 240, x_dis, y_dis), flush=True)
                if hold >= CENTER_HOLD:
                    centered = True
                    print('  已居中', flush=True)
                    break
            board.bus_servo_set_position(SLEEP_S, [[21, x_dis], [24, y_dis]])
            time.sleep(SLEEP_S)
        if not centered:
            print('  追踪步数用尽仍未居中，仍进入前进', flush=True)

        # 3) 阶段二：前进靠近（22/23 展开，24 保持水平），前方距离达阈值夹
        #    距离可靠时直接测；目标丢失/框被裁时用「一致速度」外推
        w22, z23 = RESET[22], RESET[23]
        last_dist = 999.0      # 最近一次可靠距离
        dist_rate = 0.4        # 每步距离下降速率(cm/步)，实测自适应
        since_reliable = 0     # 距上次可靠测量过了多少步
        reached = False
        print('阶段二：前进靠近（距离阈值 %.1fcm）...' % args.distance, flush=True)
        for step in range(APPROACH_STEPS):
            _f, r = cam.read()
            cur = None
            if r is not None:
                w, h = r.get('w', 0.0), r.get('h', 0.0)
                if 0 < h < RELIABLE_MAX_H:   # 框未裁，距离可靠
                    d = F_PX * BUG_HEIGHT_CM / h
                    if last_dist < 999.0 and d < last_dist:
                        rate = last_dist - d
                        dist_rate = rate if dist_rate <= 0 else 0.7 * dist_rate + 0.3 * rate
                    last_dist = d
                    since_reliable = 0
                    cur = d
                else:
                    since_reliable += 1
            else:
                since_reliable += 1
            if cur is None:
                # 外推：按一致速度推算当前距离
                cur = max(0.0, last_dist - dist_rate * since_reliable)
            print('  前进%02d: 距离=%.1fcm | 21=%d 22=%d 23=%d 24=%d'
                  % (step, cur, x_dis, w22, z23, y_dis), flush=True)
            if cur <= args.distance:
                reached = True
                print('  距离达阈值，停止前进', flush=True)
                break
            # 前进：22 展开、23 伸展，24 联动保持夹爪水平（alpha=0）
            w22 = max(0, min(1000, int(w22 - APPROACH_D22)))
            z23 = max(0, min(1000, int(z23 + APPROACH_D23)))
            y_dis = max(0, min(1000, int(LEVEL_SUM - w22 - z23)))
            board.bus_servo_set_position(APPROACH_SLEEP, [[21, x_dis], [24, y_dis], [22, w22], [23, z23]])
            time.sleep(APPROACH_SLEEP)
        if not reached:
            print('  前进步数用尽仍未达阈值，用当前位夹取', flush=True)

        # 4) 闭合夹爪 + 保持
        move(board, [(25, GRIPPER_CLOSE)], 0.8)
        set_status(last_result='done', message='已夹取')
        time.sleep(HOLD_SEC)

        # 5) 复位（夹爪保持闭合）
        move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]), (24, RESET[24])], 0.8)
        set_status(last_result='done', message='已夹取并恢复')
        print('夹取成功', flush=True)
    finally:
        cam.stop()
        cap.release()


if __name__ == '__main__':
    main()
