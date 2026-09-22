#!/usr/bin/python3
# coding=utf8
"""vision_common.py —— 视觉 / 推流 / YOLO 检测的共享封装。

Auto-capture.py 和 stream.py 共用：只依赖 agcs_lib 与 communication.task_server，
不碰舵机、不碰走路/夹取逻辑。抽出来是为了让两个脚本不再各自复制取帧 / 去畸变 / 推流代码。

约定：
- open_vision 返回 (cam, read_frame, detector, publish)，颜色导航和 YOLO 共用同一路
  read_frame（只开一个 /dev/video0）。
- detect() / detector() 内部都会推流标注画面，不用另外再推。
"""

import os
import sys
import threading
import time

import cv2
import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    load_params,
    load_lab_data,
    load_undistort_maps,
    detect_color,
    correct_camera,
    open_camera,
    capture,
)

try:
    from communication import task_server
except ImportError:
    task_server = None


camera_lock = threading.Lock()


def open_vision(color, min_area):
    """打开摄像头，返回 (cam, read_frame, detector, publish)。

    read_frame：读一帧并去畸变（颜色导航和 YOLO 共用这一路，只开一个 /dev/video0）。
    detector：颜色检测（融合导航用），内部调 read_frame + 推流。
    publish：推流（YOLO 检测也用它推标注画面）。
    """
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    def read_frame():
        with camera_lock:
            f = capture(cam)
        if f is None:
            return None
        return cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)

    def publish(frame):
        if task_server is not None:
            task_server.publish_frame(frame, max_fps=10.0)
            task_server.publish_lab_frame(lab_view(frame, lab, color), max_fps=10.0)

    def detector():
        frame = read_frame()
        if frame is None:
            return None
        result = detect_color(frame, lab, color, min_area=min_area)
        if result is not None:
            x, y, w, h = cv2.boundingRect(result['contour'])
            ul, ur = x, x + w
            result['bbox_center_x'] = (ul + ur) / 2.0
            cx, cy = result['center']
            cv2.circle(frame, (cx, cy), int(result.get('radius', 20)), (0, 255, 0), 2)
            cv2.putText(frame, color, (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        publish(frame)
        return result

    return cam, read_frame, detector, publish


def depth_cm_at(depth, cx, cy, rotate):
    """读深度相机在彩色像素 (cx,cy) 处的距离(cm)；无有效读数返回 None。"""
    try:
        d = depth.read_depth(timeout_ms=200)
        if d is None:
            return None
        if rotate:
            d = correct_camera(d, rotate)
        h, w = d.shape
        dx = min(w - 1, max(0, int(round(cx))))
        dy = min(h - 1, max(0, int(round(cy))))
        z_mm = int(d[dy, dx])
        if z_mm <= 0:
            return None
        return z_mm / 10.0
    except Exception:
        return None


class ModelDetector:
    """ONNX YOLO 检测器（onnxruntime 本地推理），detect() 返回 {'x','y','w','h','conf'} 或 None。

    独立自包含（只依赖 agcs_lib），已顶替原 NO6/NO7 的 Auto-capture.py / Auto-capture-1.py。
    """

    NAME = 'fake bug'  # 目标类别名

    def __init__(self, model_path, conf, classes, read_frame, publish):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4  # Pi5 四核并行
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        self.output_name = self.sess.get_outputs()[0].name
        self.conf = conf
        self.classes = set(classes) if classes else None
        self.read_frame = read_frame
        self.publish = publish
        shp = self.sess.get_inputs()[0].shape
        self.in_h, self.in_w = int(shp[2]), int(shp[3])

    def _letterbox(self, img):
        """等比缩放到模型输入尺寸补灰边，返回 (画布, 缩放比, pad_x, pad_y)。"""
        h0, w0 = img.shape[:2]
        ih, iw = self.in_h, self.in_w
        r = min(iw / w0, ih / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        pad_x, pad_y = (iw - new_w) // 2, (ih - new_h) // 2
        canvas = np.full((ih, iw, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(img, (new_w, new_h))
        return canvas, r, pad_x, pad_y

    def detect(self):
        """检测一帧，返回 {'x','y','w','h','conf','name'} 或 None，并推流标注画面。"""
        if self.classes and self.NAME not in self.classes:
            return None
        frame = self.read_frame()
        if frame is None:
            return None
        h0, w0 = frame.shape[:2]
        canvas, r, pad_x, pad_y = self._letterbox(frame)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run([self.output_name], {self.input_name: blob})[0][0]  # [4+nc, 8400]

        nc = out.shape[0] - 4   # 类别数（单类=1，多类=16）
        best = None  # (x1, y1, x2, y2, score)
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
            x1 = max(0, min(w0, x1))
            y1 = max(0, min(h0, y1))
            x2 = max(0, min(w0, x2))
            y2 = max(0, min(h0, y2))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            if best is None or score > best[4]:
                best = (x1, y1, x2, y2, score)

        result = None
        if best is not None:
            x1, y1, x2, y2, score = best
            result = {'x': int(x1), 'y': int(y1), 'w': int(x2 - x1), 'h': int(y2 - y1),
                      'conf': float(score), 'name': self.NAME}
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(frame, '%s %.2f' % (self.NAME, score), (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        self.publish(frame)
        return result


def video_loop(detectors, stop_event):
    """后台持续取帧推流：颜色 + YOLO 都叠加推流显示。"""
    while not stop_event.is_set():
        for d in detectors:
            d()
        time.sleep(0.1)


def lab_view(frame, lab, color):
    """生成 LAB 阈值图，只保留识别到的颜色区域。"""
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.bitwise_and(frame, frame, mask=mask)
