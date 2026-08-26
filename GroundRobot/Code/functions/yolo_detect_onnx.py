#!/usr/bin/python3
# coding=utf8
"""ONNX 版 YOLO 检测器（树莓派本地推理，不依赖 torch/ultralytics）。

与 yolo_detect.py（ultralytics 版）接口一致：detect() 返回
dict(center, area, color, conf) 或 None，供 autonomous_pick 无缝切换。

依赖：树莓派执行 `pip3 install onnxruntime`（aarch64 官方 wheel）。
模型：MacYoLo 训练产物 best.onnx（单类 worm）。

用法：
    单张图无头测试：
        python3 functions/yolo_detect_onnx.py --model models/worm_best.onnx --image /tmp/x.jpg
    摄像头实时（VNC 终端）：
        python3 functions/yolo_detect_onnx.py --model models/worm_best.onnx
"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2
import numpy as np
import onnxruntime as ort


class YoloDetector:
    """ONNX YOLO 检测器，接口与 ultralytics 版 YoloDetector 对齐。"""

    def __init__(self, model_path, names=('worm',), conf=0.35, size=(640, 640)):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4  # Pi5 四核并行，约 2.5→4.4 FPS
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        self.output_name = self.sess.get_outputs()[0].name
        self.names = names
        self.nc = len(names)
        self.conf = conf
        self.size = size

    def _letterbox(self, img):
        """等比缩放到 640x640 并补灰边，返回 (画布, 缩放比, pad_x, pad_y)。"""
        h0, w0 = img.shape[:2]
        sw, sh = self.size
        r = min(sw / w0, sh / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        pad_x = (sw - new_w) // 2
        pad_y = (sh - new_h) // 2
        canvas = np.full((sh, sw, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(img, (new_w, new_h))
        return canvas, r, pad_x, pad_y

    @staticmethod
    def _nms(boxes, iou_thr=0.45):
        """简单 IoU NMS，boxes 每项为 [x1, y1, x2, y2, score, cls]。"""
        boxes = sorted(boxes, key=lambda b: -b[4])
        keep = []
        while boxes:
            best = boxes.pop(0)
            keep.append(best)
            rest = []
            for b in boxes:
                x1 = max(best[0], b[0])
                y1 = max(best[1], b[1])
                x2 = min(best[2], b[2])
                y2 = min(best[3], b[3])
                inter = max(0, x2 - x1) * max(0, y2 - y1)
                area1 = (best[2] - best[0]) * (best[3] - best[1])
                area2 = (b[2] - b[0]) * (b[3] - b[1])
                iou = inter / (area1 + area2 - inter + 1e-6)
                if iou < iou_thr:
                    rest.append(b)
            boxes = rest
        return keep

    def detect(self, img, min_area=100):
        """检测目标，返回 dict(center, area, color, conf) 或 None，并在 img 上绘制。"""
        h0, w0 = img.shape[:2]
        canvas, r, pad_x, pad_y = self._letterbox(img)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run([self.output_name], {self.input_name: blob})[0][0]  # [4+nc, 8400]

        cands = []
        for i in range(out.shape[1]):
            cx, cy, w, h = out[0, i], out[1, i], out[2, i], out[3, i]
            scores = out[4:4 + self.nc, i]
            cls = int(scores.argmax())
            score = float(scores[cls])
            if score < self.conf:
                continue
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
            cands.append([x1, y1, x2, y2, score, cls])

        keep = self._nms(cands)
        if not keep:
            return None
        x1, y1, x2, y2, score, cls = keep[0]
        area = (x2 - x1) * (y2 - y1)
        if area < min_area:
            return None
        center = (int((x1 + x2) // 2), int((y1 + y2) // 2))
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(img, '%s %.2f' % (self.names[cls], score),
                    (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return {'center': center, 'area': float(area), 'color': self.names[cls], 'conf': score}


def main():
    parser = argparse.ArgumentParser(description='ONNX YOLO 检测（树莓派本地）')
    parser.add_argument('--model', default='/home/pi/spiderpi/models/worm_best.onnx',
                        help='ONNX 模型路径')
    parser.add_argument('--conf', type=float, default=0.35, help='置信度阈值')
    parser.add_argument('--min-area', type=int, default=100, help='最小目标面积')
    parser.add_argument('--image', default=None, help='测单张图（无头模式）')
    args = parser.parse_args()

    detector = YoloDetector(args.model, conf=args.conf)

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit('读图失败: %s' % args.image)
        r = detector.detect(img, min_area=args.min_area)
        print('检测结果:', r)
        out_path = args.image.rsplit('.', 1)[0] + '_onnx.jpg'
        cv2.imwrite(out_path, img)
        print('已保存标注图:', out_path)
        return

    from calibration.camera import Camera
    from functions.vision_utils import load_undistort_maps, correct_camera
    from functions.robot_config import load_params
    mapx, mapy = load_undistort_maps()
    rotate = load_params()['vision'].get('camera_rotate', 0)
    camera = Camera()
    camera.camera_open()
    try:
        while True:
            frame = camera.frame
            if frame is None:
                time.sleep(0.01)
                continue
            frame = correct_camera(frame, rotate)
            img = cv2.remap(frame.copy(), mapx, mapy, cv2.INTER_LINEAR)
            detector.detect(img, min_area=args.min_area)
            cv2.imshow('OnnxYoloDetect', img)
            if cv2.waitKey(1) == 27:
                break
    finally:
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
