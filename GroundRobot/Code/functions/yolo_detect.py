#!/usr/bin/python3
# coding=utf8
"""YOLO 目标检测（视觉识别进阶，可选）。

对应 README 第三步"目标识别模型部署"：先用颜色检测跑通流程，
训练好豆荚螟危害特征模型后，用本模块替换颜色检测。
autonomous_pick.py 已支持 --detector yolo 切换。

依赖：pip3 install ultralytics（在树莓派上安装）。
"""
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2


class YoloDetector:
    """YOLO 检测器，接口与 detect_color 对齐（返回 dict 或 None）。"""

    def __init__(self, model_path='yolov8n.pt', target_classes=('pod',),
                 conf=0.45, size=(640, 480)):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit('未安装 ultralytics，请先在树莓派执行: pip3 install ultralytics')
        self.model = YOLO(model_path)
        self.target_classes = set(target_classes)
        self.conf = conf
        self.size = size

    def detect(self, img, min_area=100):
        """检测目标类别，返回 dict(center, area, color, conf) 或 None，并在 img 上绘制。"""
        results = self.model.predict(img, verbose=False, imgsz=self.size[0])[0]
        names = results.names
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if names.get(cls_id) not in self.target_classes or conf < self.conf:
                continue
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            area = (x2 - x1) * (y2 - y1)
            if area < min_area:
                continue
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = '%s %.2f' % (names.get(cls_id), conf)
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
            return {'center': center, 'area': area,
                    'color': names.get(cls_id), 'conf': conf}
        return None


def main():
    from calibration.camera import Camera
    from functions.vision_utils import load_undistort_maps

    detector = YoloDetector()
    mapx, mapy = load_undistort_maps()
    camera = Camera()
    camera.camera_open()
    try:
        while True:
            img = camera.frame
            if img is None:
                time.sleep(0.01)
                continue
            frame = cv2.remap(img.copy(), mapx, mapy, cv2.INTER_LINEAR)
            detector.detect(frame)
            cv2.imshow('YoloDetect', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    finally:
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
