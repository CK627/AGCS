#!/usr/bin/python3
# coding=utf8
"""树莓派摄像头实时 YOLO 检测 + 跟踪可视化。

在 VNC 终端运行：
    export DISPLAY=:0
    cd ~/spiderpi/YOLO/Code
    python3 live_detect.py --model ../Model/worm_best.onnx --conf 0.35
按 q 退出。
"""
import argparse
import os
import sys
import time

_CODE_ROOT = os.path.dirname(os.path.abspath(__file__))
_SPIDERPI_ROOT = os.path.dirname(os.path.dirname(_CODE_ROOT))
if _CODE_ROOT not in sys.path:
    sys.path.insert(0, _CODE_ROOT)
if _SPIDERPI_ROOT not in sys.path:
    sys.path.insert(0, _SPIDERPI_ROOT)

import cv2

from agcs_lib import make_board, load_params, load_undistort_maps, correct_camera, open_camera, capture
from ylib import YoloDetector


def main():
    parser = argparse.ArgumentParser(description='树莓派摄像头实时 YOLO 检测')
    parser.add_argument('--model', default='../Model/worm_best.onnx')
    parser.add_argument('--conf', type=float, default=0.35)
    parser.add_argument('--min-area', type=int, default=100)
    args = parser.parse_args()

    board = make_board()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    mapx, mapy = load_undistort_maps()
    cam = open_camera()
    detector = YoloDetector(args.model, conf=args.conf)

    last_center = None
    try:
        while True:
            f = capture(cam)
            if f is None:
                time.sleep(0.01)
                continue
            img = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
            r = detector.detect(img, min_area=args.min_area)
            if r is not None:
                cx, cy = r['center']
                cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)
                if last_center is not None:
                    cv2.line(img, last_center, (cx, cy), (255, 0, 0), 2)
                last_center = (cx, cy)
                cv2.putText(img, '%s %.2f' % (r.get('color', 'worm'), r.get('conf', 0)),
                            (cx - 40, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('YOLO live', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
