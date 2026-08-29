#!/usr/bin/python3
# coding=utf8
"""YOLO 识别 + 云台跟踪主程序。

用法：
    python3 main.py --model ../Model/worm_best.onnx --conf 0.35
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
from ylib.tracker import ColorTracker


def main():
    parser = argparse.ArgumentParser(description='YOLO 识别 + 云台跟踪')
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

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detector.detect(f, min_area=args.min_area)

    tracker = ColorTracker(board, detect, dead_x=40, dead_y=60)
    tracker.start(500, 260)
    print('YOLO tracker started. Ctrl+C 退出')
    try:
        while True:
            r = tracker.latest()
            if r is not None:
                print('center=%s area=%.0f 21=%d 24=%d' % (
                    r['center'], r.get('area', 0), r['x_dis'], r['y_dis']))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        cam.camera_close()


if __name__ == '__main__':
    main()
