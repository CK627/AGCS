#!/usr/bin/python3
# coding=utf8
"""摄像头推流 + 颜色检测标注（红/黄），只推流不动机器人。

浏览器打开 http://<机器人IP>:5000/video.mjpeg 看画面，检测到的色块会画圈标注。
用法：
    python3 tasks/CS/CS-color-track.py [--colors red,yellow]
"""
import argparse
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import open_camera, capture, load_params, correct_camera, load_undistort_maps
from agcs_lib.vision import load_lab_data, detect_color

try:
    from communication import task_server
except ImportError:
    task_server = None


def main():
    parser = argparse.ArgumentParser(description='推流 + 颜色检测标注')
    parser.add_argument('--colors', default='red,yellow', help='检测的颜色，逗号分隔')
    args = parser.parse_args()
    colors = [c.strip() for c in args.colors.split(',')]

    lab = load_lab_data()
    rotate = load_params()['vision'].get('camera_rotate', 0)
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    if task_server is not None:
        task_server.start_server()
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        finally:
            s.close()
        print('推流地址: http://%s:5000/video.mjpeg' % ip, flush=True)

    try:
        while True:
            f = capture(cam, tries=5)
            if f is None:
                time.sleep(0.05)
                continue
            frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
            for color in colors:
                r = detect_color(frame, lab, color, min_area=200)
                if r is not None:
                    cx, cy = r['center']
                    cv2.circle(frame, (cx, cy), r['radius'], (0, 255, 0), 2)
                    cv2.putText(frame, color, (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if task_server is not None:
                task_server.publish_frame(frame, max_fps=10.0)
    except KeyboardInterrupt:
        pass
    finally:
        cam.camera_close()


if __name__ == '__main__':
    main()
