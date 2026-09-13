#!/usr/bin/python3
# coding=utf8
"""录制 RGB-D 帧（深度 + 彩色），供 Mac 上 Open3D TSDF 重建。

深度走 OpenNI2（uint16 mm），彩色走 OpenCV /dev/video0。回车开始/结束，
每 interval 秒录一帧：depth_XXXXX.png（uint16 mm）+ color_XXXXX.jpg（RGB），
并把深度内参保存在 intrinsic.json。

用法（先 sudo systemctl stop spiderpi）：
    python3 record_rgbd.py --out /tmp/rgbd --interval 0.2
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera


def main():
    parser = argparse.ArgumentParser(description='录制 RGB-D 帧')
    parser.add_argument('--out', default='/tmp/rgbd')
    parser.add_argument('--interval', type=float, default=0.2, help='录帧间隔(秒)')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 深度内参
    fx, fy, cx, cy = cam.get_depth_intrinsics()
    with open(os.path.join(args.out, 'intrinsic.json'), 'w') as f:
        json.dump({'width': 640, 'height': 480, 'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}, f, indent=2)
    print('深度内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f' % (fx, fy, cx, cy), flush=True)

    input('按回车开始录制（每 %.2f 秒一帧）...' % args.interval)

    stop_event = threading.Event()
    threading.Thread(target=lambda: (input('按回车结束...'), stop_event.set()), daemon=True).start()

    idx = 0
    try:
        while not stop_event.is_set():
            d = cam.read_depth(timeout_ms=2000)
            ok, bgr = cap.read()
            if d is None or not ok:
                time.sleep(args.interval)
                continue
            cv2.imwrite(os.path.join(args.out, 'depth_%05d.png' % idx), d)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            cv2.imwrite(os.path.join(args.out, 'color_%05d.jpg' % idx), rgb,
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
            print('已存第 %d 帧' % idx, flush=True)
            idx += 1
            time.sleep(args.interval)
    finally:
        cap.release()
        cam.close()
    print('录制完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)


if __name__ == '__main__':
    main()
