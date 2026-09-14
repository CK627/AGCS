#!/usr/bin/python3
# coding=utf8
"""原地转一圈采集 RGB-D 帧（建图用）。

把机器人放到作业空间大致中心（不放植株），原地转身 360°，每转 step 度拍一帧
深度 + 彩色，存到 out 目录（depth_XXXXX.png + color_XXXXX.jpg + intrinsic.json）。
机器人自己转（不是人抱着走），相机高度恒定，重建更稳。

用法（先 sudo systemctl stop spiderpi）：
    python3 rotate_capture_rgbd.py --out /tmp/rgbd_360 --views 24
"""
import argparse
import json
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera, make_board, make_ik, stand, turn_left


def main():
    parser = argparse.ArgumentParser(description='原地转一圈采集 RGB-D')
    parser.add_argument('--out', default='/tmp/rgbd_360')
    parser.add_argument('--views', type=int, default=24, help='360° 视角数（每步 360/views 度）')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    for _ in range(5):
        cap.read()

    fx, fy, cx, cy = cam.get_depth_intrinsics()
    with open(os.path.join(args.out, 'intrinsic.json'), 'w') as f:
        json.dump({'width': 640, 'height': 480, 'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy},
                  f, indent=2)
    print('深度内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f' % (fx, fy, cx, cy), flush=True)

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)

    step = 360.0 / args.views
    idx = 0
    try:
        for v in range(args.views):
            time.sleep(0.5)  # 站稳
            d = cam.read_depth(timeout_ms=2000)
            ok, bgr = cap.read()
            if d is None or not ok:
                print('视角 %d/%d 读取失败' % (v + 1, args.views), flush=True)
            else:
                cv2.imwrite(os.path.join(args.out, 'depth_%05d.png' % idx), d)
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                cv2.imwrite(os.path.join(args.out, 'color_%05d.jpg' % idx), rgb,
                            [cv2.IMWRITE_JPEG_QUALITY, 90])
                print('视角 %d/%d 已存' % (v + 1, args.views), flush=True)
                idx += 1
            if v < args.views - 1:
                turn_left(ik, angle=step, speed=60)
                time.sleep(0.8)
    finally:
        cap.release()
        cam.close()
        stand(ik)
    print('采集完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)


if __name__ == '__main__':
    main()
