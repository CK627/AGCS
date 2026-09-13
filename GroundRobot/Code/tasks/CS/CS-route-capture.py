#!/usr/bin/python3
# coding=utf8
"""沿固定路线采集深度点云（跳过夹取/放下），记录里程计位姿，供建图。

机器人沿 fixed_route.json 走一遍，只执行 forward/back/turn_left/turn_right，
跳过 pick/place/stand。每走一步采集一帧深度点云，并把从路线推算的里程计
位姿 (R, t) 一起存进 view_*.npz。

位姿约定：相机坐标系 X右 Y上 Z前（深度）；世界 = 起点相机坐标系。

用法（先 sudo systemctl stop spiderpi）：
    python3 CS-route-capture.py --route fixed_route.json --out /tmp/pcl_route
"""
import argparse
import json
import os
import sys
import time

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (DepthCamera, make_board, make_ik, stand,
                      turn_left, turn_right, go_forward, go_back)
from agcs_lib.pcl import rot_y


def main():
    parser = argparse.ArgumentParser(description='沿路线采集点云')
    parser.add_argument('--route', default='fixed_route.json')
    parser.add_argument('--out', default='/tmp/pcl_route')
    args = parser.parse_args()

    with open(args.route, 'r', encoding='utf-8') as f:
        actions = json.load(f)
    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)

    R = np.eye(3, dtype=np.float32)   # 累计旋转（起点相机坐标系）
    t = np.zeros(3, dtype=np.float32)  # 累计平移

    idx = 0
    try:
        for i, act in enumerate(actions):
            name = act.get('action')
            if name == 'forward':
                step = int(act.get('step', 100))
                go_forward(ik, step=step, speed=50, times=1)
                time.sleep(0.5)
                t = t + R @ np.array([0, 0, step], dtype=np.float32)
            elif name == 'back':
                step = int(act.get('step', 50))
                go_back(ik, step=step, speed=50)
                time.sleep(0.5)
                t = t + R @ np.array([0, 0, -step], dtype=np.float32)
            elif name == 'turn_left':
                angle = int(act.get('angle', 90))
                turn_left(ik, angle=angle, speed=60)
                time.sleep(0.5)
                R = rot_y(angle).astype(np.float32) @ R
            elif name == 'turn_right':
                angle = int(act.get('angle', 90))
                turn_right(ik, angle=angle, speed=60)
                time.sleep(0.5)
                R = rot_y(-angle).astype(np.float32) @ R
            elif name in ('pick', 'place', 'stand'):
                continue
            else:
                continue

            d = cam.read_depth(timeout_ms=2000)
            if d is None:
                print('步骤 %d/%d %s 深度读取失败' % (i + 1, len(actions), name), flush=True)
                continue
            pcl = cam.depth_to_pointcloud(d)
            valid = ~np.isnan(pcl[:, :, 0])
            pts = pcl[valid].reshape(-1, 3).astype(np.float32)
            np.savez_compressed(os.path.join(args.out, 'view_%03d.npz' % idx),
                                pts=pts, R=R, t=t)
            print('步骤 %d/%d %s -> view_%03d (%d 点)' % (i + 1, len(actions), name, idx, len(pts)), flush=True)
            idx += 1
    finally:
        stand(ik)
        cam.close()
    print('采集完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)


if __name__ == '__main__':
    main()
