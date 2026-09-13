#!/usr/bin/python3
# coding=utf8
"""完整建图采集：原地 360° 建模周围 + 沿路线补充路线区域（跳过夹取/放下）。

流程：
    阶段一：原地转身 360°（默认 12 视角 × 30°），采集周围一圈（含遮挡）。
    阶段二：沿 fixed_route.json 走一遍，只执行 forward/back/turn，跳过 pick/place，
            采集路线区域。
所有帧保存 pts + 里程计位姿 (R, t)，同一坐标系（起点相机坐标系）。

位姿约定：相机坐标系 X右 Y上 Z前（深度）；世界 = 起点相机坐标系。

用法（先 sudo systemctl stop spiderpi）：
    python3 CS-full-capture.py --route CompetitionUse/fixed_route.json --out /tmp/pcl_full
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


def capture_view(cam, outdir, idx, R, t):
    """拍一帧深度点云 + 位姿，存 view_%03d.npz。"""
    d = cam.read_depth(timeout_ms=2000)
    if d is None:
        return False
    pcl = cam.depth_to_pointcloud(d)
    valid = ~np.isnan(pcl[:, :, 0])
    pts = pcl[valid].reshape(-1, 3).astype(np.float32)
    np.savez_compressed(os.path.join(outdir, 'view_%03d.npz' % idx),
                        pts=pts, R=R, t=t)
    return True


def main():
    parser = argparse.ArgumentParser(description='完整建图采集')
    parser.add_argument('--route', default='fixed_route.json')
    parser.add_argument('--out', default='/tmp/pcl_full')
    parser.add_argument('--views', type=int, default=12, help='原地 360° 视角数')
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

    R = np.eye(3, dtype=np.float32)   # 累计旋转
    t = np.zeros(3, dtype=np.float32)  # 累计平移
    idx = 0

    try:
        # ===== 阶段一：原地 360° 采集周围 =====
        print('===== 阶段一：原地 360° 采集（%d 视角）=====' % args.views, flush=True)
        step = 360.0 / args.views
        for v in range(args.views):
            time.sleep(0.5)  # 站稳
            ok = capture_view(cam, args.out, idx, R, t)
            print('原地视角 %d/%d %s' % (v + 1, args.views, 'OK' if ok else 'FAIL'), flush=True)
            idx += 1
            if v < args.views - 1:
                turn_left(ik, angle=step, speed=60)
                time.sleep(0.5)
                R = rot_y(step).astype(np.float32) @ R

        # 转完 360° 回到起点朝向（名义 R≈eye，t=0；ICP 建图会精修小误差）
        R = np.eye(3, dtype=np.float32)
        t = np.zeros(3, dtype=np.float32)

        # 等待用户手动检查/调整机器人位置（回车确认后继续）
        input('原地 360° 采集完成。请手动检查/调整机器人位置（不偏移不漂移），'
              '确认后回车继续沿路线采集...')

        # ===== 阶段二：沿路线采集补充 =====
        print('===== 阶段二：沿路线采集 =====', flush=True)
        for i, act in enumerate(actions):
            name = act.get('action')
            if name == 'forward':
                s = int(act.get('step', 100))
                go_forward(ik, step=s, speed=50, times=1)
                time.sleep(0.5)
                t = t + R @ np.array([0, 0, s], dtype=np.float32)
            elif name == 'back':
                s = int(act.get('step', 50))
                go_back(ik, step=s, speed=50)
                time.sleep(0.5)
                t = t + R @ np.array([0, 0, -s], dtype=np.float32)
            elif name == 'turn_left':
                a = int(act.get('angle', 90))
                turn_left(ik, angle=a, speed=60)
                time.sleep(0.5)
                R = rot_y(a).astype(np.float32) @ R
            elif name == 'turn_right':
                a = int(act.get('angle', 90))
                turn_right(ik, angle=a, speed=60)
                time.sleep(0.5)
                R = rot_y(-a).astype(np.float32) @ R
            elif name in ('pick', 'place', 'stand'):
                continue
            else:
                continue

            ok = capture_view(cam, args.out, idx, R, t)
            print('路线 %d/%d %s -> view_%03d %s' % (i + 1, len(actions), name, idx, 'OK' if ok else 'FAIL'), flush=True)
            idx += 1
    finally:
        stand(ik)
        cam.close()

    print('采集完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)


if __name__ == '__main__':
    main()
