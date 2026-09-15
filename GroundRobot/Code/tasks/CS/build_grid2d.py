#!/usr/bin/python3
# coding=utf8
"""2D 建图：沿路线边走边采，2D ICP 增量建图（符合田间作业）。

机器人沿 fixed_route.json 走一遍（只走 forward/back/turn，跳过 pick/place/stand），
每走一步取深度点云压成 2D 墙/障碍轮廓，用 ICP 匹配到已建地图精化位姿，再累加。
输出 map2d.npz（pts: (N,2) 世界 mm，原点 = 起点）。

位姿：theta=航向(度, 0=+y 前)，t=位置(mm)。相机 2D 点(x右,y前) → 世界 =
R(theta)@点 + t。odometry 用「命令步数/角度」作初值，ICP 精化。

用法（先 sudo systemctl stop spiderpi）：
    python3 build_grid2d.py --route CompetitionUse/fixed_route.json --out /tmp/map2d.npz --pitch 45
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
from agcs_lib.pcl2d import depth_to_2d, voxel_2d, icp_2d


def _rot2(deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]], dtype=np.float32)


def capture_2d(cam, pitch, min_h, max_h):
    """采一帧深度压 2D，返回 (M,2) 相机系点。"""
    d = cam.read_depth(timeout_ms=2000)
    if d is None:
        return None
    pcl = cam.depth_to_pointcloud(d)
    valid = ~np.isnan(pcl[:, :, 0])
    pts3d = pcl[valid].reshape(-1, 3).astype(np.float32)
    # 高度范围（调试 pitch/高度带用）：墙应在 min_h..max_h 之间
    p = np.radians(pitch)
    R = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)],
                  [0, np.sin(p), np.cos(p)]], dtype=np.float32)
    h = (pts3d @ R.T)[:, 1]
    print('    高度 h %.0f..%.0f mm' % (h.min(), h.max()), flush=True)
    return depth_to_2d(pts3d, pitch, min_h, max_h)


def main():
    parser = argparse.ArgumentParser(description='2D 建图（沿路线边走边采）')
    parser.add_argument('--route', default='fixed_route.json')
    parser.add_argument('--out', default='/tmp/map2d.npz')
    parser.add_argument('--pitch', type=float, default=45.0, help='相机下俯角(度)')
    parser.add_argument('--min-h', type=float, default=-800.0, help='墙高度带下限(mm，负=相机下方)')
    parser.add_argument('--max-h', type=float, default=2000.0, help='墙高度带上限(mm)')
    parser.add_argument('--voxel', type=float, default=50.0, help='2D 下采样体素(mm)')
    args = parser.parse_args()

    with open(args.route, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)

    theta = 0.0            # 航向(度)
    t = np.zeros(2, dtype=np.float32)  # 位置(mm)
    map_parts = []         # 累计地图（世界系 2D 点）
    try:
        for i, act in enumerate(actions, 1):
            name = act.get('action')
            moved = False
            if name == 'forward':
                step = int(act.get('step', 100))
                fwd = np.array([np.sin(np.radians(theta)), np.cos(np.radians(theta))])
                t = t + step * fwd
                go_forward(ik, step=step, speed=50)
                moved = True
            elif name == 'back':
                step = int(act.get('step', 50))
                fwd = np.array([np.sin(np.radians(theta)), np.cos(np.radians(theta))])
                t = t - step * fwd
                go_back(ik, step=step)
                moved = True
            elif name == 'turn_left':
                angle = int(act.get('angle', 90))
                theta += angle
                turn_left(ik, angle=angle, speed=60)
                moved = True
            elif name == 'turn_right':
                angle = int(act.get('angle', 90))
                theta -= angle
                turn_right(ik, angle=angle, speed=60)
                moved = True
            else:
                continue  # pick/place/stand 跳过

            time.sleep(0.3)
            pts2d = capture_2d(cam, args.pitch, args.min_h, args.max_h)
            if pts2d is None or len(pts2d) < 20:
                print('%d/%d %s 2D点太少' % (i, len(actions), name), flush=True)
                continue

            # ICP 精化位姿（有地图后）
            if len(map_parts) > 0:
                map_all = voxel_2d(np.vstack(map_parts), args.voxel)
                res = icp_2d(pts2d, map_all, init_theta=np.radians(theta), init_t=t)
                if res is not None:
                    theta = np.degrees(res[0])
                    t = res[1]

            world = (_rot2(theta) @ pts2d.T).T + t
            map_parts.append(world)
            print('%d/%d %s theta=%.1f° t=(%.0f,%.0f) 2D点=%d'
                  % (i, len(actions), name, theta, t[0], t[1], len(pts2d)), flush=True)
    finally:
        cam.close()
        stand(ik)

    if not map_parts:
        print('FAIL：没有采到 2D 点', flush=True)
        return
    pts = voxel_2d(np.vstack(map_parts), args.voxel)
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=pts)
    print('已存 %s (%d 点)，x %.0f..%.0f y %.0f..%.0f'
          % (args.out, len(pts), pts[:, 0].min(), pts[:, 0].max(),
             pts[:, 1].min(), pts[:, 1].max()), flush=True)


if __name__ == '__main__':
    main()
