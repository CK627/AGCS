#!/usr/bin/python3
# coding=utf8
"""合并四个角的 2D 扫描成一张完整地图。

场地是 W×H 的矩形（mm）。机器人依次放到四个角、**机头朝场地中心**，每个角用
scan_2d.py 扫 180°，存下来的是「机器人坐标系」的 2D 点（x 右 / y 前）。四份按
各自角的位姿搬到同一个世界系就拼成整张图。

世界系（俯视，x 向右、y 向上；y 轴 = 角 1 所在的那条边方向）：

    角 4 (0,H) ───────── 角 3 (W,H)
        │                      │
        │      场地中心         │
        │                      │
    角 1 (0,0) ───────── 角 2 (W,0)
              ← 边长 W →

四个角**逆时针**依次：角1 → 角2 → 角3（角1 的对角）→ 角4。
每个角机头都朝场地中心，所以位姿是定死的：

    角1 (0,0)  θ=+atan2(W,H)        角2 (W,0)  θ=-atan2(W,H)
    角4 (0,H)  θ=180°-atan2(W,H)    角3 (W,H)  θ=180°+atan2(W,H)

点变换：世界 = R(θ)@点 + C，R(θ) = [[cosθ, sinθ], [-sinθ, cosθ]]（θ 从 +y 转向 +x）。

用法：
    python3 merge_scans.py --w 2400 --h 2400 \
        --c1 /tmp/scan_c1.npz --c2 /tmp/scan_c2.npz \
        --c3 /tmp/scan_c3.npz --c4 /tmp/scan_c4.npz --out /tmp/map2d.npz

看完预览图（--png）如果发现整张图像「镜子里的」（左右反了），说明四个角是**顺时针**
走的，四个角都换成顺时针顺序重跑一次即可（或者加 --mirror）。
"""
import argparse
import math
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.pcl2d import voxel_2d
from agcs_lib.mapview import ascii_map, draw_map_png


def corner_pose(idx, w, h):
    """第 idx 个角（1..4）的世界位姿 (C(x,y), theta度)，机头朝中心。"""
    a = math.degrees(math.atan2(w, h))
    if idx == 1:
        return (0.0, 0.0), a
    if idx == 2:
        return (float(w), 0.0), -a
    if idx == 3:
        return (float(w), float(h)), 180.0 + a
    return (0.0, float(h)), 180.0 - a


def to_world(pts, c, theta_deg):
    """机器人系 2D 点 → 世界系：R(θ)@p + C。"""
    t = math.radians(theta_deg)
    ct, st = math.cos(t), math.sin(t)
    x = ct * pts[:, 0] + st * pts[:, 1]
    y = -st * pts[:, 0] + ct * pts[:, 1]
    return np.stack([x + c[0], y + c[1]], axis=1).astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description='合并四个角的 2D 扫描')
    parser.add_argument('--w', type=float, required=True, help='场地宽(角1→角2 边长, mm)')
    parser.add_argument('--h', type=float, required=True, help='场地长(角1→角4 边长, mm)')
    parser.add_argument('--c1', required=True, help='角1 的 npz')
    parser.add_argument('--c2', required=True, help='角2 的 npz')
    parser.add_argument('--c3', required=True, help='角3 的 npz')
    parser.add_argument('--c4', required=True, help='角4 的 npz')
    parser.add_argument('--out', default='/tmp/map2d.npz')
    parser.add_argument('--png', default='')
    parser.add_argument('--margin', type=float, default=800.0,
                        help='场地外这个范围的散点也保留(mm)，再远算噪声丢掉')
    parser.add_argument('--voxel', type=float, default=50.0, help='合并后下采样体素(mm)')
    parser.add_argument('--mirror', action='store_true',
                        help='四个角是顺时针走的，结果左右反了 → 加这个翻回来')
    args = parser.parse_args()

    parts = []
    for idx, path in enumerate([args.c1, args.c2, args.c3, args.c4], 1):
        if not os.path.exists(path):
            print('FAIL：找不到 %s' % path, flush=True)
            return
        d = np.load(path)
        pts = d['pts'].astype(np.float32)
        c, theta = corner_pose(idx, args.w, args.h)
        wpts = to_world(pts, c, theta)
        lo = np.array([-args.margin, -args.margin], dtype=np.float32)
        hi = np.array([args.w + args.margin, args.h + args.margin], dtype=np.float32)
        inside = ((wpts >= lo) & (wpts <= hi)).all(axis=1)
        print('角%d 位姿 C=(%.0f,%.0f) θ=%.1f° | %d 点 → 场内 %d 点（丢 %.0f%%）'
              % (idx, c[0], c[1], theta, len(pts), int(inside.sum()),
                 100.0 * (1 - inside.mean())), flush=True)
        parts.append(wpts[inside])

    pts = voxel_2d(np.vstack(parts), args.voxel)
    if args.mirror:
        pts = pts.copy()
        pts[:, 0] = args.w - pts[:, 0]

    marks = [(c[0], c[1], 'c%d' % i, (0, 0, 255))
             for i, (c, _) in enumerate([corner_pose(i, args.w, args.h)
                                         for i in (1, 2, 3, 4)], 1)]
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=pts, w=args.w, h=args.h)
    print('已存 %s (%d 点)，x %.0f..%.0f y %.0f..%.0f（场地 0..%.0f × 0..%.0f）'
          % (args.out, len(pts), pts[:, 0].min(), pts[:, 0].max(),
             pts[:, 1].min(), pts[:, 1].max(), args.w, args.h), flush=True)

    print(ascii_map(pts, marks=marks + [(args.w / 2.0, args.h / 2.0, 'C')],
                    title='合并地图（c1..c4=四个角，C=场地中心）%d 点' % len(pts)), flush=True)

    png = args.png or (os.path.splitext(args.out)[0] + '_merged.png')
    if png:
        if draw_map_png(pts, png, marks=marks):
            print('预览图 %s（拉回本地看：墙应该围成 %.0f×%.0f 的框）'
                  % (png, args.w, args.h), flush=True)


if __name__ == '__main__':
    main()
