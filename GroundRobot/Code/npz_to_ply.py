#!/usr/bin/python3
# coding=utf8
"""把 map.npz（pts + normals）转成彩色 .ply，方便 MeshLab / CloudCompare 查看。

按高度 Y 着色：低 -> 蓝，高 -> 红。

用法：
    python3 npz_to_ply.py --in map_cropped.npz --out map_cropped.ply
"""
import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser(description='npz 转 ply')
    parser.add_argument('--in', dest='inp', required=True)
    parser.add_argument('--out', default='map.ply')
    args = parser.parse_args()

    d = np.load(args.inp)
    pts = d['pts'].astype(np.float32)

    y = pts[:, 1]
    ymin, ymax = y.min(), y.max()
    norm = (y - ymin) / max(ymax - ymin, 1e-6)
    c = np.zeros((len(pts), 3), dtype=np.uint8)
    c[:, 2] = (255 * (1 - norm)).astype(np.uint8)  # B（低处）
    c[:, 0] = (255 * norm).astype(np.uint8)        # R（高处）

    with open(args.out, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write('element vertex %d\n' % len(pts))
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        for i in range(len(pts)):
            f.write('%.2f %.2f %.2f %d %d %d\n' % (pts[i, 0], pts[i, 1], pts[i, 2],
                                                    c[i, 0], c[i, 1], c[i, 2]))
    print('已生成 %s (%d 点)' % (args.out, len(pts)))


if __name__ == '__main__':
    main()
