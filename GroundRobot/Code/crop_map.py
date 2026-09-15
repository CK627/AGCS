#!/usr/bin/python3
# coding=utf8
"""裁剪/下采样点云地图（在 Mac 上跑），保留指定包围盒内的点。

地图坐标系 = 起点相机坐标系：X 右、Y 上、Z 前（深度），单位 mm。

用法（先看地图范围再定包围盒）：
    python3 crop_map.py --in map.npz --out map_cropped.npz \
        --xmin -2000 --xmax 2000 --zmin -1000 --zmax 4000 \
        --voxel 40
    缺省的某个轴不裁剪（保留该轴全部）。

裁剪完传回树莓派：
    scp map_cropped.npz pi@<IP>:/home/pi/spiderpi/models/map.npz
"""
import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser(description='裁剪/下采样地图')
    parser.add_argument('--in', dest='inp', default='map.npz')
    parser.add_argument('--out', default='map_cropped.npz')
    parser.add_argument('--xmin', type=float, default=None)
    parser.add_argument('--xmax', type=float, default=None)
    parser.add_argument('--ymin', type=float, default=None)
    parser.add_argument('--ymax', type=float, default=None)
    parser.add_argument('--zmin', type=float, default=None)
    parser.add_argument('--zmax', type=float, default=None)
    parser.add_argument('--voxel', type=float, default=0.0, help='下采样体素(mm)，0=不下采样')
    args = parser.parse_args()

    d = np.load(args.inp)
    pts = d['pts'].astype(np.float32)
    normals = d['normals'].astype(np.float32)
    print('原图点数:', len(pts))

    mask = np.ones(len(pts), dtype=bool)
    for axis, lo, hi in (('x', args.xmin, args.xmax),
                         ('y', args.ymin, args.ymax),
                         ('z', args.zmin, args.zmax)):
        if lo is not None or hi is not None:
            col = {'x': 0, 'y': 1, 'z': 2}[axis]
            if lo is not None:
                mask &= pts[:, col] >= lo
            if hi is not None:
                mask &= pts[:, col] <= hi

    pts = pts[mask]
    normals = normals[mask]
    print('裁剪后点数:', len(pts))

    if args.voxel > 0:
        vox = np.floor(pts / args.voxel).astype(np.int64)
        _, idx = np.unique(vox, axis=0, return_index=True)
        pts = pts[idx]
        normals = normals[idx]
        print('下采样后点数:', len(pts))

    np.savez_compressed(args.out, pts=pts, normals=normals)
    print('已存 %s' % args.out)


if __name__ == '__main__':
    main()
