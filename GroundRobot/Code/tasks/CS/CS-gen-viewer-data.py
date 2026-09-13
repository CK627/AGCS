#!/usr/bin/python3
# coding=utf8
"""生成 3D 查看器数据：把多视角点云配准到全局坐标，按视角存成紧凑 npz。

输出 /tmp/viewer_data.npz：
    views_pts[i]  (N,3) float32  第 i 视角全局坐标点
    views_rgb[i]  (N,3) uint8    颜色
    poses[i]      (4,4)          位姿（R|t 扩展）
用法：python3 CS-gen-viewer-data.py --in /tmp/pcl_views --out /tmp/viewer_data.npz
"""
import argparse
import glob
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.pcl import register_sequence, apply_transform


def voxel_indices(points, voxel_size):
    """返回体素下采样后的点索引。"""
    vox = np.floor(points / voxel_size).astype(np.int64)
    _, idx = np.unique(vox, axis=0, return_index=True)
    return idx


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="indir", default="/tmp/pcl_views")
    parser.add_argument("--out", default="/tmp/viewer_data.npz")
    parser.add_argument("--step", type=float, default=30.0)
    parser.add_argument("--voxel", type=float, default=25.0, help="查看器下采样体素 mm")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.indir, "view_*.npz")))
    if len(files) < 2:
        print("FAIL: 至少 2 个视角，找到 %d" % len(files))
        sys.exit(1)

    clouds, rgbs = [], []
    for f in files:
        d = np.load(f)
        clouds.append(d["pts"].astype(np.float32))
        rgbs.append(d["rgb"].astype(np.uint8))

    poses = register_sequence(clouds, [args.step] * (len(clouds) - 1),
                              voxel_size=20.0)

    views_pts, views_rgb = [], []
    for i, (R, t) in enumerate(poses):
        p = apply_transform(clouds[i], R, t)
        r = rgbs[i]
        idx = voxel_indices(p, args.voxel)
        views_pts.append(p[idx].astype(np.float32))
        views_rgb.append(r[idx])
        print("视角 %d: %d 点" % (i, len(idx)))

    pose_mats = []
    for R, t in poses:
        M = np.eye(4, dtype=np.float32)
        M[:3, :3] = R
        M[:3, 3] = t
        pose_mats.append(M)

    np.savez_compressed(
        args.out,
        views_pts=np.array(views_pts, dtype=object),
        views_rgb=np.array(views_rgb, dtype=object),
        poses=np.array(pose_mats, dtype=np.float32),
    )
    total = sum(len(p) for p in views_pts)
    print("已存 %s，总 %d 点" % (args.out, total))
    print("DONE")


if __name__ == "__main__":
    main()
