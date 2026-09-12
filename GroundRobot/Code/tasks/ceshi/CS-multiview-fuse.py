#!/usr/bin/python3
# coding=utf8
"""多视角点云配准+融合：读采集的 .npz，ICP 配准到统一坐标系，存彩色 .ply。

离线脚本（不动机器人），可反复调参重跑。用法：
    python3 CS-multiview-fuse.py --in /tmp/pcl_views --out /tmp/fused.ply --step 30
"""
import argparse
import glob
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.pcl import register_sequence, apply_transform, voxel_downsample


def save_ply_ascii_color(path, pts, rgb):
    n = len(pts)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write("element vertex %d\n" % n)
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(n):
            p = pts[i]
            c = rgb[i]
            f.write("%.2f %.2f %.2f %d %d %d\n" % (p[0], p[1], p[2], c[0], c[1], c[2]))


def main():
    parser = argparse.ArgumentParser(description="多视角点云配准融合")
    parser.add_argument("--in", dest="indir", default="/tmp/pcl_views")
    parser.add_argument("--out", default="/tmp/fused.ply")
    parser.add_argument("--step", type=float, default=30.0, help="每视角名义转角(度)")
    parser.add_argument("--voxel", type=float, default=20.0, help="配准下采样体素(mm)")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.indir, "view_*.npz")))
    if len(files) < 2:
        print("FAIL: 需要至少 2 个视角，找到 %d 个 (%s)" % (len(files), args.indir))
        sys.exit(1)

    clouds, rgbs = [], []
    for f in files:
        d = np.load(f)
        clouds.append(d["pts"].astype(np.float32))
        rgbs.append(d["rgb"])
    print("加载 %d 个视角" % len(files))

    init_angles = [args.step] * (len(clouds) - 1)
    poses = register_sequence(clouds, init_angles, voxel_size=args.voxel)
    print("配准完成")

    fused_pts, fused_rgb = [], []
    for i, (R, t) in enumerate(poses):
        fused_pts.append(apply_transform(clouds[i], R, t))
        fused_rgb.append(rgbs[i])
    fused_pts = np.vstack(fused_pts)
    fused_rgb = np.vstack(fused_rgb)
    # 融合后去重（同一体素取一个点，避免重叠区重复点过密）
    keep_idx = np.unique(np.floor(fused_pts / args.voxel).astype(np.int64),
                         axis=0, return_index=True)[1]
    fused_pts = fused_pts[keep_idx]
    fused_rgb = fused_rgb[keep_idx]

    save_ply_ascii_color(args.out, fused_pts, fused_rgb)
    print("融合点云已存 %s (%d 点)" % (args.out, len(fused_pts)))
    print("DONE")


if __name__ == "__main__":
    main()
