#!/usr/bin/python3
# coding=utf8
"""建图：把多视角点云配准融合成一张 3D 地图，存成 .npz 供 Localizer 定位。

用法（先 sudo systemctl stop spiderpi，机器人原地转一圈采集）：
    python3 tasks/CS/CS-multiview-capture.py --step 30 --views 12 --out /tmp/pcl_map
    python3 tasks/CS/CS-build-map.py --in /tmp/pcl_map --out /home/pi/spiderpi/models/map.npz
"""
import argparse
import glob
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.pcl import register_sequence, apply_transform, voxel_downsample, compute_normals


def main():
    parser = argparse.ArgumentParser(description='多视角点云建图')
    parser.add_argument("--in", dest="indir", default="/tmp/pcl_map")
    parser.add_argument("--out", default="/home/pi/spiderpi/models/map.npz")
    parser.add_argument("--step", type=float, default=30.0, help="每视角名义转角(度)")
    parser.add_argument("--voxel", type=float, default=20.0, help="地图体素(mm)")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.indir, "view_*.npz")))
    if len(files) < 2:
        print("FAIL: 至少 2 个视角，找到 %d 个 (%s)" % (len(files), args.indir))
        sys.exit(1)

    clouds = []
    for f in files:
        clouds.append(np.load(f)["pts"].astype(np.float32))

    poses = register_sequence(clouds, [args.step] * (len(clouds) - 1),
                              voxel_size=20.0)

    fused = np.vstack([apply_transform(clouds[i], R, t)
                       for i, (R, t) in enumerate(poses)])
    fused = voxel_downsample(fused, args.voxel)
    normals = compute_normals(fused, k=12)

    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=fused, normals=normals)
    print("地图已存 %s (%d 点)" % (args.out, len(fused)))
    print("DONE")


if __name__ == "__main__":
    main()
