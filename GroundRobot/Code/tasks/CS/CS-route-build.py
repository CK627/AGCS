#!/usr/bin/python3
# coding=utf8
"""沿路线点云建图：读 view_*.npz（pts + 里程计 R/t），ICP 精修，融合成 map.npz。

与 CS-multiview-capture 的「原地转身」不同，这里里程计含平移（前进/后退）+
旋转（转弯），用 register_route 做点到平面 ICP 精修。

用法：
    python3 CS-route-build.py --in /tmp/pcl_route --out /home/pi/spiderpi/models/map.npz
"""
import argparse
import glob
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.pcl import register_route, apply_transform, voxel_downsample, compute_normals


def main():
    parser = argparse.ArgumentParser(description='沿路线点云建图')
    parser.add_argument("--in", dest="indir", default="/tmp/pcl_route")
    parser.add_argument("--out", default="/home/pi/spiderpi/models/map.npz")
    parser.add_argument("--voxel", type=float, default=20.0, help="地图体素(mm)")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.indir, "view_*.npz")))
    if len(files) < 2:
        print("FAIL: 至少 2 帧，找到 %d 帧 (%s)" % (len(files), args.indir))
        sys.exit(1)

    clouds, odom = [], []
    for f in files:
        d = np.load(f)
        clouds.append(d["pts"].astype(np.float32))
        odom.append((d["R"].astype(np.float32), d["t"].astype(np.float32)))
    print("加载 %d 帧" % len(files))

    poses = register_route(clouds, odom, voxel_size=20.0)

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


if __name__ == '__main__':
    main()
