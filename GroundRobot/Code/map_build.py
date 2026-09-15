#!/usr/bin/python3
# coding=utf8
"""本地建图（在 Mac 上跑）：读 view_*.npz（pts + 里程计 R/t），ICP 精修，融合成 map.npz。

不 import agcs_lib 包（避免触发树莓派专属的 common 依赖），用 importlib 直接
加载 agcs_lib/pcl.py（pcl.py 只依赖 numpy + scipy，本地可跑）。

用法（先把树莓派采集的 /tmp/pcl_full 拉到本地）：
    scp -r pi@<IP>:/tmp/pcl_full ./pcl_full
    python3 map_build.py --in ./pcl_full --out map.npz
    scp map.npz pi@<IP>:/home/pi/spiderpi/models/map.npz
"""
import argparse
import glob
import importlib.util
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PCL_PATH = os.path.join(_HERE, 'agcs_lib', 'pcl.py')


def _load_pcl():
    spec = importlib.util.spec_from_file_location('pcl_standalone', _PCL_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    parser = argparse.ArgumentParser(description='本地建图')
    parser.add_argument("--in", dest="indir", default="./pcl_full")
    parser.add_argument("--out", default="map.npz")
    parser.add_argument("--voxel", type=float, default=20.0, help="地图体素(mm)")
    args = parser.parse_args()

    pcl = _load_pcl()

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

    poses = pcl.register_route(clouds, odom, voxel_size=20.0)
    print("配准完成")

    fused = np.vstack([pcl.apply_transform(clouds[i], R, t)
                       for i, (R, t) in enumerate(poses)])
    fused = pcl.voxel_downsample(fused, args.voxel)
    normals = pcl.compute_normals(fused, k=12)

    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=fused, normals=normals)
    print("地图已存 %s (%d 点)" % (args.out, len(fused)))
    print("DONE")


if __name__ == '__main__':
    main()
