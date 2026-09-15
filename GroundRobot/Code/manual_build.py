#!/usr/bin/python3
# coding=utf8
"""手动采集点云建图（在 Mac 上跑）：ICP 配准（身份初值）+ 融合成 map.npz。

不 import agcs_lib 包（避开树莓派专属依赖），用 importlib 直接加载 pcl.py。

用法（先把树莓派的 /tmp/pcl_manual 拉到本地）：
    scp -r pi@<IP>:/tmp/pcl_manual ./pcl_manual
    python3 manual_build.py --in ./pcl_manual --out map.npz
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
    parser = argparse.ArgumentParser(description='手动采集点云建图')
    parser.add_argument('--in', dest='indir', default='./pcl_manual')
    parser.add_argument('--out', default='map.npz')
    parser.add_argument('--voxel', type=float, default=20.0, help='地图体素(mm)')
    args = parser.parse_args()

    pcl = _load_pcl()

    files = sorted(glob.glob(os.path.join(args.indir, 'view_*.npz')))
    if len(files) < 2:
        print('FAIL: 至少 2 帧，找到 %d 帧 (%s)' % (len(files), args.indir))
        sys.exit(1)

    clouds = [np.load(f)['pts'].astype(np.float32) for f in files]
    print('加载 %d 帧' % len(files))

    # 手动采集无里程计，用身份初值 + ICP 逐帧配准（要求相邻帧有重叠）
    poses = pcl.register_sequence(clouds, [0.0] * (len(clouds) - 1), voxel_size=20.0)
    print('配准完成')

    fused = np.vstack([pcl.apply_transform(clouds[i], R, t)
                       for i, (R, t) in enumerate(poses)])
    fused = pcl.voxel_downsample(fused, args.voxel)
    normals = pcl.compute_normals(fused, k=12)

    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=fused, normals=normals)
    print('地图已存 %s (%d 点)' % (args.out, len(fused)))
    print('DONE')


if __name__ == '__main__':
    main()
