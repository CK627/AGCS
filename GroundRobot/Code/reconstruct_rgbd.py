#!/usr/bin/python3
# coding=utf8
"""Open3D RGB-D 三维重建：RGB-D 里程计算位姿 + TSDF 体素融合 → 点云。

读 record_rgbd.py 录制的 depth_XXXXX.png + color_XXXXX.jpg + intrinsic.json。

用法（用 venv 的 python，先把树莓派 /tmp/rgbd 拉到本地）：
    scp -r pi@<IP>:/tmp/rgbd ./rgbd
    ~/open3d_env/bin/python reconstruct_rgbd.py --in ./rgbd --out map.ply
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description='Open3D RGB-D 重建')
    parser.add_argument('--in', dest='indir', default='./rgbd')
    parser.add_argument('--out', default='map.ply')
    parser.add_argument('--voxel', type=float, default=0.02, help='TSDF 体素边长(m)')
    parser.add_argument('--depth-trunc', type=float, default=5.0, help='最大深度(m)')
    args = parser.parse_args()

    with open(os.path.join(args.indir, 'intrinsic.json')) as f:
        intr = json.load(f)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intr['width'], intr['height'], intr['fx'], intr['fy'], intr['cx'], intr['cy'])

    depth_files = sorted(glob.glob(os.path.join(args.indir, 'depth_*.png')))
    color_files = sorted(glob.glob(os.path.join(args.indir, 'color_*.jpg')))
    n = min(len(depth_files), len(color_files))
    print('帧数: %d' % n)
    if n < 2:
        print('FAIL: 帧数不足')
        sys.exit(1)

    def make_rgbd(depth_file, color_file):
        color = o3d.io.read_image(color_file)
        depth = o3d.io.read_image(depth_file)
        return o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=1000.0, depth_trunc=args.depth_trunc,
            convert_rgb_to_intensity=False)

    option = o3d.pipelines.odometry.OdometryOption()
    option.depth_diff_max = 0.03
    option.depth_min = 0.3
    option.depth_max = args.depth_trunc

    # 逐帧 RGB-D 里程计，累积位姿
    poses = [np.eye(4)]
    rgbd_prev = make_rgbd(depth_files[0], color_files[0])
    for i in range(1, n):
        rgbd = make_rgbd(depth_files[i], color_files[i])
        ok, trans, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
            rgbd_prev, rgbd, intrinsic, np.eye(4),
            o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
        if ok:
            poses.append(poses[-1] @ trans)
        else:
            poses.append(poses[-1])  # 失败沿用上一帧位姿
        rgbd_prev = rgbd
        if i % 10 == 0:
            print('odometry %d/%d' % (i, n), flush=True)
    print('里程计完成')

    # TSDF 融合
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel, sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    for i in range(n):
        rgbd = make_rgbd(depth_files[i], color_files[i])
        volume.integrate(rgbd, intrinsic, np.linalg.inv(poses[i]))
        if i % 10 == 0:
            print('integrate %d/%d' % (i, n), flush=True)
    print('融合完成')

    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.normals = mesh.vertex_normals
    o3d.io.write_point_cloud(args.out, pcd)
    print('已存 %s (%d 点)' % (args.out, len(pcd.points)))
    print('DONE')


if __name__ == '__main__':
    main()
