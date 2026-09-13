#!/usr/bin/python3
# coding=utf8
"""深度视觉定位：用深度点云 ICP 匹配预建 3D 地图，估计相机位姿。

用于 NO7 阶段一「视觉定位 + 固定路线」：机器人走固定路线时，拍一张深度图，
ICP 匹配到预建地图，得到相机/机器人在世界（地图）坐标系下的位姿，用来校正
IMU 航向漂移。

地图文件(.npz)由 tasks/CS/CS-build-map.py 生成，含 pts(N,3) + normals(N,3)。
"""
import numpy as np

from agcs_lib.pcl import icp_plane, voxel_downsample


class Localizer:
    """加载预建 3D 地图，用深度图 ICP 定位。"""

    def __init__(self, map_path):
        d = np.load(map_path)
        self.map_pts = d['pts'].astype(np.float32)
        self.map_normals = d['normals'].astype(np.float32)

    def localize_points(self, points, init_R=None, init_t=None, voxel_size=20.0):
        """用点云 (N,3) 定位（相机坐标系）。

        返回 (R, t, err)：满足 R@点+t≈地图点，即相机在地图坐标系的位姿；
        失败返回 None。
        """
        if len(points) < 100:
            return None
        pts = voxel_downsample(points, voxel_size)
        if init_R is None:
            init_R = np.eye(3)
        if init_t is None:
            init_t = np.zeros(3)
        R, t, err = icp_plane(pts, self.map_pts, self.map_normals,
                              init_R=init_R, init_t=init_t, dist_thresh=200.0,
                              max_iter=40)
        return R, t, err

    def localize(self, depth, depth_cam, init_R=None, init_t=None, voxel_size=20.0):
        """用一张深度图定位。

        depth: (H, W) uint16 深度图(mm)。depth_cam: DepthCamera 实例。
        init_R/init_t: ICP 初值 = 相机在地图坐标系的位姿猜测（路线推算）。
        返回同 localize_points。
        """
        pcl = depth_cam.depth_to_pointcloud(depth)  # (H, W, 3) 相机坐标系
        valid = ~np.isnan(pcl[:, :, 0])
        pts = pcl[valid].reshape(-1, 3).astype(np.float32)
        return self.localize_points(pts, init_R=init_R, init_t=init_t,
                                    voxel_size=voxel_size)
