#!/usr/bin/python3
# coding=utf8
"""2D 定位：把深度点云压成 2D（墙/障碍的水平轮廓），用 2D ICP 匹配。

室内平地导航用：只关心墙/障碍在水平面上的位置，忽略高度。相比 3D SLAM，
数据量小、匹配快、没有顶棚/护栏那种干扰。

坐标约定：
    相机点云（depth_to_pointcloud 输出）：X 右 / Y 上 / Z 前，单位 mm。
    世界 2D（地板对齐后）：x 右 / y 前，单位 mm。
"""
import numpy as np
from scipy.spatial import cKDTree


def depth_to_2d(points, pitch_deg, min_height_mm=150.0, max_height_mm=1500.0):
    """相机 3D 点云 -> 世界 2D 点（墙/障碍高度带）。

    points: (N,3) 相机坐标系（X右 Y上 Z前，mm）。
    pitch_deg: 相机绕 X 轴的下俯角（度），用于把相机前向转到水平（地板对齐）。
              相机低头看地面，pitch 为正；标定不对时试正负号。
    min_height_mm / max_height_mm: 世界高度带（mm，相对相机）。低于 min 视为地板
              （滤掉），高于 max 视为顶棚/护栏（滤掉），只留中间这段墙/障碍。
    返回 (M,2) 世界 2D 点（x 右, y 前），mm。
    """
    pitch = np.radians(pitch_deg)
    R = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)],
    ])
    w = points @ R.T
    h = w[:, 1]  # 世界高度(上)
    mask = (h >= min_height_mm) & (h <= max_height_mm)
    return w[mask][:, [0, 2]]


def voxel_2d(points, voxel_mm=50.0):
    """2D 体素下采样，voxel_mm 单位 mm。"""
    points = np.asarray(points, dtype=np.float32)
    if len(points) == 0:
        return points
    v = np.floor(points / voxel_mm).astype(np.int64)
    _, idx = np.unique(v, axis=0, return_index=True)
    return points[idx]


def _rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def icp_2d(src, dst, init_theta=0.0, init_t=None, max_iter=40, dist_thresh=300.0):
    """2D 点到点 ICP：返回 (theta, t(2,), err) 使 R(theta)@src + t 对齐 dst。

    src: (N,2) 当前扫描（相机/机器人坐标系），dst: (M,2) 地图点（世界坐标系）。
    init_theta / init_t: 位姿初值（IMU 航向 + 上次位姿），给得越准收敛越快。
    dist_thresh: 对应点距离阈值(mm)，剔除离群。
    点数不足或收敛失败返回 None。
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if len(src) < 10 or len(dst) < 10:
        return None
    theta = float(init_theta)
    t = np.asarray(init_t, dtype=np.float64) if init_t is not None else np.zeros(2)
    tree = cKDTree(dst)
    prev_err = np.inf
    for _ in range(max_iter):
        s = (_rot2(theta) @ src.T).T + t
        dist, idx = tree.query(s, k=1)
        mask = dist < dist_thresh
        if mask.sum() < 5:
            break
        # 2D Procrustes：求 (dtheta, dt) 使 s[mask] 对齐 dst[idx[mask]]
        sc = s[mask].mean(axis=0)
        dc = dst[idx[mask]].mean(axis=0)
        H = (s[mask] - sc).T @ (dst[idx[mask]] - dc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        dt = dc - _rot2(dtheta) @ sc
        # 增量更新（相对当前估计）
        theta += dtheta
        t = _rot2(dtheta) @ t + dt
        err = float(dist[mask].mean())
        if abs(prev_err - err) < 1e-4:
            break
        prev_err = err
    return theta, t, err


def build_grid(points, resolution_mm=50.0, size_mm=5000):
    """把 2D 点集画进占据栅格，返回 (grid, origin_mm)。

    grid: (H,W) uint8，1=占据；origin_mm: 左下角世界坐标 (x, y)。
    """
    points = np.asarray(points, dtype=np.float32)
    n = int(size_mm / resolution_mm)
    origin = np.array([-size_mm / 2.0, -size_mm / 2.0], dtype=np.float32)
    grid = np.zeros((n, n), dtype=np.uint8)
    if len(points) == 0:
        return grid, origin
    ij = np.floor((points - origin) / resolution_mm).astype(np.int64)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < n) & (ij[:, 1] >= 0) & (ij[:, 1] < n)
    grid[ij[ok, 1], ij[ok, 0]] = 1
    return grid, origin
