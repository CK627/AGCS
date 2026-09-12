#!/usr/bin/python3
# coding=utf8
"""点云配准工具（numpy + scipy，无重依赖）。

用于 S3 静态 3D 建模：机器人原地转身拍多视角点云，用 ICP 把相邻视角
配准到同一坐标系，拼成静态地图。

坐标约定沿用 depth.py 的点云：X 右、Y 上、Z 前（深度）。机器人原地
转身 = 绕竖直轴（本坐标系里的 Y 轴）旋转。
"""
import numpy as np
from scipy.spatial import cKDTree


def apply_transform(points, R, t):
    """把 (N,3) 点云按 R(3,3) 旋转 + t(3,) 平移。"""
    return points @ R.T + t


def voxel_downsample(points, voxel_size=20.0):
    """体素下采样，返回每个体素取一个代表点，加速 ICP。voxel_size 单位 mm。"""
    if len(points) == 0:
        return points
    vox = np.floor(points / voxel_size).astype(np.int64)
    _, idx = np.unique(vox, axis=0, return_index=True)
    return points[idx]


def best_fit_transform(src, dst):
    """Umeyama/SVD：求 R、t 最小化 ||R@src + t - dst||²。"""
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)
    H = (src - src_c).T @ (dst - dst_c)  # 3x3 协方差
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  # 反射修正
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = dst_c - R @ src_c
    return R, t


def icp(source, target, init_R=None, init_t=None, max_iter=50, tol=1e-6,
        dist_thresh=100.0):
    """点到点 ICP。source/target 为 (N,3)/(M,3) 点云。

    返回 (R, t, err)：使 R@source+t 对齐到 target。dist_thresh 单位 mm，
    用于剔除离群对应点。
    """
    if init_R is None:
        init_R = np.eye(3)
    if init_t is None:
        init_t = np.zeros(3)
    R, t = init_R.copy(), init_t.copy()
    src = apply_transform(source, R, t)
    tree = cKDTree(target)
    prev_err = np.inf
    err = np.inf
    for _ in range(max_iter):
        dist, idx = tree.query(src, k=1)
        mask = dist < dist_thresh
        if mask.sum() < 3:
            break
        R_new, t_new = best_fit_transform(src[mask], target[idx[mask]])
        R = R_new @ R
        t = R_new @ t + t_new
        src = apply_transform(source, R, t)
        err = float(dist[mask].mean())
        if abs(prev_err - err) < tol:
            break
        prev_err = err
    return R, t, err


def compute_normals(points, k=12):
    """估计每个点的法向量（局部协方差最小特征值方向），用于点到平面 ICP。"""
    tree = cKDTree(points)
    normals = np.zeros_like(points)
    for i in range(len(points)):
        _, idx = tree.query(points[i], k=min(k, len(points)))
        nb = points[idx]
        cov = np.cov(nb.T)
        w, v = np.linalg.eigh(cov)
        normals[i] = v[:, 0]
    return normals


def icp_plane(source, target, target_normals, init_R=None, init_t=None,
              max_iter=40, dist_thresh=150.0):
    """点到平面 ICP（比点到点对平地/大面场景的平移约束更好）。

    返回 (R, t, err)：使 R@source+t 对齐 target。需要 target 的法向量。
    """
    R = np.eye(3) if init_R is None else init_R.copy()
    t = np.zeros(3) if init_t is None else init_t.copy()
    tree = cKDTree(target)
    prev_err = np.inf
    err = np.inf
    for _ in range(max_iter):
        src = apply_transform(source, R, t)
        dist, idx = tree.query(src)
        mask = dist < dist_thresh
        if mask.sum() < 3:
            break
        s = src[mask]
        d = target[idx[mask]]
        n = target_normals[idx[mask]]
        # 线性化增量 (ω 小旋转, t_inc 平移)：(ω×s + t_inc)·n = (d - s)·n
        A = np.hstack([np.cross(s, n), n])          # (M, 6)
        b = np.sum((d - s) * n, axis=1)             # (M,)
        x, *_ = np.linalg.lstsq(A, b, rcond=None)   # (6,) = [ω, t_inc]
        w = x[:3]
        t_inc = x[3:]
        R_inc = np.array([[1, -w[2], w[1]],
                          [w[2], 1, -w[0]],
                          [-w[1], w[0], 1]])
        R = R_inc @ R
        t = R_inc @ t + t_inc
        err = float(np.abs(np.sum((s - d) * n, axis=1)).mean())
        if abs(prev_err - err) < 1e-6:
            break
        prev_err = err
    return R, t, err


def rot_y(deg):
    """绕 Y 轴（竖直轴）旋转 deg 度，返回 3x3 旋转矩阵。"""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def register_sequence(clouds, init_angles_deg, voxel_size=20.0):
    """按顺序配准一串点云（原地转身场景），用点到平面 ICP。

    clouds: list[(N,3)]，init_angles_deg: list[float] 每个云相对前一云的名义转角。
    返回 list[(R, t)]，把第 i 个云变换到第 0 个云的坐标系（累加）。
    """
    downs = [voxel_downsample(c, voxel_size) for c in clouds]
    normals = [compute_normals(d) for d in downs]
    poses = [(np.eye(3), np.zeros(3))]  # 第 0 个云不动
    for i in range(1, len(downs)):
        R0 = rot_y(init_angles_deg[i - 1])  # 名义转角作初值
        R, t, _ = icp_plane(downs[i], downs[i - 1], normals[i - 1],
                            init_R=R0, init_t=np.zeros(3))
        R_prev, t_prev = poses[-1]
        poses.append((R_prev @ R, R_prev @ t + t_prev))
    return poses
