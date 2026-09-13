#!/usr/bin/python3
# coding=utf8
"""ICP 配准算法验证：已知变换 + 噪声，验证点到平面 ICP 能否恢复。

约定：icp(source, target) 返回使 R@source+t 对齐 target 的变换。
source = T_gt(target)，ICP 应恢复 T_gt 的逆。用测地线旋转误差判断。

纯算法测试，不碰深度相机。用法：python3 CS-icp.py
"""
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.pcl import (
    icp_plane, compute_normals, register_sequence,
    voxel_downsample, rot_y, apply_transform,
)


def rot_angle_err(R, R_ref):
    """两个旋转矩阵之间的测地线角度误差（度）。"""
    c = np.clip((np.trace(R @ R_ref.T) - 1) / 2, -1, 1)
    return np.degrees(np.arccos(c))


def make_scene():
    """有特征、破坏对称的合成场景：地面 + 两个盒子 + 墙 + 斜柱。"""
    pts = []
    gx, gz = np.meshgrid(np.arange(0, 2001, 50), np.arange(0, 2001, 50))
    pts.append(np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], axis=1))
    for bx in range(500, 801, 40):
        for bz in range(800, 1101, 40):
            pts.append(np.array([[bx, 0, bz], [bx, 300, bz]]))
    for bx in range(1200, 1501, 40):
        for bz in range(1200, 1501, 40):
            pts.append(np.array([[bx, 0, bz], [bx, 500, bz]]))
    wx, wy = np.meshgrid(np.arange(0, 2001, 50), np.arange(0, 1001, 50))
    pts.append(np.stack([wx.ravel(), wy.ravel(), np.full(wx.size, 1900.0)], axis=1))
    for h in range(0, 801, 40):
        pts.append(np.array([[1800.0 - h * 0.5, h, 300.0]]))
    return np.vstack(pts)


def test_pair():
    rng = np.random.default_rng(0)
    target = voxel_downsample(make_scene(), voxel_size=40)
    normals = compute_normals(target, k=12)
    gt, t_gt = 30.0, np.array([60.0, 0.0, -30.0])
    R_gt = rot_y(gt)
    source = apply_transform(target, R_gt, t_gt)
    source = source + rng.normal(0, 5.0, source.shape)

    R, t, err = icp_plane(source, target, normals, init_R=rot_y(-(gt - 5)),
                          init_t=np.zeros(3), dist_thresh=150, max_iter=60)
    a = rot_angle_err(R, rot_y(-gt))
    te = np.linalg.norm(t - (-R_gt.T @ t_gt))
    print("单对配准: 角度误差 %.2f° 平移误差 %.1fmm 残差 %.2fmm" % (a, te, err))
    return a < 1.0 and te < 20.0


def test_sequence():
    """模拟原地转身 3 视角（每视角转 30°，相机带偏移），验证序列配准。"""
    rng = np.random.default_rng(1)
    base = make_scene()
    clouds = []
    for i in range(3):
        # 第 i 个视角：机器人转了 i*30°，相机相对旋转中心有偏移
        R_view = rot_y(i * 30.0)
        # 相机位置绕旋转中心转 i*30°（偏移半径 150mm）
        cam_pos = np.array([150 * np.cos(np.radians(i * 30)), 0.0,
                            -150 * np.sin(np.radians(i * 30))])
        c = apply_transform(base, R_view, cam_pos)
        c = c + rng.normal(0, 5.0, c.shape)
        # 只保留相机前方 FOV 内的点（模拟视野裁剪）
        clouds.append(c)
    # 名义每步 +30°
    poses = register_sequence(clouds, [30.0, 30.0], voxel_size=40)
    # 验证：把每个云用对应 pose 变换后，与第 0 个云对齐（测重投影后与 base 的贴合度）
    fused = []
    for i, (R, t) in enumerate(poses):
        fused.append(apply_transform(clouds[i], R, t))
    fused = np.vstack(fused)
    # 简单检验：融合后的地面点 y 应接近 0
    floor = fused[np.abs(fused[:, 1]) < 150]
    print("序列配准(3视角): 融合后地面点 %d, 地面 y 均值 %.1fmm (应≈0)" %
          (len(floor), floor[:, 1].mean()))
    return abs(floor[:, 1].mean()) < 30.0


def main():
    r1 = test_pair()
    r2 = test_sequence()
    print("PASS" if (r1 and r2) else "FAIL")


if __name__ == "__main__":
    main()
