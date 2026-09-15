#!/usr/bin/python3
# coding=utf8
"""深度相机当 2D 激光雷达用：取帧、标定俯仰角、把点云压成机器人系 2D 点。

scan_2d.py（转一点停一下）和 scan_2d_fast.py（一边转一边拍）共用这些。

坐标约定
    相机点云：X 右 / Y 上 / Z 前，mm（depth_to_pointcloud 输出）。
    机器人系 2D：x 右 / y 前，mm。相机系 2D 绕竖直轴转 φ 就到机器人系，
    φ = (500 - pan) * 90 / 400（21 号舵机 500=正前、900=左90°、100=右90°）。
"""
import time

import numpy as np

from agcs_lib.pcl2d import depth_to_2d, icp_2d

PAN_CENTER = 500      # 21 号舵机正前脉宽
PAN_PER_DEG = 400.0 / 90.0  # 每度多少脉宽（90° = 400 脉宽）


def pan_angle_deg(pan):
    """21 号舵机脉宽 → 相机光轴在机器人系里的角度（度，+右 -左）。"""
    return (PAN_CENTER - float(pan)) * 90.0 / 400.0


def read_pts(cam, flush=2):
    """读一帧深度 → (N,3) 相机系点云（X右 Y上 Z前，mm）。读不到返回 None。

    flush: 先丢掉缓冲里的旧帧数（timeout=0 不等待）。连续扫时传 0，一帧都别丢。
    """
    for _ in range(flush):
        cam.read_depth(timeout_ms=0)
    d = cam.read_depth(timeout_ms=2000)
    if d is None:
        return None
    pcl = cam.depth_to_pointcloud(d)
    ok = ~np.isnan(pcl[:, :, 0])
    if not ok.any():
        return None
    return pcl[ok].reshape(-1, 3).astype(np.float32)


def fit_floor(cam, samples=5, z_max=2500.0):
    """拟合地板平面，返回 (pitch_deg, cam_h_mm, inlier_ratio)；失败返回 None。

    pitch 是相机下俯角（度），正好是 depth_to_2d 要的那个值；cam_h 是相机离地
    高度（mm）。迭代重拟合，把花盆/墙的点挤出内点集。
    """
    parts = []
    for _ in range(samples):
        pts = read_pts(cam, flush=1)
        if pts is not None:
            pts = pts[pts[:, 2] < z_max]
            if len(pts):
                parts.append(pts)
        time.sleep(0.15)
    if not parts:
        return None
    pts = np.vstack(parts)
    if len(pts) < 500:
        return None

    for _ in range(3):
        c = pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts - c, full_matrices=False)
        n = Vt[-1]
        if n[1] > 0:  # 法向量统一指向相机下方
            n = -n
        d0 = float(n @ c)
        dist = np.abs(pts @ n - d0)
        keep = dist < max(15.0, 2.0 * float(np.median(dist)))
        if keep.sum() < 500 or keep.all():
            break
        pts = pts[keep]

    c = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - c, full_matrices=False)
    n = Vt[-1]
    if n[1] > 0:
        n = -n
    d0 = float(n @ c)
    ratio = float((np.abs(pts @ n - d0) < 20.0).mean())
    # 地板法向量（世界系 (0,-1,0)）在相机系里 = (0, -cosθ, sinθ)，θ=下俯角。
    # 反推 θ = atan2(n[2], -n[1])。写成 atan2(-n[2], ...) 会差个正负号，把地板
    # 压成斜坡（就是之前建图散架的原因）。
    pitch = float(np.degrees(np.arctan2(n[2], -n[1])))
    return pitch, abs(d0), ratio


def band_2d(pts3d, pitch, cam_h, floor_clear=80.0, obstacle_h=1200.0, z_max=6000.0):
    """点云 → 机器人系角度为 0 的 2D 点（相机系 x右 y前），按高度带滤掉地板/顶棚。

    pitch/cam_h 来自 fit_floor：地板在 -cam_h，高度带取 [-cam_h+floor_clear,
    -cam_h+obstacle_h]，只要中间这段竖直障碍（墙、花盆）。
    """
    pts2d = depth_to_2d(pts3d, pitch, -cam_h + floor_clear, -cam_h + obstacle_h)
    if len(pts2d) == 0:
        return pts2d
    r = np.hypot(pts2d[:, 0], pts2d[:, 1])
    return pts2d[r <= z_max]


def to_robot(pts2d, ang_deg, cam_offset=(0.0, 0.0)):
    """相机系 2D 点 → 机器人系 2D 点（x 右 / y 前）。ang_deg 是光轴的角度。

    cam_offset: 相机相对 21 号舵机转轴的水平偏移 (ox, oy) mm（pan=500 时的机器人系
        坐标）。相机装在机械臂末端、不在转轴上，转 21 时相机自己也绕圈走，不补这个
        偏移的话 ±90° 那两段的图会相对中间整体平移最多 offset 那么大（墙会被拉成
        双线）。默认 0=忽略，要精确就标一下再传。
    """
    a = np.radians(ang_deg)
    c, s = np.cos(a), np.sin(a)
    x = c * pts2d[:, 0] + s * pts2d[:, 1]
    y = -s * pts2d[:, 0] + c * pts2d[:, 1]
    ox, oy = cam_offset
    if ox or oy:  # 相机位置 C(φ) = R(φ)@offset，点要加上它才回到机器人系
        x = x + (c * ox + s * oy)
        y = y + (-s * ox + c * oy)
    return np.stack([x, y], axis=1)


def chain_angles(frames, pan_from, pan_to, n_links=10, dist_thresh=100.0, max_iter=40):
    """连续扫时靠帧间 2D ICP 串出每帧的 pan 角（度，机器人系）。

    舵机没有位置反馈，又不能用「按时间线性插值」猜角度（伺服加减速一偏整张图就
    歪）。相邻帧视场重叠一大半，用 2D ICP 求相对转角串起来，最后按「从 pan_from
    一共转到 pan_to」做一次整体缩放，消掉累积漂移。

    n_links: 把整段扫分成多少段来串（每段约 180/n_links 度）。
        **别调大间隔**：实测每段 18° 时角度误差 <1°，每段 36° 时重叠只剩 24°，
        ICP 初值拉不回来会直接崩（误差几十度）。帧少时自动用更小的间隔。

    frames: [(N_i,2) 相机系 2D 点]。返回 (angles, info)。
    """
    n = len(frames)
    base = pan_angle_deg(pan_from)
    if n < 2:
        return [base] * n, {'frames': n, 'note': '帧太少，角度按起始值算', 'scale': 1.0}

    stride = max(1, n // max(1, n_links))
    idx = list(range(0, n, stride))
    if idx[-1] != n - 1:
        idx.append(n - 1)

    step = [0.0]          # 每个采样点的转角（度），长度 = len(idx)
    prev = 0.0
    fails = 0
    for k in range(1, len(idx)):
        gap = idx[k] - idx[k - 1]
        prev_gap = (idx[k - 1] - idx[k - 2]) if k > 1 else gap
        # 初值：上一段转角按帧间隔比例放大（伺服加减速时也够准）
        init = -prev * gap / max(1, prev_gap)
        # icp_2d 用标准数学转角（逆时针为正），本文件的角度是「从 +y 转向 +x」，
        # 差一个负号：icp 返回 θ 满足 Rot(θ)@src≈dst，对应 Δφ = -θ。
        res = icp_2d(frames[idx[k]], frames[idx[k - 1]], init_theta=np.radians(init),
                     dist_thresh=dist_thresh, max_iter=max_iter)
        d = -float(np.degrees(res[0])) if res is not None else init
        if not np.isfinite(d) or abs(d) > 45.0:   # 离谱值当失败，退回初值
            d = init
            fails += 1
        step.append(d)
        prev = d

    total = float(np.sum(step))
    target = pan_angle_deg(pan_to) - base
    scale = target / total if abs(total) > 1e-6 else 1.0
    # 采样点角度 → 逐帧角度（采样点之间的帧线性插值）
    sampled = np.cumsum(step) * scale + base
    angles = list(np.interp(np.arange(n), idx, sampled))
    info = {'frames': n, 'links': len(idx) - 1, 'stride': stride,
            'icp_total': total, 'target': target, 'scale': scale, 'fails': fails,
            'd_min': float(np.min(step[1:])), 'd_max': float(np.max(step[1:])),
            'd_mean': float(np.mean(step[1:]))}
    return angles, info
