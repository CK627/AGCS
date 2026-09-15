#!/usr/bin/python3
# coding=utf8
"""奥比中光 Astra Pro Plus 深度摄像头抓取算法（独立演示文件）。

本文件不接入具体相机 SDK，也不依赖本项目其它模块，只展示算法逻辑。
使用前提：你已经拿到下面两张对齐后的 numpy 图像：

    color_img  : HxWx3 的 BGR 彩色图
    depth_img  : HxW   的深度图，每个像素是深度值

奥比中光 SDK 里通常需要：

    - 打开彩色流和深度流
    - 使用 SDK 的“深度对齐彩色”接口，得到 aligned_depth_frame
    - 把深度帧转成 numpy 数组

例如伪代码：

    color_frame = pipeline.get_color_frame()
    depth_frame = pipeline.get_depth_frame()
    aligned = align.process(color_frame, depth_frame)
    color_img = color_frame.as_numpy()
    depth_img = aligned.as_numpy()      # 单位按 SDK，常见为毫米

本文件默认深度单位是“毫米”，如果你用米或厘米，请统一改 DEPTH_UNIT。

核心流程：

    1. 在彩色图上检测目标，得到目标框或掩膜 mask
    2. 在目标 mask 内找“最近点”，也就是最高/最突出的抓取点
    3. 把该像素的深度转成相机 3D 坐标
    4. 把相机 3D 坐标转成机械臂 3D 坐标
    5. 可选：用深度点云估计表面法向量，让夹爪尽量垂直于曲面
"""

import math

import cv2
import numpy as np


# 深度单位：1 表示毫米。如果 SDK 输出是米，改成 1000；输出是厘米，改成 10。
DEPTH_UNIT = 1.0


# ---------------------------------------------------------------------------
# 1. 深度像素 -> 相机 3D 坐标
# ---------------------------------------------------------------------------
def depth_pixel_to_camera_xyz(u, v, depth, K):
    """把一个深度像素点转换成相机坐标系下的 3D 点。

    参数
    ----
    u, v   : 像素坐标，浮点数，可以是亚像素
    depth  : 该像素的深度值，单位和 DEPTH_UNIT 一致
    K      : 相机内参矩阵 3x3，取自相机标定或 SDK 提供的参数

    返回
    ----
    (X, Y, Z)：相机坐标系 3D 点，单位和 depth 一致。

    原理：小孔成像模型。
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth
    """
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)

    # 内参：焦距 fx/fy，光心 cx/cy
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    Z = float(depth)
    X = (float(u) - cx) * Z / fx
    Y = (float(v) - cy) * Z / fy

    return X, Y, Z


# ---------------------------------------------------------------------------
# 2. 相机 3D 坐标 -> 机械臂 3D 坐标
# ---------------------------------------------------------------------------
def camera_xyz_to_arm_xyz(cam_xyz, R_cam2arm, t_cam2arm):
    """把相机坐标系下的 3D 点变换到机械臂坐标系。

    参数
    ----
    cam_xyz    : (X, Y, Z) 相机坐标
    R_cam2arm  : 3x3 旋转矩阵，相机坐标系 -> 机械臂坐标系
    t_cam2arm  : 3x1 平移向量，相机坐标系 -> 机械臂坐标系

    返回
    ----
    (x, y, z)：机械臂坐标。

    注意：
    这个 R/t 是“手眼标定”结果，和以前相机到地面的标定不是同一套。
    需要重新做一次相机与机械臂之间的外参标定。
    """
    cam = np.asarray(cam_xyz, dtype=np.float64).reshape(3, 1)
    R = np.asarray(R_cam2arm, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t_cam2arm, dtype=np.float64).reshape(3, 1)

    arm = R @ cam + t
    return float(arm[0, 0]), float(arm[1, 0]), float(arm[2, 0])


# ---------------------------------------------------------------------------
# 3. 由检测框生成目标掩膜
# ---------------------------------------------------------------------------
def mask_from_bbox(h, w, bbox, margin=0):
    """把目标检测框转换成一个布尔掩膜。

    bbox 格式：(x1, y1, x2, y2)，像素坐标，可以是 int 或 float。
    返回 (H, W) 的 uint8 掩膜，目标内部为 1，外部为 0。
    """
    x1, y1, x2, y2 = bbox

    # 加 margin，避免只取到目标边缘，深度边缘通常噪声很大
    x1 = max(0, int(x1) - margin)
    y1 = max(0, int(y1) - margin)
    x2 = min(w, int(x2) + margin)
    y2 = min(h, int(y2) + margin)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y1:y2, x1:x2] = 1
    return mask


# ---------------------------------------------------------------------------
# 4. 深度有效性过滤
# ---------------------------------------------------------------------------
def valid_depth_mask(depth_img, min_depth=200.0, max_depth=5000.0):
    """生成“深度可信”的掩膜。

    - 深度为 0 / NaN / Inf 的像素不可信。
    - 小于 min_depth：太近，可能是物体贴住镜头或噪声。
    - 大于 max_depth：太远，可能是没有深度数据或背景。

    单位与 depth_img 一致，默认按毫米设置。
    """
    mask = np.isfinite(depth_img) & (depth_img > 0)
    mask &= depth_img >= min_depth
    mask &= depth_img <= max_depth
    return mask


# ---------------------------------------------------------------------------
# 5. 在目标区域内找“最高/最突出”的抓取点
# ---------------------------------------------------------------------------
def pick_point_from_depth(depth_img, mask, K, top_ratio=0.05):
    """在目标 mask 内找最适合夹取的点，返回相机 3D 坐标。

    原理：
    - 目标区域里“深度最小”的点，通常是离相机最近的点。
    - 对地面目标来说，这就是目标最高点；
    - 对曲面目标来说，这近似曲面的最高/最突出位置。

    为了避免单点噪声，取深度最小的一小部分像素做中位数。

    参数
    ----
    depth_img : 对齐后的深度图
    mask      : 目标掩膜，0/1
    K         : 相机内参 3x3
    top_ratio : 取最近多少比例的像素，默认 5%

    返回
    ----
    (X, Y, Z) 相机坐标；找不到可信点时返回 None。
    """
    depth_img = np.asarray(depth_img, dtype=np.float64)
    mask = np.asarray(mask, dtype=np.uint8)

    # 目标区域 + 深度可信，两个条件同时满足
    valid = valid_depth_mask(depth_img) & (mask > 0)
    if not valid.any():
        return None

    depths = depth_img[valid]

    # 取深度最小的一小部分像素，作为“最高点候选区”
    # 例如 top_ratio=0.05，就是取最近 5% 的像素
    threshold = np.percentile(depths, top_ratio * 100.0)
    top_mask = valid & (depth_img <= threshold)

    # 对这些候选像素取中位数坐标，避免单个坏点
    ys, xs = np.where(top_mask)
    u = float(np.median(xs))
    v = float(np.median(ys))

    # 用候选点的中位深度，而不是单点深度
    depth = float(np.median(depth_img[top_mask]))

    # 转成相机 3D 坐标
    return depth_pixel_to_camera_xyz(u, v, depth, K)


# ---------------------------------------------------------------------------
# 6. 估计抓取点附近的表面法向量
# ---------------------------------------------------------------------------
def surface_normal_at_point(depth_img, u, v, K, window=5):
    """用抓取点周围的小邻域拟合平面，估计表面法向量。

    用途：
    曲面物体不是平面，夹爪如果始终竖直夹，可能只夹到边缘。
    算出法向量后，可以让夹爪尽量垂直于物体表面，抓得更稳。

    方法：
    取 (u, v) 附近 window x window 邻域的有效深度点，
    转成 3D 点云，用 PCA/SVD 拟合平面，最小奇异值对应的方向就是法向量。

    返回
    ----
    单位法向量 (nx, ny, nz)，并调整为指向相机方向；无法估计时返回 None。
    """
    depth_img = np.asarray(depth_img, dtype=np.float64)
    h, w = depth_img.shape
    r = int(window // 2)

    points = []
    for y in range(int(v) - r, int(v) + r + 1):
        for x in range(int(u) - r, int(u) + r + 1):
            if x < 0 or y < 0 or x >= w or y >= h:
                continue
            d = depth_img[y, x]
            if not (math.isfinite(d) and d > 0):
                continue
            points.append(depth_pixel_to_camera_xyz(x, y, d, K))

    if len(points) < 3:
        return None

    pts = np.asarray(points, dtype=np.float64)

    # 去中心化
    centroid = pts.mean(axis=0)
    pts_centered = pts - centroid

    # SVD 拟合平面：最小奇异值对应的右奇异向量就是法向量
    _, _, vh = np.linalg.svd(pts_centered, full_matrices=False)
    normal = vh[-1]

    # 统一法向量方向：让法向量指向相机。
    # 相机坐标系里，可见表面大致朝向相机，通常 normal 的 Z 分量应为负。
    if normal[2] > 0:
        normal = -normal

    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        return None

    normal = normal / norm
    return float(normal[0]), float(normal[1]), float(normal[2])


# ---------------------------------------------------------------------------
# 7. 汇总：深度图 + 目标框 -> 机械臂抓取点
# ---------------------------------------------------------------------------
def compute_grasp_from_depth(
    depth_img,
    bbox,
    K,
    R_cam2arm,
    t_cam2arm,
    top_ratio=0.05,
    margin=3,
    estimate_normal=True,
):
    """从深度图和目标框直接计算机械臂抓取坐标。

    参数
    ----
    depth_img : 对齐后的深度图
    bbox      : 目标框 (x1, y1, x2, y2)
    K         : 相机内参 3x3
    R_cam2arm : 相机->机械臂旋转矩阵
    t_cam2arm : 相机->机械臂平移向量
    estimate_normal : 是否同时估计表面法向量

    返回
    ----
    dict：
        {
            'camera_xyz': (X, Y, Z),
            'arm_xyz':    (x, y, z),
            'normal':     (nx, ny, nz) 或 None,
        }
    找不到有效抓取点时返回 None。
    """
    h, w = depth_img.shape[:2]
    mask = mask_from_bbox(h, w, bbox, margin=margin)

    cam_xyz = pick_point_from_depth(depth_img, mask, K, top_ratio=top_ratio)
    if cam_xyz is None:
        return None

    arm_xyz = camera_xyz_to_arm_xyz(cam_xyz, R_cam2arm, t_cam2arm)

    normal = None
    if estimate_normal:
        # 先用候选区中心像素估计表面法向量
        # 这里需要一个像素坐标，pick_point_from_depth 内部已算过，
        # 为保持接口简单，下面重新用同样逻辑取一次像素中心。
        valid = valid_depth_mask(depth_img) & (mask > 0)
        depths = depth_img[valid]
        threshold = np.percentile(depths, top_ratio * 100.0)
        top_mask = valid & (depth_img <= threshold)
        ys, xs = np.where(top_mask)
        u = float(np.median(xs))
        v = float(np.median(ys))
        normal = surface_normal_at_point(depth_img, u, v, K)

    return {
        'camera_xyz': cam_xyz,
        'arm_xyz': arm_xyz,
        'normal': normal,
    }


# ---------------------------------------------------------------------------
# 演示：不接相机，用一张合成深度图展示算法流程
# ---------------------------------------------------------------------------
def _demo():
    """生成一个曲面凸起目标，演示如何从深度图计算抓取点。"""
    h, w = 480, 640

    # 合成深度图：地面深度约 2000mm，中心有一个曲面凸起，最近点约 1200mm
    yy, xx = np.mgrid[0:h, 0:w]
    cx, cy = 320.0, 240.0
    r2 = (xx - cx) ** 2 / (90.0 ** 2) + (yy - cy) ** 2 / (90.0 ** 2)
    depth = 2000.0 - 800.0 * np.exp(-r2)
    depth = depth.astype(np.float64)

    # 模拟一个目标框，框住曲面凸起区域
    bbox = (220, 140, 420, 340)

    # 相机内参：与项目标定文件类似
    K = np.array([
        [424.6064659125572, 0.0, 302.62477665348985],
        [0.0, 425.6259206559359, 276.32211219330236],
        [0.0, 0.0, 1.0],
    ])

    # 相机 -> 机械臂外参，这里是演示占位值，实际必须手眼标定
    R_cam2arm = np.eye(3)
    t_cam2arm = np.array([[0.0], [0.0], [0.0]])

    result = compute_grasp_from_depth(
        depth, bbox, K, R_cam2arm, t_cam2arm,
        top_ratio=0.05,
        margin=3,
        estimate_normal=True,
    )

    print('目标框:', bbox)
    if result is None:
        print('未找到有效抓取点')
        return

    cam = result['camera_xyz']
    arm = result['arm_xyz']
    normal = result['normal']

    print('相机坐标(mm): X=%.2f Y=%.2f Z=%.2f' % cam)
    print('机械臂坐标(mm): x=%.2f y=%.2f z=%.2f' % arm)
    print('表面法向量:', normal)

    # 把深度图归一化后保存，方便肉眼查看
    depth_show = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    cv2.imwrite('/tmp/depth_demo.png', depth_show)
    print('深度图已保存: /tmp/depth_demo.png')


if __name__ == '__main__':
    _demo()
