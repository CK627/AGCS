#!/usr/bin/python3
# coding=utf8
"""独立的“像素 -> 机械臂 x,y,z”核心算法（合并版）。

背景
----
单目相机拍到的目标只是 2D 像素坐标 (u, v)。要控制机械臂去抓取，需要知道目标
在机械臂坐标系里的 3D 位置 (x, y, z)。

本文件把原先散落在 vision.py、grab_official.py、CS-grab-alt.py 里的坐标换算
逻辑合并成一套独立实现，共分三层：

1. pixel_to_ground_world()
   像素坐标 -> 地面世界坐标（单位 mm）。

   前提假设：目标落在一个已知平面上，这里默认是地面 z=0。单目相机本身无法
   直接测深，所以通过“目标在地面上”这个平面约束，把 2D 像素反投影成 3D 点。

2. pixel_to_arm_xy()
   把地面世界坐标从 mm 转成 cm，做相机/机械臂方向校正，再加上机械臂检测姿态
   的偏移，得到机械臂 (x, y)。

3. compute_arm_target()
   把 (x, y) 和 z 合并成最终抓取坐标。注意：z 不是从像素算出来的，而是抓取
   高度参数 pick_z，因为单目 + 地面平面假设无法可靠恢复目标高度。

依赖
----
- numpy：矩阵运算
- cv2：cv2.Rodrigues 把旋转向量转成旋转矩阵

这份文件本身是纯算法，不依赖本项目其它模块；运行时只需要传入 K/R/T 和像素点。
"""

import os

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# 0. 标定参数读取（可选，只在需要从 yaml 文件加载时使用）
# ---------------------------------------------------------------------------
def load_block_params(path):
    """从 camera_cal.yaml 读取 block_params，返回 K, R, T。

    - K：相机内参矩阵 3x3，描述焦距和光心，镜头固定后一般不变。
    - R：相机外参旋转向量 3x1，描述相机相对地面的姿态。
    - T：相机外参平移向量 3x1，描述相机相对地面的位置。
    """
    try:
        import yaml
    except ImportError as e:
        raise RuntimeError('缺少 PyYAML，无法读取标定文件') from e

    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    block = data['block_params']
    K = np.array(block['K'], dtype=np.float64).reshape(3, 3)
    R = np.array(block['R'], dtype=np.float64).reshape(3, 1)
    T = np.array(block['T'], dtype=np.float64).reshape(3, 1)
    return K, R, T


# ---------------------------------------------------------------------------
# 1. 像素 -> 地面世界坐标（单位 mm）
# ---------------------------------------------------------------------------
def pixel_to_ground_world(K, R, T, pixel):
    """把图像像素坐标换算成地面世界坐标。

    参数
    ----
    K : 3x3 相机内参矩阵
    R : 3x1 相机外参旋转向量（Rodrigues 形式）
    T : 3x1 相机外参平移向量
    pixel : (u, v) 图像像素坐标

    返回
    ----
    world_mm : numpy 一维数组 [X, Y, Z]，单位 mm；其中 Z 应该接近 0。

    数学推导
    --------
    相机投影模型：
        s * [u, v, 1]^T = K * [R | T] * [X, Y, Z, 1]^T

    如果已知目标在地面平面 Z=0 上，就可以反向求出唯一的 (X, Y)。
    """
    # 统一输入形状，避免外部传入 list / 不同 shape 导致矩阵乘法报错。
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    R = np.asarray(R, dtype=np.float64).reshape(3, 1)
    T = np.asarray(T, dtype=np.float64).reshape(3, 1)

    # 取出像素坐标并转成浮点数。
    u, v = float(pixel[0]), float(pixel[1])

    # 步骤 1：求相机内参矩阵的逆 K^-1。
    # 作用：把像素坐标反投影回相机坐标系，得到“归一化”射线方向。
    inv_k = np.linalg.inv(K)

    # 步骤 2：把外参旋转向量 R 转成 3x3 旋转矩阵，再求逆 R^-1。
    # cv2.Rodrigues 专门做旋转向量 <-> 旋转矩阵 的互相转换。
    r_mat, _ = cv2.Rodrigues(R)
    inv_r = np.linalg.inv(r_mat)

    # 步骤 3：transPlaneToCam = R^-1 * T。
    # 物理意义：相机光心在地面平面坐标系中的位置。
    # 后续用它把“沿射线缩放后的点”平移回正确位置。
    trans_plane_to_cam = inv_r @ T

    # 步骤 4：像素齐次坐标 [u, v, 1]^T。
    pixel_homo = np.array([[u], [v], [1.0]], dtype=np.float64)

    # 步骤 5：ray_cam = K^-1 * [u, v, 1]^T。
    # 结果不是真实深度，而是相机坐标系里指向该像素的一条射线方向。
    ray_cam = inv_k @ pixel_homo

    # 步骤 6：ray_plane = R^-1 * ray_cam。
    # 把这条射线从相机坐标系旋转到地面坐标系。
    ray_plane = inv_r @ ray_cam

    # 步骤 7：scale = transPlaneToCam.z / ray_plane.z。
    # 这是整个算法的核心：沿射线缩放，让射线正好落到 Z=0 的地面平面上。
    # 换句话说，我们“猜”出目标在该射线方向上离相机多远。
    z_cam = float(trans_plane_to_cam[2, 0])
    z_plane = float(ray_plane[2, 0])

    # 防御：如果射线方向几乎平行于地面，z_plane 会接近 0，除法会爆炸。
    # 实际标定正常时不会出现，这里显式给出可读错误，而不是输出 inf。
    if abs(z_plane) < 1e-9:
        raise ValueError('射线方向几乎与地面平行，无法求出地面交点，请检查 R/T 标定')

    scale = z_cam / z_plane

    # 步骤 8：world = scale * ray_plane - transPlaneToCam。
    # 先沿射线方向走到地面平面，再平移回地面坐标原点。
    world_mm = scale * ray_plane - trans_plane_to_cam

    # 返回一维 [X, Y, Z]；理论上 Z 为 0，浮点误差下会是一个非常小的数。
    return world_mm.flatten()


# ---------------------------------------------------------------------------
# 2. 像素 -> 机械臂 (x, y)，单位 cm
# ---------------------------------------------------------------------------
def pixel_to_arm_xy(K, R, T, pixel, initial_xy=(0.0, 15.0)):
    """把像素坐标换算成机械臂水平坐标 (x, y)。

    这一步只处理水平 x/y，z 在第三步单独决定。
    """
    # 先得到地面世界坐标，单位 mm。
    world_mm = pixel_to_ground_world(K, R, T, pixel)

    # mm -> cm：除以 10。
    # 符号取负：因为相机坐标系和机械臂坐标系方向相反。
    dx_cm = -world_mm[0] / 10.0
    dy_cm = -world_mm[1] / 10.0

    # initial_xy 是机械臂检测姿态的 (x, y)，通常为 (0, 15)。
    # 世界坐标算出的是相对偏移，加上检测姿态后才是机械臂绝对目标坐标。
    x_cm = float(initial_xy[0]) + dx_cm
    y_cm = float(initial_xy[1]) + dy_cm
    return x_cm, y_cm


# ---------------------------------------------------------------------------
# 3. 合并成最终机械臂抓取坐标 (x, y, z)
# ---------------------------------------------------------------------------
def compute_arm_target(K, R, T, pixel, initial_pose=(0.0, 15.0, 5.0), pick_z=None):
    """计算机械臂最终抓取坐标 (x, y, z)。

    - x, y：由像素坐标通过 pixel_to_ground_world + pixel_to_arm_xy 算出。
    - z：不是从像素算的，而是抓取高度参数 pick_z。
      默认沿用 initial_pose 的 z，也就是机械臂检测姿态高度。
    """
    x_cm, y_cm = pixel_to_arm_xy(
        K, R, T, pixel,
        initial_xy=(initial_pose[0], initial_pose[1]),
    )

    # z 直接取外部给定的抓取高度；没有给 pick_z 时回退到初始姿态高度。
    z_cm = float(initial_pose[2]) if pick_z is None else float(pick_z)

    return x_cm, y_cm, z_cm


# ---------------------------------------------------------------------------
# 演示：直接运行本文件会加载项目标定参数，并打印换算过程
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    # 优先读取仓库里的 camera_cal.yaml，找不到就退回内置示例参数。
    default_yaml = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..',
        'config',
        'camera_cal.yaml',
    )

    if os.path.exists(default_yaml):
        K, R, T = load_block_params(default_yaml)
        print('已加载标定文件:', default_yaml)
    else:
        # 示例参数，只用于演示；实际机器人必须用现场标定值。
        K = np.array([
            [424.6064659125572, 0.0, 302.62477665348985],
            [0.0, 425.6259206559359, 276.32211219330236],
            [0.0, 0.0, 1.0],
        ])
        R = np.array([[-0.05649842532150532],
                      [-3.0633799663577834],
                      [-0.2680961631560443]])
        T = np.array([[12.081784817362529],
                      [27.65451956772551],
                      [156.65232978134537]])
        print('未找到标定文件，使用内置示例 K/R/T')

    # 演示几个像素点：画面中心、偏左、偏下。
    for pixel in [(320, 240), (160, 240), (320, 400)]:
        world_mm = pixel_to_ground_world(K, R, T, pixel)
        x_cm, y_cm = pixel_to_arm_xy(K, R, T, pixel, initial_xy=(0.0, 15.0))
        xyz = compute_arm_target(K, R, T, pixel, pick_z=-4.0)

        print('\n像素点:', pixel)
        print('  地面世界坐标(mm): X=%.2f Y=%.2f Z=%.4f' % tuple(world_mm))
        print('  机械臂 xy(cm):    x=%.2f y=%.2f' % (x_cm, y_cm))
        print('  最终抓取坐标(cm): x=%.2f y=%.2f z=%.2f' % xyz)
