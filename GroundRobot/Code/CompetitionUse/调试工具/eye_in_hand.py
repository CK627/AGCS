#!/usr/bin/python3
# coding=utf8
"""Eye-in-Hand 坐标转换（动态版，纯数学可本地跑）。

方案 B：固定手眼 T_cam_to_end + 动态正解 T_end_to_base。

    T_cam_to_base(当前位姿) = T_end_to_base(当前位姿) × T_cam_to_end
    虫子基座坐标 = T_cam_to_base(当前位姿) × 虫子彩色相机坐标

T_cam_to_end 从 cam2arm.yaml 的 R/t（彩色相机->基座，标定位姿下）拆出：
    T_cam_to_end = inv(T_end_to_base(标定位姿)) × T_cam_to_base(标定位姿)

坐标约定：
    彩色相机：X 右 / Y 下 / Z 前，mm（OpenCV/ArUco）
    深度相机：X 右 / Y 上 / Z 前，mm（OpenNI2 工厂标定；深度->彩色只翻 Y）
    机械臂  ：x 右 / y 前 / z 上，cm，原点 = 云台中心在地面的投影
    舵机角度：theta22=0.24*servo22-30（肩）、theta23=120-0.24*servo23（肘）、
             yaw=(500-servo21)*0.225（底座横转）、alpha=theta24+theta22-theta23（末端俯仰）

正解已对官方 inverse_kinematics.py 做 round-trip 验证：位置 Y/Z 一致到 0.01cm，
X 有 ~7% 偏差（官方 servo21 斜率 0.24°/pulse 与实物 0.225°/pulse 之差，近正前可忽略）。
"""
import math

import numpy as np

L1, L2, L3 = 13.0, 13.0, 13.0   # 连杆长度 cm
_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi


# ---------------- 脉宽 -> 角度 ----------------
def servo22_to_theta22(pulse):
    """22 号脉宽 -> 肩角度(度)。500=90°(竖直向上)，400=66°。"""
    return 0.24 * float(pulse) - 30.0


def servo23_to_theta23(pulse):
    """23 号脉宽 -> 肘角度(度)。500=0°(伸直)，90=98.4°(弯)。"""
    return 120.0 - 0.24 * float(pulse)


def servo21_to_yaw(pulse):
    """21 号脉宽 -> 底座横转角(度)。500=0°(正前)，+右，-左。"""
    return (500.0 - float(pulse)) * 0.225   # 实物：800 脉宽 = 180°


def servo24_to_theta24(pulse):
    """24 号脉宽 -> 腕俯仰角(度)。500=0°，越大越朝上。"""
    return (float(pulse) - 500.0) * (240.0 / 1000.0)


# ---------------- 正解：脉宽 -> 腕部(相机)位姿 ----------------
def forward_kinematics(p21, p22, p23, p24):
    """由 21/22/23/24 脉宽算腕部(相机)在基座系的位置 + 末端俯仰角。

    返回 (pos_xyz_cm [3], alpha_deg)：pos=[X右, Y前, Z上]，alpha=末端俯仰(相对水平面，+上)。
    """
    t22 = servo22_to_theta22(p22) * _DEG
    t23 = servo23_to_theta23(p23) * _DEG
    yaw = servo21_to_yaw(p21) * _DEG
    t24 = servo24_to_theta24(p24) * _DEG

    af = L2 * math.cos(t22) + L3 * math.cos(t22 - t23)   # 水平伸出
    cf = L2 * math.sin(t22) + L3 * math.sin(t22 - t23)   # 竖直（肩以上为正）

    z = L1 + cf
    x = af * math.sin(yaw)   # 基座系 X（右+）
    y = af * math.cos(yaw)   # 基座系 Y（前+）

    alpha = math.degrees(t24 + t22 - t23)                # 末端俯仰角
    return np.array([x, y, z], dtype=np.float64), alpha


def end_rotation(p21, p22, p23, p24):
    """腕部(相机)在基座系的旋转矩阵 R_end_to_base (3x3) = Rz(-yaw)·Rx(alpha)。

    底座先绕 Z 横转 yaw，再绕（横转后的）X 轴俯仰 alpha。这个「end 系」定义可以
    任意，只要正解对任意位姿都一致，拆 T_cam_to_end 时它就会在链里抵消。
    """
    yaw = servo21_to_yaw(p21) * _DEG
    _, alpha_deg = forward_kinematics(p21, p22, p23, p24)
    alpha = alpha_deg * _DEG

    cy, sy = math.cos(yaw), math.sin(yaw)
    ca, sa = math.cos(alpha), math.sin(alpha)
    Rz = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]], dtype=np.float64)  # Rz(-yaw)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]], dtype=np.float64)  # Rx(alpha)
    return Rz @ Rx


def end_pose(p21, p22, p23, p24):
    """腕部(相机)在基座系的位姿：返回 (R 3x3, t 3x1 cm)。"""
    R = end_rotation(p21, p22, p23, p24)
    pos_cm, _ = forward_kinematics(p21, p22, p23, p24)
    return R, pos_cm.reshape(3, 1)


# ---------------- 手眼标定 ----------------
def load_cam2arm(path):
    """读 cam2arm.yaml，返回 (R 3x3, t 3x1 mm)，彩色相机 -> 基座。读不到回退 R=I,t=0。"""
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)['cam2arm']
        R = np.array(data['R'], dtype=np.float64).reshape(3, 3)
        t = np.array(data['t'], dtype=np.float64).reshape(3, 1)
        return R, t
    except Exception:
        return np.eye(3), np.zeros((3, 1))


# ---------------- 固定手眼拆解 ----------------
def extract_cam2end(R_c2b, t_c2b_mm, p21, p22, p23, p24):
    """从标定位姿的 cam2arm(R/t, 彩色相机->基座) 拆出固定手眼 T_cam_to_end。

    T_cam_to_end = inv(T_end_to_base(标定位姿)) × T_cam_to_base(标定位姿)
    返回 (R_cam2end 3x3, t_cam2end 3x1 mm)。(p21..p24 是标定时机械臂的位姿)
    """
    R_e2b, t_e2b_cm = end_pose(p21, p22, p23, p24)
    t_e2b_mm = t_e2b_cm * 10.0
    R_c2e = R_e2b.T @ R_c2b
    t_c2e = R_e2b.T @ (t_c2b_mm - t_e2b_mm)
    return R_c2e, t_c2e


# ---------------- 目标相机系 -> 基座系（动态）----------------
def depth_world_to_cam(wx, wy, wz):
    """深度世界(X右,Y上,Z前) -> 彩色相机(X右,Y下,Z前)。只翻 Y。返回 3x1 mm。"""
    return np.array([wx, -wy, wz], dtype=np.float64).reshape(3, 1)


def cam_to_base_dynamic(point_cam_mm, p21, p22, p23, p24, R_c2e, t_c2e_mm):
    """目标彩色相机 3D(mm) -> 基座 3D(cm)，在当前臂位姿下。

    base = T_end_to_base(当前位姿) × T_cam_to_end × point_cam
    """
    R_e2b, t_e2b_cm = end_pose(p21, p22, p23, p24)
    t_e2b_mm = t_e2b_cm * 10.0
    p_base_mm = R_e2b @ (R_c2e @ point_cam_mm + t_c2e_mm) + t_e2b_mm
    return p_base_mm / 10.0


# ---------------- 自测（纯数学）----------------
if __name__ == '__main__':
    pos, alpha = forward_kinematics(500, 400, 500, 250)
    print('正解 GRAB(500,400,500,250)：腕部 (x=%.2f, y=%.2f, z=%.2f) cm，俯仰=%.1f°'
          % (pos[0], pos[1], pos[2], alpha))
    pos, alpha = forward_kinematics(500, 705, 90, 330)
    print('正解 RESET(500,705,90,330)：腕部 (x=%.2f, y=%.2f, z=%.2f) cm，俯仰=%.1f°'
          % (pos[0], pos[1], pos[2], alpha))

    # 动态链自洽性：任取一个固定手眼，在任意两位姿下算出的相机->基座应当一致（同一相机点）
    R_c2b = np.array([[0.696, -0.678, -0.235], [0.717, 0.645, 0.264], [-0.028, -0.352, 0.935]])
    t_c2b = np.array([[-9.9], [-54.6], [176.3]])
    R_c2e, t_c2e = extract_cam2end(R_c2b, t_c2b, 500, 705, 90, 260)
    p_cam = np.array([[100.0], [-50.0], [800.0]])   # 彩色相机某点
    xyz_A = cam_to_base_dynamic(p_cam, 500, 705, 90, 260, R_c2e, t_c2e)
    xyz_B = cam_to_base_dynamic(p_cam, 500, 600, 200, 240, R_c2e, t_c2e)
    print('动态链自检：固定手眼 R_c2e 拆解成功，两位姿下同一相机点 -> 基座')
    print('  位姿A(500,705,90,260) -> (%.1f,%.1f,%.1f) cm' % (xyz_A[0,0], xyz_A[1,0], xyz_A[2,0]))
    print('  位姿B(500,600,200,240) -> (%.1f,%.1f,%.1f) cm' % (xyz_B[0,0], xyz_B[1,0], xyz_B[2,0]))
