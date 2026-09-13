#!/usr/bin/python3
# coding=utf8
"""手眼标定：求深度相机坐标系 -> 机械臂坐标系 的 R_cam2arm / t_cam2arm。

方法（点对应，避免正运动学，机器人上最容易跑）：
    机械臂摆到一个固定位姿不动（相机能拍到地面上的标记即可）；把一张 ArUco
    标记放到若干「已知机械臂坐标」的位置（用尺量：正前方多远、离地多高、左右
    偏移），脚本用彩色相机测出标记在相机坐标系的 3D 位置，最后用 Kabsch
    （Procrustes）解出 R/t，存到 config/cam2arm.yaml。

坐标约定：
    相机坐标系：X 右 / Y 下 / Z 前，单位 mm（OpenCV 约定，ArUco tvec 一致）。
    机械臂坐标系：x 右 / y 前 / z 上，单位 cm，原点 = 云台中心在地面的投影。

用法：
    python3 calib_cam2arm.py
    按提示：把标记放到指定位置 -> 输入该位置机械臂坐标 (x,y,z) cm -> 回车采样。
    采满 >=3 个不共线点后按 q 求解并保存。
"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import numpy as np

from agcs_lib.marker import MarkerDetector


def kabsch(cam_pts, arm_pts):
    """点对应求刚体变换：arm ≈ R @ cam + t。

    cam_pts / arm_pts：Nx3，单位一致。返回 R(3x3), t(3x1)。
    """
    cam = np.asarray(cam_pts, dtype=np.float64)
    arm = np.asarray(arm_pts, dtype=np.float64)
    cam_c = cam.mean(axis=0)
    arm_c = arm.mean(axis=0)
    cam_cen = cam - cam_c
    arm_cen = arm - arm_c
    H = cam_cen.T @ arm_cen
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = arm_c - R @ cam_c
    return R, t


def main():
    import cv2
    from calibration.camera import Camera

    detector = MarkerDetector(marker_size_mm=100.0)  # 标记边长 100mm，如不同请改
    cam = Camera()
    cam.camera_open()

    cam_pts = []   # 相机坐标 (mm)
    arm_pts = []   # 机械臂坐标 (cm -> 转成 mm)

    print('=== 手眼标定 ===')
    print('把 ArUco 标记(边长100mm)放到一个「已知机械臂坐标」的位置，')
    print('机械臂坐标 = (x 右, y 前, z 上)，单位 cm，原点在云台中心地面投影。')
    print('每次输入坐标回车采样；采满 >=3 个不共线点后输入 q 求解。')

    try:
        while True:
            frame = cam.frame
            if frame is None:
                continue
            marks = detector.detect(frame)
            if marks:
                m = marks[0]
                tvec = m['tvec']  # mm, 相机坐标 X右/Y下/Z前
                print('  检测到标记 id=%d  相机坐标 (X右,Y下,Z前)mm = (%.0f, %.0f, %.0f)'
                      % (m['id'], tvec[0], tvec[1], tvec[2]))
            else:
                print('  （当前画面没检测到标记）')

            s = input('  输入机械臂坐标 x,y,z (cm)，或 q 求解: ').strip()
            if s.lower() == 'q':
                break
            try:
                x, y, z = [float(v) for v in s.split(',')]
            except Exception:
                print('  格式错误，例如：20,15,8')
                continue
            # 需要当前帧有标记才采样
            if not marks:
                print('  当前帧没标记，先对准标记再输入')
                continue
            cam_pts.append(tvec)
            arm_pts.append([x * 10.0, y * 10.0, z * 10.0])  # cm -> mm
            print('  已采样 %d 个点' % len(cam_pts))
    except KeyboardInterrupt:
        pass
    finally:
        cam.camera_close()

    if len(cam_pts) < 3:
        print('样本不足 3 个，退出')
        return

    R, t = kabsch(cam_pts, arm_pts)
    # 残差检查：用解算的 R/t 反投影，看误差多大
    pred = (np.asarray(cam_pts) @ R.T) + t.ravel()
    err = np.linalg.norm(pred - np.asarray(arm_pts), axis=1)
    print('\n=== 标定结果 ===')
    print('R_cam2arm =')
    print(np.array2string(R, precision=6, suppress_small=True))
    print('t_cam2arm = (mm)', t.ravel())
    print('各点残差 (mm):', np.round(err, 1))
    if err.mean() > 30:
        print('警告：平均残差 %.1fmm 偏大，标定可能不准，建议重新采样' % err.mean())
    else:
        print('平均残差 %.1fmm，标定质量良好' % err.mean())

    # 存成 yaml
    import yaml
    out = {
        'cam2arm': {
            'R': R.tolist(),
            't': [float(v) for v in t.ravel()],
            'unit': 'mm',
        }
    }
    path = os.path.join(_PKG_ROOT, 'config', 'cam2arm.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(out, f, allow_unicode=True, default_flow_style=None)
    print('已保存到', path)


if __name__ == '__main__':
    main()
