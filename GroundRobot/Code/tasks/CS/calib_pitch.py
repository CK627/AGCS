#!/usr/bin/python3
# coding=utf8
"""标定相机下俯角：拍几帧深度，拟合地板平面，算出相机下俯角 pitch 和相机高度。

机器人立正静止（别放植株），跑这个脚本。它拟合地板平面，输出：
    pitch = 相机下俯角（度），直接填给 build_grid2d.py 的 --pitch
    H     = 相机离地板高度（mm），用于定高度带 --min-h/--max-h

用法（先 sudo systemctl stop spiderpi）：
    python3 calib_pitch.py
"""
import sys
import time

import numpy as np

_PKG_ROOT = __import__('os').path.dirname(
    __import__('os').path.dirname(__import__('os').path.dirname(
        __import__('os').path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera, make_board, make_ik, stand


def main():
    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)

    parts = []
    for _ in range(5):
        d = cam.read_depth(timeout_ms=2000)
        if d is None:
            continue
        pcl = cam.depth_to_pointcloud(d)
        valid = ~np.isnan(pcl[:, :, 0])
        pts = pcl[valid].reshape(-1, 3).astype(np.float32)
        pts = pts[pts[:, 2] < 2000]  # 只取 2m 内，避免远处噪声
        parts.append(pts)
        time.sleep(0.2)
    cam.close()
    stand(ik)

    if not parts:
        print('FAIL：没读到深度', flush=True)
        return
    pts = np.vstack(parts)

    # SVD 最小二乘拟合平面：n·p = d
    c = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - c)
    n = Vt[-1]
    # 法向量统一指向地板（相机下方），即 n[1] < 0
    if n[1] > 0:
        n = -n
    d = float(n @ c)
    # 相机离地板高度 = 原点到平面距离（法向量已归一）
    H = abs(d)

    # pitch：相机下俯角 θ。地板法向量（世界系 (0,-1,0)）在相机系里表达为
    #   n = Rx(θ)^T @ (0,-1,0) = (0, -cos θ, +sin θ)   （n[1]<0 指下，已统一）
    # 反推 θ = atan2(n[2], -n[1])。注意别写成 atan2(-n[2], ...)，那样会差个正负号，
    # 传给 depth_to_2d 会把地板压成斜坡。
    pitch = float(np.degrees(np.arctan2(n[2], -n[1])))

    print('地板法向量 n=(%.3f, %.3f, %.3f)' % tuple(n), flush=True)
    print('相机下俯角 pitch ≈ %.1f°' % pitch, flush=True)
    print('相机离地板高度 H ≈ %.0f mm' % H, flush=True)
    print('', flush=True)
    print('建议 build_grid2d.py 参数：', flush=True)
    print('  --pitch %.0f --min-h %.0f --max-h %.0f'
          % (pitch, -H + 50, -H + 1200), flush=True)


if __name__ == '__main__':
    main()
