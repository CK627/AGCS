#!/usr/bin/python3
# coding=utf8
"""2D 建图：原地转一圈 + 深度压 2D → 累加成 2D 地图。

机器人放到作业空间大致中心（不放植株），原地转一圈（IMU 反馈转角），每转一步
取深度点云压成 2D（墙/障碍轮廓），按当前航向旋转到世界系，累加。输出
map2d.npz（pts: (N,2)，世界 mm，原点 = 机器人起点）。

相比 3D 建图：没有顶棚/护栏干扰、点少、快。

用法（先 sudo systemctl stop spiderpi）：
    python3 build_grid2d.py --out /tmp/map2d.npz --views 24 --pitch 45
"""
import argparse
import os
import sys
import time

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera, make_board, make_ik, stand
from agcs_lib.pcl2d import depth_to_2d, voxel_2d


def read_gz(board):
    try:
        data = board.get_imu()
        return float(data[5]) if data is not None else None
    except Exception:
        return None


def init_imu(board):
    board.enable_reception()
    vals = []
    while len(vals) < 200:
        gz = read_gz(board)
        if gz is not None:
            vals.append(gz)
        time.sleep(0.005)
    return {'bias': sum(vals) / len(vals) if vals else 0.0,
            'yaw': 0.0, 'last_t': time.monotonic()}


def update_yaw(state, board):
    now = time.monotonic()
    dt = now - state['last_t']
    state['last_t'] = now
    gz = read_gz(board)
    if gz is None:
        return
    state['yaw'] += (gz - state['bias']) * dt * 1.18


def turn_to_heading(ik, board, imu_state, target_yaw):
    for _ in range(40):
        update_yaw(imu_state, board)
        err = target_yaw - imu_state['yaw']
        if abs(err) <= 1.0:
            return
        step = min(5.0, abs(err))
        if err > 0:
            ik.turn_left(ik.initial_pos, 2, int(step), 60, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, int(step), 60, 1)
        time.sleep(0.25)


def _rot2(deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]], dtype=np.float32)


def main():
    parser = argparse.ArgumentParser(description='2D 建图（原地转一圈）')
    parser.add_argument('--out', default='/tmp/map2d.npz')
    parser.add_argument('--views', type=int, default=24, help='360° 视角数')
    parser.add_argument('--pitch', type=float, default=45.0, help='相机下俯角(度)')
    parser.add_argument('--min-h', type=float, default=150.0, help='墙高度带下限(mm)')
    parser.add_argument('--max-h', type=float, default=1500.0, help='墙高度带上限(mm)')
    parser.add_argument('--voxel', type=float, default=50.0, help='2D 下采样体素(mm)')
    args = parser.parse_args()

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)
    imu_state = init_imu(board)

    map_parts = []
    step = 360.0 / args.views
    target_yaw = 0.0
    try:
        for v in range(args.views):
            time.sleep(0.5)  # 站稳
            d = cam.read_depth(timeout_ms=2000)
            if d is None:
                print('视角 %d/%d 深度读取失败' % (v + 1, args.views), flush=True)
            else:
                pcl = cam.depth_to_pointcloud(d)
                valid = ~np.isnan(pcl[:, :, 0])
                pts3d = pcl[valid].reshape(-1, 3).astype(np.float32)
                pts2d = depth_to_2d(pts3d, args.pitch, args.min_h, args.max_h)
                if len(pts2d) < 20:
                    print('视角 %d/%d 2D 点太少(%d)' % (v + 1, args.views, len(pts2d)),
                          flush=True)
                else:
                    world = (_rot2(target_yaw) @ pts2d.T).T  # 转世界
                    map_parts.append(world)
                    print('视角 %d/%d yaw=%.1f° 2D点=%d'
                          % (v + 1, args.views, target_yaw, len(pts2d)), flush=True)
            if v < args.views - 1:
                target_yaw += step
                turn_to_heading(ik, board, imu_state, target_yaw)
                time.sleep(0.5)
    finally:
        cam.close()
        stand(ik)

    if not map_parts:
        print('FAIL：没有采到 2D 点', flush=True)
        return
    pts = np.vstack(map_parts)
    pts = voxel_2d(pts, args.voxel)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez_compressed(args.out, pts=pts)
    print('已存 %s (%d 点)，x %.0f..%.0f y %.0f..%.0f'
          % (args.out, len(pts), pts[:, 0].min(), pts[:, 0].max(),
             pts[:, 1].min(), pts[:, 1].max()), flush=True)


if __name__ == '__main__':
    main()
