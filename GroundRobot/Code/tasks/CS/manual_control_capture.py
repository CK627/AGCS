#!/usr/bin/python3
# coding=utf8
"""手动遥控六足行走 + 采集深度点云建图。

机器人贴地自己走（高度恒定），你键盘遥控方向，每走一步自动采集一帧深度点云 +
记录里程计位姿 (R, t)。避免了「抱着机器人」导致的高度变化。

用法（先 sudo systemctl stop spiderpi）：
    python3 manual_control_capture.py --out /tmp/pcl_manual --step 40 --angle 20

按键：
    w 前进   s 后退   a 左横移   d 右横移
    q 左转   e 右转   c 补采一帧  x 退出
"""
import argparse
import os
import sys
import termios
import time
import tty

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (DepthCamera, make_board, make_ik, stand,
                      turn_left, turn_right, go_forward, go_back)
from agcs_lib.pcl import rot_y


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    parser = argparse.ArgumentParser(description='手动控制 + 采集点云')
    parser.add_argument('--out', default='/tmp/pcl_manual')
    parser.add_argument('--step', type=int, default=40, help='直行距离(mm)')
    parser.add_argument('--angle', type=int, default=20, help='转弯角度(度)')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()
    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)

    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)
    idx = 0

    def capture():
        nonlocal idx
        d = cam.read_depth(timeout_ms=2000)
        if d is None:
            print('深度读取失败', flush=True)
            return
        pcl = cam.depth_to_pointcloud(d)
        valid = ~np.isnan(pcl[:, :, 0])
        pts = pcl[valid].reshape(-1, 3).astype(np.float32)
        np.savez_compressed(os.path.join(args.out, 'view_%03d.npz' % idx),
                            pts=pts, R=R, t=t)
        print('已存 view_%03d (%d 点)' % (idx, len(pts)), flush=True)
        idx += 1

    print('=== 遥控采集模式 ===', flush=True)
    print('w前进 s后退 a左横移 d右横移 q左转 e右转 c补采 x退出', flush=True)
    print('每走一步自动采一帧；走一圈回到起点后 x 退出', flush=True)

    try:
        while True:
            ch = getch().lower()
            moved = False
            if ch == 'w':
                go_forward(ik, step=args.step, speed=50, times=1)
                t = t + R @ np.array([0, 0, args.step], dtype=np.float32)
                moved = True
            elif ch == 's':
                go_back(ik, step=args.step, speed=50)
                t = t + R @ np.array([0, 0, -args.step], dtype=np.float32)
                moved = True
            elif ch == 'a':
                # 左横移 = 相机 -X
                t = t + R @ np.array([-args.step, 0, 0], dtype=np.float32)
                ik.left_move(ik.initial_pos, 2, args.step, 50, 1)
                moved = True
            elif ch == 'd':
                # 右横移 = 相机 +X
                t = t + R @ np.array([args.step, 0, 0], dtype=np.float32)
                ik.right_move(ik.initial_pos, 2, args.step, 50, 1)
                moved = True
            elif ch == 'q':
                turn_left(ik, angle=args.angle, speed=60)
                R = rot_y(args.angle).astype(np.float32) @ R
                moved = True
            elif ch == 'e':
                turn_right(ik, angle=args.angle, speed=60)
                R = rot_y(-args.angle).astype(np.float32) @ R
                moved = True
            elif ch == 'c':
                capture()
            elif ch == 'r':
                stand(ik)
            elif ch in ('x', '\x03'):
                print('退出', flush=True)
                break
            else:
                print('未识别: %r' % ch, flush=True)

            if moved:
                time.sleep(0.6)  # 站稳
                capture()
            time.sleep(0.05)
    finally:
        stand(ik)
        cam.close()
    print('采集完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)


if __name__ == '__main__':
    main()
