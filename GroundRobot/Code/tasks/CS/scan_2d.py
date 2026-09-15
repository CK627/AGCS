#!/usr/bin/python3
# coding=utf8
"""2D 扫描（稳版）：机器人放角落，21 号舵机「转一点停一下」扫 180°，当 2D 激光雷达。

    机器人站着不动（没有里程计漂移），只动 21 号舵机（底座横转）：
        pan=500 正前、900 左 90°、100 右 90°
    每个角度停稳了取一帧深度 → 按「离地高度带」滤掉地板和顶棚，只留墙/花盆这类
    竖直障碍 → 按当前 pan 角旋到机器人坐标系 → 累加成 2D 点集。

    相机俯仰角由 24 号舵机姿态决定，脚本开跑前自动标定（拟合地板平面求 pitch 和
    相机高度 H），不用手填 —— 手填 pitch 猜错正是之前 build_grid2d 建图散架的原因。

    稳（每个角度都等舵机停稳 + 整帧丢弃缓冲），但慢：41 个角度约 35 秒。要快就用
    scan_2d_fast.py（一边匀速转一边连拍，一个角约 6 秒）。

    四个角各扫一次，再用 merge_scans.py 拼成完整 2D 地图。

用法（先 sudo systemctl stop spiderpi）：
    python3 scan_2d.py --out /tmp/scan_c1.npz
    python3 scan_2d.py --out /tmp/scan_c1.npz --pitch 32   # 不想自动标定就手填

跑完终端里直接打 ASCII 图，当场看扫得对不对。
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
from agcs_lib.pcl2d import voxel_2d
from agcs_lib.depthscan import band_2d, fit_floor, pan_angle_deg, read_pts, to_robot
from agcs_lib.mapview import ascii_map, draw_map_png

SCAN_ARM = {21: 500, 22: 705, 23: 90, 24: 330}  # 扫描姿态（24 由 --pitch24 覆盖）
SERVO_MOVE_S = 0.25


def main():
    parser = argparse.ArgumentParser(description='2D 扫描（21 舵机转一点停一下，扫 180°）')
    parser.add_argument('--out', default='/tmp/scan_2d.npz')
    parser.add_argument('--png', default='', help='额外存一张 PNG（默认不存）')
    parser.add_argument('--pan-min', type=int, default=100, help='21 起始脉宽(右90°)')
    parser.add_argument('--pan-max', type=int, default=900, help='21 结束脉宽(左90°)')
    parser.add_argument('--pan-step', type=int, default=20, help='21 每步脉宽(20≈4.5°)')
    parser.add_argument('--settle', type=float, default=0.5, help='每步等舵机到位(秒)')
    parser.add_argument('--pitch24', type=int, default=SCAN_ARM[24], help='24 号舵机脉宽(相机俯仰)')
    parser.add_argument('--pitch', type=float, default=-999.0, help='手填相机下俯角(度)，默认自动标定')
    parser.add_argument('--cam-h', type=float, default=300.0,
                        help='地板拟合失败时的相机离地高度(mm)兜底')
    parser.add_argument('--floor-clear', type=float, default=80.0, help='离地高度下限(mm)，滤掉地板')
    parser.add_argument('--obstacle-h', type=float, default=1200.0, help='障碍高度上限(mm，相对地板)')
    parser.add_argument('--z-max', type=float, default=6000.0, help='最远距离(mm)，超过丢弃')
    parser.add_argument('--voxel', type=float, default=50.0, help='2D 下采样体素(mm)')
    parser.add_argument('--cam-offset', default='0,0',
                        help='相机相对 21 转轴的水平偏移 "ox,oy" mm（默认 0,0=忽略）')
    args = parser.parse_args()
    args.cam_offset = tuple(float(v) for v in args.cam_offset.split(','))

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    scan_arm = dict(SCAN_ARM)
    scan_arm[24] = args.pitch24
    board.bus_servo_set_position(0.6, [[sid, scan_arm[sid]] for sid in (21, 22, 23, 24)])
    time.sleep(1.5)

    scans = []
    try:
        # ---- 1. 相机俯仰角（决定高度带，标不准整张图都是歪的）----
        print('正在标定相机俯仰角（别动机器人）...', flush=True)
        fit = fit_floor(cam)
        if fit is None:
            pitch_fit, cam_h = None, args.cam_h
            print('地板平面拟合失败，相机高退回默认 %.0fmm' % cam_h, flush=True)
        else:
            pitch_fit, cam_h, ratio = fit
            print('标定结果：pitch=%.1f° 相机高 H=%.0fmm 地板内点率=%.0f%%'
                  % (pitch_fit, cam_h, ratio * 100), flush=True)
            if not (-40.0 <= pitch_fit <= 70.0):
                print('警告：俯仰角 %.1f° 不像正常值（相机可能没对着地板），'
                      '结果可疑，建议检查 24 号舵机姿态' % pitch_fit, flush=True)
            if not (80.0 <= cam_h <= 1500.0):
                print('警告：相机离地 %.0fmm 不像正常值，结果可疑' % cam_h, flush=True)
        pitch = args.pitch if args.pitch > -900.0 else pitch_fit
        if pitch is None:
            print('FAIL：地板拟合失败，请手填 --pitch（可先跑 calib_pitch.py）', flush=True)
            return
        if args.pitch > -900.0:
            print('俯仰角用手填值 pitch=%.1f°（相机高 H=%.0fmm）' % (pitch, cam_h), flush=True)

        print('高度带 %.0f..%.0f mm（地板在 %.0f）'
              % (-cam_h + args.floor_clear, -cam_h + args.obstacle_h, -cam_h), flush=True)

        # ---- 2. 转一点、停一下，扫一圈 ----
        pans = list(range(args.pan_min, args.pan_max + 1, args.pan_step))
        for k, pan in enumerate(pans, 1):
            board.bus_servo_set_position(SERVO_MOVE_S, [[21, pan], [24, scan_arm[24]]])
            time.sleep(args.settle)
            pts3d = read_pts(cam)
            if pts3d is None:
                print('[%2d/%2d] pan=%3d 读深度失败' % (k, len(pans), pan), flush=True)
                continue
            pts2d = band_2d(pts3d, pitch, cam_h, args.floor_clear, args.obstacle_h, args.z_max)
            if len(pts2d) < 10:
                print('[%2d/%2d] pan=%3d 有效点太少（原始 %d 点）'
                      % (k, len(pans), pan, len(pts3d)), flush=True)
                continue
            ang = pan_angle_deg(pan)
            rob = to_robot(pts2d, ang, args.cam_offset)
            rr = np.hypot(rob[:, 0], rob[:, 1])
            scans.append(rob)
            print('[%2d/%2d] pan=%3d ang=%6.1f° 点=%5d 本帧最近=%.0fmm 中位=%.0fmm'
                  % (k, len(pans), pan, ang, len(rob), rr.min(), np.median(rr)), flush=True)
    finally:
        board.bus_servo_set_position(0.5, [[21, 500], [24, scan_arm[24]]])
        cam.close()
        stand(ik)

    if not scans:
        print('FAIL：一个角度都没扫到点', flush=True)
        return
    pts = voxel_2d(np.vstack(scans), args.voxel)
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=pts, pitch=pitch, cam_h=cam_h,
                        pan_min=args.pan_min, pan_max=args.pan_max)
    print('已存 %s (%d 点)，x %.0f..%.0f y %.0f..%.0f'
          % (args.out, len(pts), pts[:, 0].min(), pts[:, 0].max(),
             pts[:, 1].min(), pts[:, 1].max()), flush=True)
    print(ascii_map(pts, marks=[(0.0, 0.0, 'R')],
                    title='机器人系俯视图（R=机器人，上=正前方）%d 点' % len(pts)), flush=True)

    if args.png:
        if draw_map_png(pts, args.png, marks=[(0.0, 0.0, 'robot', (0, 0, 255))]):
            print('预览图 %s' % args.png, flush=True)


if __name__ == '__main__':
    main()
