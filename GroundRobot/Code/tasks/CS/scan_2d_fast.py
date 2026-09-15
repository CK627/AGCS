#!/usr/bin/python3
# coding=utf8
"""2D 快速扫描：21 号舵机一边匀速转、深度相机一边连续拍，靠相邻帧 ICP 串出角度。

为什么快
    scan_2d.py 是「转一点、停一下、拍一张」：41 个角度 ×(0.25s 转动 + 0.5s 等待)
    ≈ 35 秒。本脚本只发一条「用 N 秒从 pan_min 匀速转到 pan_max」的指令，转的过程
    中约 30fps 连续读帧，转完即止 —— 一个角 ≈ N+1 秒（默认 N=5），而且点密得多。

角度怎么来的（不靠时间猜）
    舵机没有位置反馈（本机 bus_servo_read_position 读不到），按「时间线性插值」
    猜角度是不行的：伺服加减速一偏，整张图就歪。这里用数据自己算 ——
    相邻两帧视场重叠一大半（每帧视场约 60°），用 2D ICP 求出它们的相对转角，
    从第一帧串起来得到每帧角度；最后按「从 pan_min 一共转到 pan_max」做一次整体
    缩放，消掉串联累积的漂移。所以**舵机快慢、有没有按 duration 走都不影响**角度，
    只要它单调地从 pan_min 转到 pan_max。

用法（先 sudo systemctl stop spiderpi）：
    python3 scan_2d_fast.py --out /tmp/scan_c1.npz
    python3 scan_2d_fast.py --out /tmp/scan_c1.npz --seconds 8   # 舵机慢就加时间

跑完终端里直接打 ASCII 图，当场看扫得对不对；四个角扫完用 merge_scans.py 合并。
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
from agcs_lib.depthscan import band_2d, chain_angles, fit_floor, pan_angle_deg, read_pts, to_robot
from agcs_lib.mapview import ascii_map, draw_map_png

SCAN_ARM = {21: 500, 22: 705, 23: 90, 24: 330}  # 扫描姿态（24 由 --pitch24 覆盖）


def sweep(cam, board, pan_from, pan_to, seconds, pitch, cam_h, args):
    """连续扫：发一条匀速转指令，边转边读帧，返回 [ (N,2) 相机系 2D 点 ]。"""
    board.bus_servo_set_position(0.6, [[21, pan_from], [24, args.pitch24]])
    time.sleep(1.0)  # 先站到起始角，别把起始段漏了

    frames = []
    t0 = time.monotonic()
    board.bus_servo_set_position(seconds, [[21, pan_to]])
    while time.monotonic() - t0 < seconds + 0.5:
        pts3d = read_pts(cam, flush=0)  # 连续扫时一帧都别丢
        if pts3d is None:
            continue
        pts2d = band_2d(pts3d, pitch, cam_h, args.floor_clear, args.obstacle_h, args.z_max)
        if len(pts2d) < 10:
            continue
        frames.append(pts2d)
        if len(frames) % 25 == 0:
            print('  已采 %d 帧 (%.1fs)' % (len(frames), time.monotonic() - t0), flush=True)
    board.bus_servo_set_position(0.3, [[21, pan_to]])  # 确保停在终点
    time.sleep(0.4)
    return frames


def main():
    parser = argparse.ArgumentParser(description='2D 快速扫描（21 匀速转 + 连拍 + ICP 串角度）')
    parser.add_argument('--out', default='/tmp/scan_2d.npz')
    parser.add_argument('--png', default='', help='额外存一张 PNG（默认不存）')
    parser.add_argument('--pan-from', type=int, default=100, help='21 起始脉宽(右90°)')
    parser.add_argument('--pan-to', type=int, default=900, help='21 结束脉宽(左90°)')
    parser.add_argument('--seconds', type=float, default=5.0, help='扫完这 180° 用几秒')
    parser.add_argument('--pitch24', type=int, default=SCAN_ARM[24], help='24 号舵机脉宽(相机俯仰)')
    parser.add_argument('--pitch', type=float, default=-999.0, help='手填相机下俯角(度)，默认自动标定')
    parser.add_argument('--cam-h', type=float, default=300.0, help='地板拟合失败时的相机高度兜底(mm)')
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
    board.bus_servo_set_position(0.6, [[sid, SCAN_ARM[sid]] for sid in (21, 22, 23, 24)])
    time.sleep(1.5)

    frames = []
    try:
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
                print('警告：俯仰角 %.1f° 不像正常值，结果可疑（检查 24 号舵机姿态）'
                      % pitch_fit, flush=True)
        pitch = args.pitch if args.pitch > -900.0 else pitch_fit
        if pitch is None:
            print('FAIL：地板拟合失败，请手填 --pitch（可先跑 calib_pitch.py）', flush=True)
            return
        print('高度带 %.0f..%.0f mm（地板在 %.0f）'
              % (-cam_h + args.floor_clear, -cam_h + args.obstacle_h, -cam_h), flush=True)

        print('开始连续扫 %.0f° → %.0f°（%.1f 秒）...'
              % (pan_angle_deg(args.pan_from), pan_angle_deg(args.pan_to), args.seconds),
              flush=True)
        t0 = time.monotonic()
        frames = sweep(cam, board, args.pan_from, args.pan_to, args.seconds,
                       pitch, cam_h, args)
        used = time.monotonic() - t0
        print('扫完：%d 帧 / %.1f 秒（含起停，%.1f fps）'
              % (len(frames), used, len(frames) / max(used, 1e-6)), flush=True)
        if len(frames) < 8:
            print('警告：帧数太少，舵机可能比 --seconds 快很多，把 --seconds 调大重扫',
                  flush=True)
    finally:
        board.bus_servo_set_position(0.5, [[21, 500], [24, args.pitch24]])
        cam.close()
        stand(ik)

    if not frames:
        print('FAIL：一帧都没采到', flush=True)
        return

    angles, info = chain_angles(frames, args.pan_from, args.pan_to)
    print('ICP 串角度：%d 段（每段 %d 帧 ≈ %.1f°），总转角 %.1f°（目标 %.1f°），'
          '缩放 %.3f，失败 %d 段'
          % (info['links'], info['stride'], abs(info['d_mean']) * info['stride'],
             info['icp_total'], info['target'], info['scale'], info['fails']), flush=True)
    if not (0.85 <= info['scale'] <= 1.18):
        print('警告：缩放 %.3f 偏离 1 太多，串角度多半没串好（舵机中途卡过？），'
              '结果可疑' % info['scale'], flush=True)
    if info['fails'] > 0:
        print('警告：有 %d 段 ICP 没配上，那几段角度是按上一段推的，会偏' % info['fails'],
              flush=True)

    parts = [to_robot(f, a, args.cam_offset) for f, a in zip(frames, angles)]
    pts = voxel_2d(np.vstack(parts), args.voxel)

    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(args.out, pts=pts, pitch=pitch, cam_h=cam_h,
                        pan_from=args.pan_from, pan_to=args.pan_to)
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
