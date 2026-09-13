#!/usr/bin/python3
# coding=utf8
"""比赛步骤 2.1 视觉追踪（默认黄色）：云台 21/24 PID 跟随色块 + 深度估位置，回传地面站。

完整独立脚本：不依赖 _common.py / ColorTracker，初始化、云台追踪、深度估位
都写在本文件，只 import agcs_lib 核心库（以及官方 common.pid 的 PID 已在本地
用纯 P 复刻）。深度走 OpenNI2、彩色走 /dev/video0（uvcvideo）。

用法（先 sudo systemctl stop spiderpi）：
    cd /home/pi/spiderpi/CompetitionUse
    python3 VisualTracking.py --color yellow
"""
import os
import sys
import time
import math
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2

from agcs_lib import (
    make_board, load_params, load_lab_data, detect_color,
    correct_camera, load_undistort_maps, open_camera, capture,
)
from agcs_lib.depth import DepthCamera

try:
    from communication import task_server
except ImportError:
    task_server = None

# ---- 云台追踪参数（复刻官方 color_track.py 的纯 P 控制）----
P_GAIN = 0.1                  # 比例增益
DEAD_X = 40                   # 水平死区（像素，色块中心 vs 画面中心 320）
DEAD_Y = 60                   # 俯仰死区（像素，色块中心 vs 画面中心 240）
PAN_MIN, PAN_MAX = 0, 1000    # 21 号水平脉宽范围
TILT_MIN, TILT_MAX = 0, 1000  # 24 号俯仰脉宽范围
START_X, START_Y = 500, 260   # 初始 21=500、24=260
FRAME_CX, FRAME_CY = 320, 240  # 画面中心（640x480 的一半）


def main():
    parser = argparse.ArgumentParser(description='2.1 视觉追踪')
    parser.add_argument('--color', default='yellow',
                        choices=['red', 'green', 'blue', 'yellow'])
    args = parser.parse_args()

    # 初始化（不依赖 _common）
    board = make_board()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()

    cam = open_camera()
    if capture(cam, tries=10) is None:
        print('摄像头取帧失败：可能被其他进程占用')
        cam.camera_close()
        return

    # 深度相机（位置估算用，失败则位置/朝向报 0）
    depth_cam = None
    try:
        depth_cam = DepthCamera()
        depth_cam.open()
        depth_cam.start_depth()
    except Exception as e:
        print('深度相机初始化失败: %s' % e)

    if task_server is not None:
        task_server.start_server()
        task_server.set_status(state='TRACKING', message='2.1 视觉追踪')

    # 检测闭包：取帧 → 畸变校正 → 颜色检测（画圈）→ 推流
    def detect(min_area=150):
        f = capture(cam)
        if f is None:
            return None
        frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        r = detect_color(frame, lab, args.color, min_area=min_area)
        if task_server is not None:
            try:
                task_server.publish_frame(frame, max_fps=10.0)
            except Exception:
                pass
        return r

    # 云台追踪状态（21 号水平、24 号俯仰）
    x_dis, y_dis = START_X, START_Y
    board.bus_servo_set_position(0.3, [[24, y_dis], [21, x_dis]])

    pos = {'x': 0.0, 'y': 0.0}
    heading = 0.0
    last_depth_t = 0.0

    print('追踪开始（Ctrl+C 退出）')
    try:
        while True:
            r = detect()
            if r is not None:
                cx, cy = r['center']

                # 21 号水平追踪（纯 P：色块偏右 cx>320 → 输出负 → x_dis 减小）
                if abs(cx - FRAME_CX) >= DEAD_X:
                    x_dis += int(P_GAIN * (FRAME_CX - cx))
                    x_dis = max(PAN_MIN, min(PAN_MAX, x_dis))
                # 24 号俯仰追踪
                if abs(cy - FRAME_CY) >= DEAD_Y:
                    y_dis += int(P_GAIN * (FRAME_CY - cy))
                    y_dis = max(TILT_MIN, min(TILT_MAX, y_dis))
                board.bus_servo_set_position(0.02, [[24, y_dis], [21, x_dis]])

                # 深度估位置 + 朝向（每 0.3s 一次，避免拖慢追踪）
                now = time.time()
                if depth_cam is not None and now - last_depth_t >= 0.3:
                    last_depth_t = now
                    d = depth_cam.read_depth(timeout_ms=100)
                    if d is not None:
                        h, w = d.shape
                        px = min(max(int(cx), 0), w - 1)
                        py = min(max(int(cy), 0), h - 1)
                        z = int(d[py, px])
                        if z > 0:
                            wc = depth_cam.depth_to_world(px, py, float(z))
                            if wc is not None:
                                pos = {'x': round(wc[0] / 1000.0, 3),
                                       'y': round(wc[2] / 1000.0, 3)}
                                heading = round(math.degrees(math.atan2(wc[0], wc[2])), 1)

                if task_server is not None:
                    task_server.set_status(
                        state='TRACKING', position_m=pos, heading_deg=heading,
                        message='追踪中 中心=(%d,%d)' % (cx, cy))
            else:
                if task_server is not None:
                    task_server.set_status(state='TRACKING', message='追踪中 未发现目标')
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])  # 云台回中
        cam.camera_close()
        if depth_cam is not None:
            depth_cam.close()
        if task_server is not None:
            task_server.set_status(state='IDLE', message='追踪结束')


if __name__ == '__main__':
    main()
