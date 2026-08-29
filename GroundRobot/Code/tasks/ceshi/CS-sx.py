#!/usr/bin/python3
# coding=utf8
"""临时调试：只动 24 号舵机上下扫，检测到目标后停下并蜂鸣。

用途：在不同角度/高度放目标，观察 24 号平滑扫描能不能看到、扫多快合适。
只写 24 号舵机，21 号固定在 500。日志走 agcs_lib.logs。
"""
import argparse
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    make_board, load_params, load_lab_data, load_undistort_maps,
    detect_color, correct_camera, open_camera, capture, make_display, show_status,
)
from agcs_lib.logs import setup_logger, action_msg


def main():
    parser = argparse.ArgumentParser(description='24号舵机上下扫描测试')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=300)
    parser.add_argument('--step', type=int, default=20, help='24号每小步脉宽增量')
    parser.add_argument('--move-ms', type=int, default=10, help='每小步移动时间 ms')
    parser.add_argument('--interval-ms', type=int, default=20, help='移动中取帧间隔 ms')
    parser.add_argument('--start', type=int, default=0, help='扫描起始脉宽')
    parser.add_argument('--end', type=int, default=1000, help='扫描结束脉宽')
    parser.add_argument('--servo', type=int, choices=[21, 24], default=24,
                        help='选择扫描的舵机：21=水平，24=俯仰')
    args = parser.parse_args()

    logger = setup_logger()
    logger.info('启动 %d号扫描测试：color=%s step=%d move_ms=%d interval_ms=%d start=%d end=%d',
                args.servo, args.color, args.step, args.move_ms, args.interval_ms, args.start, args.end)

    board = make_board()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    display = make_display()
    cam = open_camera()

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=args.min_area)

    servo_id = args.servo
    other_id = 21 if servo_id == 24 else 24
    # 只动选中的舵机；另一个固定 500
    board.bus_servo_set_position(0.5, [[other_id, 500]])
    time.sleep(0.5)

    move_sec = max(args.move_ms, 10) / 1000.0
    interval_sec = max(args.interval_ms, 10) / 1000.0
    step = max(1, args.step)
    start = max(0, min(1000, args.start))
    end = max(0, min(1000, args.end))
    if start > end:
        start, end = end, start

    y = start
    direction = 1
    logger.info('[CS-sx] %s', action_msg('开始扫描', action='%d号 %d->%d' % (servo_id, start, end)))
    try:
        while True:
            board.bus_servo_set_position(move_sec, [[servo_id, y]])
            end_time = time.time() + move_sec
            while time.time() < end_time:
                r = detect()
                if r is not None:
                    cx, cy = r['center']
                    logger.info('[CS-sx] %s', action_msg(
                        '检测到目标', action='%d号=%d 中心x=%dpx 中心y=%dpx 面积=%d像素 半径=%dpx'
                        % (servo_id, y, cx, cy, r.get('area', 0), r.get('radius', 0))))
                    show_status(display, 1)
                    board.set_buzzer(2000, 0.05, 0.15, 2)
                    time.sleep(2)
                    show_status(display, 0)
                    # 检测到就停下；本轮调试退出，方便你换个位置再跑
                    cam.camera_close()
                    return
                time.sleep(interval_sec)

            y += direction * step
            if y >= end:
                y = end
                direction = -1
            elif y <= start:
                y = start
                direction = 1
    finally:
        cam.camera_close()


if __name__ == '__main__':
    main()
