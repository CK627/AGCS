#!/usr/bin/python3
# coding=utf8
"""临时数据采集：在不同目标高度/平台高度下，记录检测到的位置参数。

用法：
    python3 CS-gd.py --color blue --label 10cm

脚本会只让 24 号舵机上下扫，检测到目标就把关键参数写入 CSV：
    时间, 标签, 24号脉宽, 21号脉宽, cx, cy, radius, area

通过多组不同高度运行，之后分析 radius/area/cy/24 号脉宽与高度、距离的关系。
"""
import argparse
import csv
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    make_board, load_params, load_lab_data, load_undistort_maps,
    detect_color, correct_camera, open_camera, capture,
)
from agcs_lib.logs import setup_logger, action_msg


def main():
    parser = argparse.ArgumentParser(description='高度与检测位置关系采集')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=300)
    parser.add_argument('--step', type=int, default=20)
    parser.add_argument('--move-ms', type=int, default=10)
    parser.add_argument('--interval-ms', type=int, default=20)
    parser.add_argument('--label', default='test', help='本次高度/位置标签，写进 CSV')
    parser.add_argument('--out', default=None, help='CSV 输出路径；默认脚本同目录 gd_samples.csv')
    args = parser.parse_args()

    logger = setup_logger()
    logger.info('[CS-gd] %s', action_msg(
        '启动数据采集', action='color=%s label=%s step=%d move_ms=%d interval_ms=%d'
        % (args.color, args.label, args.step, args.move_ms, args.interval_ms)))

    board = make_board()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    if args.out is None:
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gd_samples.csv')
    else:
        out = args.out

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=args.min_area)

    board.bus_servo_set_position(0.5, [[21, 500]])
    time.sleep(0.5)

    move_sec = max(args.move_ms, 1) / 1000.0
    interval_sec = max(args.interval_ms, 1) / 1000.0
    step = max(1, args.step)
    y = 0
    direction = 1
    file_exists = os.path.exists(out) and os.path.getsize(out) > 0
    with open(out, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['time', 'label', 'y24', 'x21', 'cx', 'cy', 'radius', 'area'])

        logger.info('[CS-gd] %s', action_msg('开始上下扫描并记录', action='CSV=%s' % out))
        try:
            while True:
                board.bus_servo_set_position(move_sec, [[24, y]])
                end_time = time.time() + move_sec
                while time.time() < end_time:
                    r = detect()
                    if r is not None:
                        cx, cy = r['center']
                        row = [round(time.time(), 2), args.label, y, 500,
                               cx, cy, r.get('radius', 0), r.get('area', 0)]
                        writer.writerow(row)
                        f.flush()
                        logger.info('[CS-gd] %s', action_msg(
                            '检测到并记录', action='y24=%d cx=%d cy=%d radius=%d area=%d'
                            % (y, cx, cy, r.get('radius', 0), r.get('area', 0))))
                        return
                    time.sleep(interval_sec)

                y += direction * step
                if y >= 1000:
                    y = 1000
                    direction = -1
                elif y <= 0:
                    y = 0
                    direction = 1
        except KeyboardInterrupt:
            logger.info('[CS-gd] %s', action_msg('停止采集', reason='Ctrl+C'))
        finally:
            cam.camera_close()


if __name__ == '__main__':
    main()
