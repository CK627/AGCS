#!/usr/bin/python3
# coding=utf8
"""临时数据采集（带有效性门）：只扫 21 或 24，检测到完整色块才记录并停止。

用法：
    python3 CS-gd-sx.py --servo 24 --color blue
    python3 CS-gd-sx.py --servo 21 --color blue

有效性门：
    1.6 <= area / (radius * radius) <= 2.4
符合才写入 CSV，避免把碎片/边缘误当目标。
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


def valid_detection(r, min_ratio=1.6, max_ratio=2.4):
    radius = max(float(r.get('radius', 0)), 1.0)
    area = float(r.get('area', 0))
    ratio = area / (radius * radius)
    return min_ratio <= ratio <= max_ratio, ratio


def trimmed_median(values):
    """去掉一个最大和一个最小后取中位数。"""
    if not values:
        return 0.0
    if len(values) >= 3:
        values = sorted(values)[1:-1]
    return values[len(values) // 2]


def main():
    parser = argparse.ArgumentParser(description='带有效性门的单轴扫描采集')
    parser.add_argument('--servo', type=int, choices=[21, 24], default=24,
                        help='21=水平，24=俯仰')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=300)
    parser.add_argument('--step', type=int, default=20)
    parser.add_argument('--move-ms', type=int, default=10)
    parser.add_argument('--interval-ms', type=int, default=20)
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--end', type=int, default=1000)
    parser.add_argument('--label', default='test')
    parser.add_argument('--out', default=None)
    parser.add_argument('--min-ratio', type=float, default=1.6)
    parser.add_argument('--max-ratio', type=float, default=2.4)
    parser.add_argument('--stable-frames', type=int, default=7,
                        help='检测到有效目标后固定位置连续取帧数')
    parser.add_argument('--stable-interval-ms', type=int, default=80,
                        help='稳定采样时每帧间隔 ms')
    args = parser.parse_args()

    logger = setup_logger('CS-gd-sx')
    logger.info('[CS-gd-sx] %s', action_msg(
        '启动采集', action='servo=%d color=%s label=%s step=%d move_ms=%d interval_ms=%d'
        % (args.servo, args.color, args.label, args.step, args.move_ms, args.interval_ms)))

    board = make_board()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    if args.out is None:
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gd_sx_samples.csv')
    else:
        out = args.out

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=args.min_area)

    servo_id = args.servo
    other_id = 21 if servo_id == 24 else 24
    board.bus_servo_set_position(0.5, [[other_id, 500]])
    time.sleep(0.5)

    move_sec = max(args.move_ms, 1) / 1000.0
    interval_sec = max(args.interval_ms, 1) / 1000.0
    step = max(1, args.step)
    start = max(0, min(1000, args.start))
    end = max(0, min(1000, args.end))
    if start > end:
        start, end = end, start

    y = start
    direction = 1
    file_exists = os.path.exists(out) and os.path.getsize(out) > 0
    with open(out, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['time', 'label', 'servo', 'pulse', 'x21', 'y24',
                             'cx', 'cy', 'radius', 'area', 'ratio'])

        logger.info('[CS-gd-sx] %s', action_msg('开始单轴扫描', action='servo=%d %d->%d' % (servo_id, start, end)))
        try:
            while True:
                if servo_id == 24:
                    board.bus_servo_set_position(move_sec, [[24, y]])
                else:
                    board.bus_servo_set_position(move_sec, [[21, y]])
                end_time = time.time() + move_sec
                while time.time() < end_time:
                    r = detect()
                    if r is not None:
                        ok, ratio = valid_detection(r, args.min_ratio, args.max_ratio)
                        if ok:
                            samples = [r]
                            stable_interval = max(args.stable_interval_ms, 1) / 1000.0
                            for _ in range(args.stable_frames - 1):
                                time.sleep(stable_interval)
                                r2 = detect()
                                if r2 is not None:
                                    ok2, _ = valid_detection(r2, args.min_ratio, args.max_ratio)
                                    if ok2:
                                        samples.append(r2)

                            if len(samples) >= 3:
                                cx = int(round(trimmed_median([s['center'][0] for s in samples])))
                                cy = int(round(trimmed_median([s['center'][1] for s in samples])))
                                radius = trimmed_median([s.get('radius', 0) for s in samples])
                                area = trimmed_median([s.get('area', 0) for s in samples])
                            else:
                                cx, cy = r['center']
                                radius = r.get('radius', 0)
                                area = r.get('area', 0)
                            ratio = area / (radius * radius) if radius > 0 else 0.0
                            pulse = y
                            row = [round(time.time(), 2), args.label, servo_id, pulse,
                                   (y if servo_id == 21 else 500),
                                   (y if servo_id == 24 else 500),
                                   cx, cy, round(radius, 1), round(area, 1), round(ratio, 2)]
                            writer.writerow(row)
                            f.flush()
                            logger.info('[CS-gd-sx] %s', action_msg(
                                '稳定采样并记录', action='servo=%d pulse=%d frames=%d cx=%d cy=%d radius=%.1f area=%.1f ratio=%.2f'
                                % (servo_id, pulse, len(samples), cx, cy, radius, area, ratio)))
                            return
                        logger.debug('[CS-gd-sx] %s', action_msg(
                            '检测到但无效', action='pulse=%d ratio=%.2f' % (y, ratio)))
                    time.sleep(interval_sec)

                y += direction * step
                if y >= end:
                    y = end
                    direction = -1
                elif y <= start:
                    y = start
                    direction = 1
        except KeyboardInterrupt:
            logger.info('[CS-gd-sx] %s', action_msg('停止采集', reason='Ctrl+C'))
        finally:
            cam.camera_close()


if __name__ == '__main__':
    main()
