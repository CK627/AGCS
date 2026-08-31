#!/usr/bin/python3
# coding=utf8
"""视觉伺服夹取标定 + 峰值判断验证：24 号 200-700 循环，实时跟踪面积峰值。

用法：
    python3 CS-servo-cal.py --color blue

流程：
1. 恢复官方检测位，21 号固定 500，24 号从 200 开始（200→700 循环，步长 100）；
2. 检测到目标 → 进入监测：实时跟踪面积，检测到「面积不再增大（回落 10%）」就记录【预判夹取】；
3. 你缓慢靠近目标，到"夹子能夹起来"的距离按回车 → 记录【实际阈值】；
4. 脚本自动对比「预判夹取面积」和「回车实际面积」：
   - 相差小（<15%）→ 峰值判断算法可行，✅；
   - 相差大 → 峰值判断偏差大，需要调；
5. 24 号 +100 继续，到 700 回 200 下一轮；按 q 退出。
数据在日志和 CSV（同目录 servo_cal.csv）。
"""
import argparse
import csv
import os
import select
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    make_board, make_arm_ik, load_params, load_lab_data,
    detect_color, correct_camera, load_undistort_maps, open_camera, capture,
)
from agcs_lib.logs import setup_logger, action_msg

FRAME_PIXELS = 640 * 480  # 307200


def main():
    parser = argparse.ArgumentParser(description='视觉伺服夹取标定 + 峰值判断验证')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--step', type=int, default=100)
    parser.add_argument('--start', type=int, default=200)
    parser.add_argument('--end', type=int, default=700)
    parser.add_argument('--min-area', type=int, default=200)
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    logger = setup_logger('CS-servo-cal')
    logger.info('[CS-servo-cal] %s', action_msg('启动标定', action='color=%s %d-%d' % (args.color, args.start, args.end)))

    board = make_board()
    ak = make_arm_ik()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    if args.out is None:
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'servo_cal.csv')
    else:
        out = args.out

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=args.min_area)

    ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
    time.sleep(2)
    board.bus_servo_set_position(0.5, [[25, 120]])  # 夹爪张开
    time.sleep(0.5)

    csv_exists = os.path.exists(out) and os.path.getsize(out) > 0
    f_csv = open(out, 'a', newline='')
    writer = csv.writer(f_csv)
    if not csv_exists:
        writer.writerow(['time', 'round', 'event', 'y24', 'area', 'ratio', 'cx', 'cy'])
    f_csv.flush()

    def _read_key():
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
        return None

    y = args.start
    round_no = 1
    unseen_shown = False
    print('=== 标定开始：24号=%d，靠近目标，峰值判断+回车对比，q 退出 ===' % y, flush=True)

    try:
        while True:
            board.bus_servo_set_position(0.05, [[24, y], [21, 500]])
            time.sleep(0.15)

            r = detect()
            if r is not None:
                unseen_shown = False
                area = r.get('area', 0)
                ratio = area / FRAME_PIXELS
                print('找到目标: 24=%d 面积=%d 占比=%.3f' % (y, area, ratio), flush=True)
                logger.info('[CS-servo-cal] %s', action_msg('找到目标', action='24=%d 面积=%d 占比=%.3f' % (y, area, ratio)))

                # ---- 监测阶段：实时跟踪面积峰值，检测回落 ----
                peak_area = 0
                predicted = False
                predicted_area = 0
                predicted_ratio = 0.0
                last_print = 0.0
                while True:
                    r2 = detect()
                    if r2 is not None:
                        a = r2.get('area', 0)
                        if a > peak_area:
                            peak_area = a
                        elif not predicted and peak_area > 0 and a < peak_area * 0.9:
                            # 面积回落 10% → 判定到峰值（夹爪已越过目标）
                            predicted = True
                            predicted_area = peak_area
                            predicted_ratio = peak_area / FRAME_PIXELS
                            logger.info('[CS-servo-cal] %s', action_msg(
                                '预判夹取(峰值)', action='24=%d 面积=%d 占比=%.3f' % (y, peak_area, predicted_ratio)))
                            print('>>> [预判夹取] 面积到峰值: 面积=%d 占比=%.3f' % (peak_area, predicted_ratio), flush=True)
                        # 实时打印当前面积（节流，0.3s 一次）
                        now = time.time()
                        if now - last_print > 0.3:
                            print('  24=%d 当前面积=%d 峰值=%d' % (y, a, peak_area), flush=True)
                            last_print = now

                    key = _read_key()
                    if key == 'q':
                        f_csv.close()
                        cam.camera_close()
                        return
                    if key == '':
                        # 回车：记录实际阈值 + 对比
                        r3 = detect()
                        if r3 is not None:
                            actual_area = r3.get('area', 0)
                            actual_ratio = actual_area / FRAME_PIXELS
                            logger.info('[CS-servo-cal] %s', action_msg(
                                '实际阈值(回车)', action='24=%d 面积=%d 占比=%.3f' % (y, actual_area, actual_ratio)))
                            writer.writerow([round(time.time(), 2), round_no, 'threshold', y, actual_area, round(actual_ratio, 4), r3['center'][0], r3['center'][1]])
                            f_csv.flush()
                            print('>>> [实际阈值] 回车: 面积=%d 占比=%.3f' % (actual_area, actual_ratio), flush=True)
                            # 对比
                            if predicted:
                                diff = abs(actual_area - predicted_area) / predicted_area * 100
                                logger.info('[CS-servo-cal] %s', action_msg(
                                    '对比', action='预判=%d 实际=%d 差=%.1f%%' % (predicted_area, actual_area, diff)))
                                print('>>> [对比] 预判峰值=%d 实际回车=%d 差=%.1f%%' % (predicted_area, actual_area, diff), flush=True)
                                if diff < 15:
                                    logger.info('[CS-servo-cal] %s', action_msg('峰值判断验证通过', reason='差<15%'))
                                    print('>>> ✅ 峰值判断验证通过（差<15%%）', flush=True)
                                else:
                                    logger.info('[CS-servo-cal] %s', action_msg('峰值判断偏差较大', reason='差=%.1f%%' % diff))
                                    print('>>> ⚠️ 峰值判断偏差较大（差=%.1f%%）' % diff, flush=True)
                            else:
                                logger.info('[CS-servo-cal] %s', action_msg('未触发峰值判断', reason='面积未回落'))
                                print('>>> ⚠️ 未触发峰值判断（面积一直涨没回落）', flush=True)
                        break
                    time.sleep(0.05)

                y += args.step
                if y > args.end:
                    y = args.start
                    round_no += 1
                    print('=== 第 %d 轮开始，24 号 = %d ===' % (round_no, y), flush=True)
                else:
                    print('=== 下一个 24 号 = %d ===' % y, flush=True)
            else:
                if not unseen_shown:
                    print('24=%d 目标不可见（把目标放进来）' % y, flush=True)
                    unseen_shown = True
                key = _read_key()
                if key == 'q':
                    break
    except KeyboardInterrupt:
        logger.info('[CS-servo-cal] %s', action_msg('停止标定', reason='Ctrl+C'))
    finally:
        f_csv.close()
        cam.camera_close()
        logger.info('[CS-servo-cal] %s', action_msg('标定结束'))


if __name__ == '__main__':
    main()
