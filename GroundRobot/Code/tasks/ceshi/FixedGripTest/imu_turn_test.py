#!/usr/bin/python3
# coding=utf8
"""IMU 转向标定测试：命令转指定角度，同时积分 gz 得到实测角度。"""

import argparse
import json
import os
import sys
import threading
import time

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import make_board, make_ik


def read_gz(board):
    try:
        data = board.get_imu()
        if data is None:
            return None
        return float(data[5])  # ax, ay, az, gx, gy, gz
    except Exception:
        return None


def calibrate_bias(board, samples=100):
    vals = []
    while len(vals) < samples:
        gz = read_gz(board)
        if gz is not None:
            vals.append(gz)
        time.sleep(0.005)
    return sum(vals) / len(vals)


def sample_gyro(board, bias, stop_event, out):
    samples = []
    while not stop_event.is_set():
        gz = read_gz(board)
        if gz is not None:
            samples.append((time.monotonic(), gz - bias))
        time.sleep(0.005)
    out.extend(samples)


def integrate_deg(samples):
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = t1 - t0
        total += (v0 + v1) * 0.5 * dt
    return total


def main():
    parser = argparse.ArgumentParser(description='IMU 转向标定测试')
    parser.add_argument('--angle', type=float, default=90.0)
    parser.add_argument('--step', type=float, default=5.0,
                        help='每次小转角度，默认 5 度')
    parser.add_argument('--speed', type=int, default=30)
    parser.add_argument('--direction', choices=['left', 'right'], default='left')
    parser.add_argument('--samples', type=int, default=100,
                        help='零漂标定采样次数')
    parser.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'imu_turn_result.json'))
    args = parser.parse_args()

    board = make_board()
    ik = make_ik(board)
    board.enable_reception()

    print('标定陀螺仪零漂...', flush=True)
    bias = calibrate_bias(board, samples=args.samples)
    print('gz bias = %.4f' % bias, flush=True)

    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    samples = []
    stop_event = threading.Event()
    th = threading.Thread(target=sample_gyro, args=(board, bias, stop_event, samples))
    th.start()

    commanded = args.angle
    remaining = args.angle
    while remaining > 0:
        move = min(args.step, remaining)
        if args.direction == 'left':
            ik.turn_left(ik.initial_pos, 2, move, args.speed, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, move, args.speed, 1)
        remaining -= move
        time.sleep(0.05)

    time.sleep(0.3)
    stop_event.set()
    th.join()

    measured = integrate_deg(samples)
    print('命令角度: %.1f 度' % commanded, flush=True)
    print('IMU 积分角度: %.1f 度' % measured, flush=True)
    print('误差: %.1f 度' % (measured - commanded), flush=True)

    result = {
        'direction': args.direction,
        'commanded_deg': commanded,
        'measured_deg': measured,
        'error_deg': measured - commanded,
        'gz_bias': bias,
        'samples': len(samples),
    }
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print('结果已保存: %s' % args.out, flush=True)


if __name__ == '__main__':
    main()
