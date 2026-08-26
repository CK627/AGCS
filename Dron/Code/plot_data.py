#!/usr/bin/python3
# coding=utf8
"""无人机数据可视化：实时曲线 / tlog 日志回放（只读）。

用法:
    python3 plot_data.py --live --seconds 60
    python3 plot_data.py --tlog 飞行日志.tlog
"""
import argparse
import math
import time
from collections import deque

import matplotlib
matplotlib.use('TkAgg')  # 交互式窗口后端；无图形界面可改成 'Agg'
import matplotlib.pyplot as plt
from pymavlink import mavutil

from connect import connect

SAMPLE_RATE = 2  # 每秒采样次数


def plot(t, roll, pitch, yaw, alt, bat):
    """三张子图：姿态 / 高度 / 电池电压。"""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(t, roll, label='roll')
    axs[0].plot(t, pitch, label='pitch')
    axs[0].plot(t, yaw, label='yaw')
    axs[0].set_ylabel('姿态(°)')
    axs[0].legend()
    axs[0].grid(True)
    axs[1].plot(t, alt, color='green')
    axs[1].set_ylabel('相对高度(m)')
    axs[1].grid(True)
    axs[2].plot(t, bat, color='orange')
    axs[2].set_ylabel('电压(V)')
    axs[2].set_xlabel('时间(s)')
    axs[2].grid(True)
    plt.tight_layout()


def run_live(seconds):
    """实时采集 seconds 秒数据并绘图。"""
    master = connect()
    maxlen = max(10, int(seconds * SAMPLE_RATE))
    ts, roll, pitch, yaw, alt, bat = [deque(maxlen=maxlen) for _ in range(6)]
    t0 = time.monotonic()
    print('采集中 %s 秒（Ctrl+C 可提前结束）...' % seconds)
    try:
        while True:
            # 收到任一相关消息即继续；master.messages 保存最新各类型消息
            master.recv_match(
                type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'BATTERY_STATUS'],
                blocking=True, timeout=2)
            msgs = master.messages
            att = msgs.get('ATTITUDE')
            if att is None:
                continue
            now = time.monotonic() - t0
            ts.append(now)
            roll.append(math.degrees(att.roll))
            pitch.append(math.degrees(att.pitch))
            yaw.append(math.degrees(att.yaw))
            gpi = msgs.get('GLOBAL_POSITION_INT')
            alt.append(gpi.relative_alt / 1000.0 if gpi else float('nan'))
            b = msgs.get('BATTERY_STATUS') or msgs.get('SYS_STATUS')
            bat.append(b.voltage_battery / 1000.0 if b else float('nan'))
            if now >= seconds:
                break
    except KeyboardInterrupt:
        print('\n提前结束')
    plot(list(ts), list(roll), list(pitch), list(yaw), list(alt), list(bat))
    plt.show()


def run_file(path):
    """回放 tlog 日志：按 ATTITUDE 消息时间对齐高度/电压。"""
    master = mavutil.mavlink_connection(path)
    ts, roll, pitch, yaw, alt, bat = [], [], [], [], [], []
    last_alt, last_bat, t0 = None, None, None
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == 'ATTITUDE':
            if t0 is None:
                t0 = msg._timestamp
            ts.append(msg._timestamp - t0)
            roll.append(math.degrees(msg.roll))
            pitch.append(math.degrees(msg.pitch))
            yaw.append(math.degrees(msg.yaw))
            alt.append(last_alt if last_alt is not None else float('nan'))
            bat.append(last_bat if last_bat is not None else float('nan'))
        elif mtype == 'GLOBAL_POSITION_INT':
            last_alt = msg.relative_alt / 1000.0
        elif mtype in ('BATTERY_STATUS', 'SYS_STATUS'):
            last_bat = msg.voltage_battery / 1000.0
    if not ts:
        raise SystemExit('日志里没有 ATTITUDE 数据')
    plot(ts, roll, pitch, yaw, alt, bat)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='无人机数据可视化')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--live', action='store_true', help='实时采集模式')
    group.add_argument('--tlog', metavar='文件', help='回放 tlog 日志')
    parser.add_argument('--seconds', type=float, default=60.0, help='实时采集时长')
    args = parser.parse_args()

    if args.live:
        run_live(args.seconds)
    else:
        run_file(args.tlog)


if __name__ == '__main__':
    main()
