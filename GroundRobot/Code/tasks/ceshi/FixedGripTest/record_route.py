#!/usr/bin/python3
# coding=utf8
"""手动录制固定路线：按 w/a/s/d 单步控制六足，并保存动作序列。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/FixedGripTest/record_route.py
    python3 tasks/ceshi/FixedGripTest/record_route.py --step 50 --angle 30 --speed 50

按键：
    w  前进一步
    s  后退一步
    a  左转一次
    d  右转一次
    r  恢复立正（会记录 stand）
    l  查看已记录的动作
    q  保存并退出

记录文件默认保存在同目录 fixed_route.json，可用 --out 修改。
"""

import argparse
import json
import os
import sys
import termios
import time
import tty

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import make_board, make_ik


def getch():
    """读取单个按键，不要求回车。"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def save_route(path, actions):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(actions, f, ensure_ascii=False, indent=2)


def print_help():
    print('w=前进一步  s=后退一步  a=左转一次  d=右转一次', flush=True)
    print('r=恢复立正  l=查看记录  q=保存并退出', flush=True)


def main():
    parser = argparse.ArgumentParser(description='手动录制六足固定路线')
    parser.add_argument('--step', type=int, default=50,
                        help='每次前进/后退距离，默认 50')
    parser.add_argument('--angle', type=int, default=30,
                        help='每次左转/右转角度，默认 30')
    parser.add_argument('--speed', type=int, default=50,
                        help='前进/后退/转弯速度，默认 50')
    parser.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json'),
        help='路线保存路径')
    args = parser.parse_args()

    board = make_board()
    ik = make_ik(board)
    actions = []

    print('=== 固定路线录制器 ===', flush=True)
    print_help()
    print('正在恢复官方立正姿态...', flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    while True:
        ch = getch().lower()
        if ch == 'w':
            ik.go_forward(ik.initial_pos, 2, args.step, args.speed, 1)
            actions.append({'action': 'forward', 'step': args.step, 'speed': args.speed})
            print('记录 %d: forward %d' % (len(actions), args.step), flush=True)
        elif ch == 's':
            ik.back(ik.initial_pos, 2, args.step, args.speed, 1)
            actions.append({'action': 'back', 'step': args.step, 'speed': args.speed})
            print('记录 %d: back %d' % (len(actions), args.step), flush=True)
        elif ch == 'a':
            ik.turn_left(ik.initial_pos, 2, args.angle, args.speed, 1)
            actions.append({'action': 'turn_left', 'angle': args.angle, 'speed': args.speed})
            print('记录 %d: turn_left %d' % (len(actions), args.angle), flush=True)
        elif ch == 'd':
            ik.turn_right(ik.initial_pos, 2, args.angle, args.speed, 1)
            actions.append({'action': 'turn_right', 'angle': args.angle, 'speed': args.speed})
            print('记录 %d: turn_right %d' % (len(actions), args.angle), flush=True)
        elif ch == 'r':
            ik.stand(ik.initial_pos, t=500)
            actions.append({'action': 'stand'})
            print('记录 %d: stand' % len(actions), flush=True)
        elif ch == 'l':
            print('--- 已记录动作 ---', flush=True)
            for i, act in enumerate(actions, 1):
                print('%d: %s' % (i, act), flush=True)
            print('------------------', flush=True)
        elif ch in ('q', 'x'):
            save_route(args.out, actions)
            print('已保存 %d 个动作到: %s' % (len(actions), args.out), flush=True)
            break
        elif ch == '\x03':
            print('Ctrl+C，不保存退出', flush=True)
            break
        else:
            print('未识别按键: %r' % ch, flush=True)
            print_help()

        time.sleep(0.15)


if __name__ == '__main__':
    main()
