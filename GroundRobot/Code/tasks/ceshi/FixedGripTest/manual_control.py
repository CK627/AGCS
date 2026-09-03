#!/usr/bin/python3
# coding=utf8
"""纯手动控制六足，不记录路线。

用法：
    python3 tasks/ceshi/FixedGripTest/manual_control.py
    python3 tasks/ceshi/FixedGripTest/manual_control.py --step 50 --angle 30 --speed 50

按键：
    w  前进一步
    s  后退一步
    a  左横移一次
    d  右横移一次
    q  左转一次
    e  右转一次
    r  恢复立正
    x  退出
"""

import argparse
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
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    parser = argparse.ArgumentParser(description='手动控制六足，不记录')
    parser.add_argument('--step', type=int, default=50, help='直线距离，默认 50')
    parser.add_argument('--angle', type=int, default=30, help='转弯角度，默认 30')
    parser.add_argument('--speed', type=int, default=50, help='动作速度，默认 50')
    args = parser.parse_args()

    board = make_board()
    ik = make_ik(board)

    print('=== 手动控制模式（不记录） ===', flush=True)
    print('w=前进 s=后退 a=左横移 d=右横移 q=左转 e=右转 r=立正 x=退出', flush=True)
    ik.stand(ik.initial_pos, t=500)

    while True:
        ch = getch().lower()
        if ch == 'w':
            ik.go_forward(ik.initial_pos, 2, args.step, args.speed, 1)
            print('forward', flush=True)
        elif ch == 's':
            ik.back(ik.initial_pos, 2, args.step, args.speed, 1)
            print('back', flush=True)
        elif ch == 'a':
            ik.left_move(ik.initial_pos, 2, args.step, args.speed, 1)
            print('left_move', flush=True)
        elif ch == 'd':
            ik.right_move(ik.initial_pos, 2, args.step, args.speed, 1)
            print('right_move', flush=True)
        elif ch == 'q':
            ik.turn_left(ik.initial_pos, 2, args.angle, args.speed, 1)
            print('turn_left', flush=True)
        elif ch == 'e':
            ik.turn_right(ik.initial_pos, 2, args.angle, args.speed, 1)
            print('turn_right', flush=True)
        elif ch == 'r':
            ik.stand(ik.initial_pos, t=500)
            print('stand', flush=True)
        elif ch == 'x':
            print('退出手动控制', flush=True)
            break
        elif ch == '\x03':
            print('Ctrl+C 退出', flush=True)
            break
        else:
            print('未识别: %r' % ch, flush=True)

        time.sleep(0.08)


if __name__ == '__main__':
    main()
