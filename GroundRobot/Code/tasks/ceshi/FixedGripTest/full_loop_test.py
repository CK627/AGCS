#!/usr/bin/python3
# coding=utf8
"""只重放完整路线，跳过 pick/place 标记，带超声波避障。"""

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

from agcs_lib import make_board, make_ik, make_ultrasonic, dist_cm


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def wait_if_obstacle(ik, ultrasonic, threshold):
    """前方距离小于阈值时停下，进入手动控制；c 退出整个脚本。"""
    while True:
        distance = dist_cm(ultrasonic)
        if distance >= threshold:
            return

        print('检测到障碍距离 %.2fcm < %.2fcm，已停止' % (distance, threshold), flush=True)
        print('手动操作: w=前进 s=后退 a=左横移 d=右横移 q=左转 e=右转 c=退出', flush=True)

        while True:
            ch = getch().lower()
            if ch == 'w':
                ik.go_forward(ik.initial_pos, 2, 50, 50, 1)
                print('手动 forward', flush=True)
            elif ch == 's':
                ik.back(ik.initial_pos, 2, 50, 50, 1)
                print('手动 back', flush=True)
            elif ch == 'a':
                ik.left_move(ik.initial_pos, 2, 50, 50, 1)
                print('手动 left_move', flush=True)
            elif ch == 'd':
                ik.right_move(ik.initial_pos, 2, 50, 50, 1)
                print('手动 right_move', flush=True)
            elif ch == 'q':
                ik.turn_left(ik.initial_pos, 2, 30, 50, 1)
                print('手动 turn_left', flush=True)
            elif ch == 'e':
                ik.turn_right(ik.initial_pos, 2, 30, 50, 1)
                print('手动 turn_right', flush=True)
            elif ch == 'c':
                print('手动退出，脚本结束', flush=True)
                sys.exit(0)
            else:
                print('未识别: %r' % ch, flush=True)
                continue

            time.sleep(0.08)
            break


def main():
    parser = argparse.ArgumentParser(description='完整路线重放测试，带超声波避障')
    parser.add_argument('--obstacle-distance', type=float, default=1.0,
                        help='超声波距离小于此值(cm)时停下等待手动操作，默认 1.0')
    args = parser.parse_args()

    route_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')
    with open(route_path, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    ultrasonic = make_ultrasonic()
    if ultrasonic is None:
        print('超声波初始化失败，无法启用自动避障', flush=True)
        return 1

    print('共 %d 个动作，开始完整路线测试（跳过 pick/place）' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions, 1):
        wait_if_obstacle(ik, ultrasonic, args.obstacle_distance)
        name = act.get('action')
        speed = int(act.get('speed', 50))

        if name == 'pick':
            print('%d/%d [skip pick]' % (i, len(actions)), flush=True)
        elif name == 'place':
            print('%d/%d [skip place]' % (i, len(actions)), flush=True)
        elif name == 'forward':
            value = int(act.get('step', 50))
            print('%d/%d forward %d' % (i, len(actions), value), flush=True)
            ik.go_forward(ik.initial_pos, 2, value, speed, 1)
        elif name == 'back':
            value = int(act.get('step', 50))
            print('%d/%d back %d' % (i, len(actions), value), flush=True)
            ik.back(ik.initial_pos, 2, value, speed, 1)
        elif name == 'turn_left':
            value = int(act.get('angle', 30))
            print('%d/%d turn_left %d' % (i, len(actions), value), flush=True)
            ik.turn_left(ik.initial_pos, 2, value, speed, 1)
        elif name == 'turn_right':
            value = int(act.get('angle', 30))
            print('%d/%d turn_right %d' % (i, len(actions), value), flush=True)
            ik.turn_right(ik.initial_pos, 2, value, speed, 1)
        elif name == 'left_move':
            value = int(act.get('step', 50))
            print('%d/%d left_move %d' % (i, len(actions), value), flush=True)
            ik.left_move(ik.initial_pos, 2, value, speed, 1)
        elif name == 'right_move':
            value = int(act.get('step', 50))
            print('%d/%d right_move %d' % (i, len(actions), value), flush=True)
            ik.right_move(ik.initial_pos, 2, value, speed, 1)
        elif name == 'stand':
            print('%d/%d stand' % (i, len(actions)), flush=True)
            ik.stand(ik.initial_pos, t=500)
        else:
            print('%d/%d 跳过未知动作 %s' % (i, len(actions), name), flush=True)

        time.sleep(0.08)

    ik.stand(ik.initial_pos, t=500)
    print('完整路线测试结束', flush=True)


if __name__ == '__main__':
    main()
