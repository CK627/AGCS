#!/usr/bin/python3
# coding=utf8
"""NO2：按固定路线自动运行两轮夹取，不覆盖任何 JSON。

- 读取 fixed_route.json 的移动路线；
- 遇到 pick 自动执行夹取，pick1/pick2 使用内置固定脉宽；
- 遇到 place 如果 success_points.json 里有 place1/place2 则自动放下，否则跳过；
- 全程不手动确认，只有超声波避障会暂停；
- 避障暂停时：w/a/s/d/q/e 移动，c 退出。
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

from agcs_lib import make_board, make_ik, make_ultrasonic, dist_cm


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}
ARM_ORDER = [22, 23, 21, 24]

PICK1 = {21: 880, 22: 365, 23: 550, 24: 250}
PICK2 = {21: 745, 22: 485, 23: 345, 24: 325}

GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400


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
    while True:
        distance = dist_cm(ultrasonic)
        if distance >= threshold:
            return

        print('检测到障碍 %.2fcm < %.2fcm，已停止' % (distance, threshold), flush=True)
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
                ik.turn_left(ik.initial_pos, 2, 10, 50, 1)
                print('手动 turn_left', flush=True)
            elif ch == 'e':
                ik.turn_right(ik.initial_pos, 2, 10, 50, 1)
                print('手动 turn_right', flush=True)
            elif ch == 'c':
                print('手动退出，脚本结束', flush=True)
                sys.exit(0)
            else:
                continue

            time.sleep(0.08)
            break


def set_arm(board, pulses, gripper):
    board.bus_servo_set_position(
        1.2, [[sid, int(pulses[sid])] for sid in ARM_ORDER])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[25, gripper]])
    time.sleep(0.8)


def restore_travel(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in ARM_ORDER])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description='NO2 固定路线自动运行')
    parser.add_argument('--obstacle-distance', type=float, default=1.0,
                        help='超声波距离小于此值(cm)时暂停，默认 1.0')
    args = parser.parse_args()

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    ultrasonic = make_ultrasonic()
    if ultrasonic is None:
        print('超声波初始化失败', flush=True)
        return 1

    pick_index = 0

    print('NO2 启动，共 %d 个路线动作' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions, 1):
        wait_if_obstacle(ik, ultrasonic, args.obstacle_distance)
        name = act.get('action')

        if name == 'pick':
            pick_index += 1
            pulses = PICK1 if pick_index == 1 else PICK2
            print('%d/%d pick%d 执行夹取' % (i, len(actions), pick_index), flush=True)
            set_arm(board, pulses, GRIPPER_CLOSE)
            restore_travel(board, GRIPPER_CLOSE)
            continue

        if name == 'place':
            print('%d/%d place 固定点打开25号夹爪' % (i, len(actions)), flush=True)
            restore_travel(board, GRIPPER_OPEN)
            continue

        speed = int(act.get('speed', 50))
        print('%d/%d %s' % (i, len(actions), act), flush=True)

        if name == 'forward':
            ik.go_forward(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
        elif name == 'back':
            ik.back(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
        elif name == 'turn_left':
            ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 10)), speed, 1)
        elif name == 'turn_right':
            ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 10)), speed, 1)
        elif name == 'left_move':
            ik.left_move(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
        elif name == 'right_move':
            ik.right_move(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

        time.sleep(0.08)

    ik.stand(ik.initial_pos, t=500)
    print('NO2 运行结束', flush=True)


if __name__ == '__main__':
    main()
