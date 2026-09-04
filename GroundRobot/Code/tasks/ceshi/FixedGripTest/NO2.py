#!/usr/bin/python3
# coding=utf8
"""NO2：固定路线自动运行两轮夹取。无超声波避障，支持临时手动微调。"""

import json
import os
import select
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


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}

PICK1 = {21: 825, 22: 350, 23: 425, 24: 460}
PLACE1 = {21: 825, 22: 350, 23: 425, 24: 460}
PICK2 = {21: 735, 22: 610, 23: 205, 24: 410}

GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400

MOVE_SPEED = 30
TURN_SPEED = 30


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def read_key_nowait():
    if select.select([sys.stdin], [], [], 0)[0]:
        return getch().lower()
    return None


def manual_fine_tune(ik):
    print('进入临时手动微调：w=前进 s=后退 a=左横移 d=右横移 q=左转 e=右转 r=立正 x=继续 c=退出', flush=True)
    while True:
        ch = getch().lower()
        if ch == 'w':
            ik.go_forward(ik.initial_pos, 2, 50, MOVE_SPEED, 1)
            print('微调 forward', flush=True)
        elif ch == 's':
            ik.back(ik.initial_pos, 2, 50, MOVE_SPEED, 1)
            print('微调 back', flush=True)
        elif ch == 'a':
            ik.left_move(ik.initial_pos, 2, 50, MOVE_SPEED, 1)
            print('微调 left_move', flush=True)
        elif ch == 'd':
            ik.right_move(ik.initial_pos, 2, 50, MOVE_SPEED, 1)
            print('微调 right_move', flush=True)
        elif ch == 'q':
            ik.turn_left(ik.initial_pos, 2, 10, TURN_SPEED, 1)
            print('微调 turn_left', flush=True)
        elif ch == 'e':
            ik.turn_right(ik.initial_pos, 2, 10, TURN_SPEED, 1)
            print('微调 turn_right', flush=True)
        elif ch == 'r':
            ik.stand(ik.initial_pos, t=500)
            print('微调 stand', flush=True)
        elif ch == 'x':
            print('退出微调，继续自动路线', flush=True)
            return
        elif ch == 'c':
            print('手动退出，脚本结束', flush=True)
            sys.exit(0)
        time.sleep(0.08)


def pick_first(board):
    print('pick1：先动 21，再动其他关节', flush=True)
    board.bus_servo_set_position(1.2, [[21, PICK1[21]]])
    time.sleep(1.2)
    board.bus_servo_set_position(
        1.2, [[22, PICK1[22]], [23, PICK1[23]], [24, PICK1[24]]])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[25, GRIPPER_CLOSE]])
    time.sleep(0.8)


def pick_second(board):
    print('pick2：先 22-23-(24+100)，再 21，再 24-100 后夹取', flush=True)
    board.bus_servo_set_position(
        1.2, [[22, PICK2[22]], [23, PICK2[23]], [24, PICK2[24] + 100]])
    time.sleep(1.2)
    board.bus_servo_set_position(1.2, [[21, PICK2[21]]])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[24, PICK2[24]]])
    time.sleep(0.8)
    board.bus_servo_set_position(0.8, [[25, GRIPPER_CLOSE]])
    time.sleep(0.8)


def place_first(board):
    print('place1：到固定点并打开 25', flush=True)
    board.bus_servo_set_position(
        1.2, [[sid, PLACE1[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[25, GRIPPER_OPEN]])
    time.sleep(0.8)


def restore_travel(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def main():
    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)

    pick_index = 0
    place_index = 0

    print('NO2 启动，共 %d 个路线动作，无避障' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions, 1):
        key = read_key_nowait()
        if key == 'm':
            manual_fine_tune(ik)

        name = act.get('action')

        if name == 'pick':
            pick_index += 1
            print('%d/%d pick%d' % (i, len(actions), pick_index), flush=True)
            if pick_index == 1:
                pick_first(board)
            else:
                pick_second(board)
            restore_travel(board, GRIPPER_CLOSE)
            continue

        if name == 'place':
            place_index += 1
            if place_index == 1:
                place_first(board)
                restore_travel(board, GRIPPER_OPEN)
            else:
                print('%d/%d place2 未配置，停车等待手动处理' % (i, len(actions)), flush=True)
                manual_fine_tune(ik)
            continue

        print('%d/%d %s' % (i, len(actions), act), flush=True)

        if name == 'forward':
            ik.go_forward(ik.initial_pos, 2, int(act.get('step', 50)), MOVE_SPEED, 1)
        elif name == 'back':
            ik.back(ik.initial_pos, 2, int(act.get('step', 50)), MOVE_SPEED, 1)
        elif name == 'turn_left':
            ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
        elif name == 'turn_right':
            ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
        elif name == 'left_move':
            ik.left_move(ik.initial_pos, 2, int(act.get('step', 50)), MOVE_SPEED, 1)
        elif name == 'right_move':
            ik.right_move(ik.initial_pos, 2, int(act.get('step', 50)), MOVE_SPEED, 1)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

        time.sleep(0.08)

    ik.stand(ik.initial_pos, t=500)
    print('NO2 运行结束', flush=True)


if __name__ == '__main__':
    main()
