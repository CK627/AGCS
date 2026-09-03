#!/usr/bin/python3
# coding=utf8
"""固定路线重放 + 机械臂固定夹取，不读取 JSON。

当前固定路线：
    forward   50mm x 25
    left_move 50mm x 9

夹取固定点：
    21=715, 22=520, 23=335, 24=245, 25=700
"""

import os
import sys
import time

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import make_board, make_ik


# 固定路线，后续如果要改放下路径，直接改这里。
ROUTE = (
    [('forward', 50)] * 25 +
    [('left_move', 50)] * 9
)

PICK_ARM_PULSES = {21: 715, 22: 520, 23: 335, 24: 245}
PICK_GRIPPER_PULSE = 700


def main():
    board = make_board()
    ik = make_ik(board)

    print('共 %d 个动作，开始按固定路线重放' % len(ROUTE), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, (name, value) in enumerate(ROUTE, 1):
        if name == 'forward':
            print('%d/%d forward %d' % (i, len(ROUTE), value), flush=True)
            ik.go_forward(ik.initial_pos, 2, value, 50, 1)
        elif name == 'back':
            print('%d/%d back %d' % (i, len(ROUTE), value), flush=True)
            ik.back(ik.initial_pos, 2, value, 50, 1)
        elif name == 'turn_left':
            print('%d/%d turn_left %d' % (i, len(ROUTE), value), flush=True)
            ik.turn_left(ik.initial_pos, 2, value, 50, 1)
        elif name == 'turn_right':
            print('%d/%d turn_right %d' % (i, len(ROUTE), value), flush=True)
            ik.turn_right(ik.initial_pos, 2, value, 50, 1)
        elif name == 'left_move':
            print('%d/%d left_move %d' % (i, len(ROUTE), value), flush=True)
            ik.left_move(ik.initial_pos, 2, value, 50, 1)
        elif name == 'right_move':
            print('%d/%d right_move %d' % (i, len(ROUTE), value), flush=True)
            ik.right_move(ik.initial_pos, 2, value, 50, 1)
        elif name == 'stand':
            print('%d/%d stand' % (i, len(ROUTE)), flush=True)
            ik.stand(ik.initial_pos, t=500)
        else:
            print('跳过未知动作: %s' % name, flush=True)

        time.sleep(0.08)

    ik.stand(ik.initial_pos, t=500)
    print('固定路线重放完成，开始机械臂夹取', flush=True)

    print('机械臂到固定夹取点: 21=715 22=520 23=335 24=245', flush=True)
    board.bus_servo_set_position(
        1.2, [[sid, PICK_ARM_PULSES[sid]] for sid in [21, 22, 23, 24]])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[25, PICK_GRIPPER_PULSE]])
    time.sleep(0.8)
    print('25 夹取完成: %d' % PICK_GRIPPER_PULSE, flush=True)


if __name__ == '__main__':
    main()
