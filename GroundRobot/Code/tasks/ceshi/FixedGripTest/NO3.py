#!/usr/bin/python3
# coding=utf8
"""NO2：固定路线自动运行，夹取/放下前进行机械臂手动微调。"""

import json
import os
import re
import sys
import time

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


def set_servos(board, pulses, order):
    board.bus_servo_set_position(
        1.2, [[sid, int(pulses[sid])] for sid in order])
    time.sleep(1.2)


def restore_travel(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def clamp_pulse(value):
    return max(0, min(1000, int(value)))


def parse_adjust(cmd):
    """返回 (servo, delta, amount)，解析失败返回 None。"""
    cmd = cmd.strip().lower()
    if not cmd:
        return None

    # a/d 控制 21 号舵机
    if cmd[0] in ('a', 'd'):
        servo = 21
        delta = -1 if cmd[0] == 'a' else 1
        amount_text = cmd[1:]
        amount = int(amount_text) if amount_text else 5
        return servo, delta, amount

    m = re.match(r'^(22|23|24)([ws])(\d*)$', cmd)
    if not m:
        return None
    servo = int(m.group(1))
    delta = 1 if m.group(2) == 'w' else -1
    amount_text = m.group(3)
    amount = int(amount_text) if amount_text else 5
    return servo, delta, amount


def arm_fine_tune(board, state, kind):
    """手动微调 21-24；回车执行夹取/放下，c 退出。"""
    print('机械臂微调中。', flush=True)
    print('21: a/d 左右，默认 5，可 a10/d10', flush=True)
    print('22/23/24: 例如 22w、22s、23w10、24s5', flush=True)
    print('回车=执行%s，c=退出' % ('夹取' if kind == 'pick' else '放下'), flush=True)

    while True:
        print_state(state)
        cmd = input('arm> ').strip().lower()
        if cmd == '':
            break
        if cmd == 'c':
            print('手动退出，脚本结束', flush=True)
            sys.exit(0)

        parsed = parse_adjust(cmd)
        if parsed is None:
            print('命令错误', flush=True)
            continue

        servo, delta, amount = parsed
        state[servo] = clamp_pulse(state[servo] + delta * amount)
        board.bus_servo_set_position(0.2, [[servo, state[servo]]])
        time.sleep(0.1)

    gripper = GRIPPER_CLOSE if kind == 'pick' else GRIPPER_OPEN
    board.bus_servo_set_position(0.8, [[25, gripper]])
    time.sleep(0.8)
    print('已执行%s，25=%d' % ('夹取' if kind == 'pick' else '放下', gripper), flush=True)
    restore_travel(board, gripper)


def print_state(state):
    print('当前 21=%d 22=%d 23=%d 24=%d'
          % (state[21], state[22], state[23], state[24]), flush=True)


def pick1_prepare(board):
    print('pick1 到位，先动 21，再动其他', flush=True)
    set_servos(board, PICK1, [21])
    set_servos(board, PICK1, [22, 23, 24])
    return dict(PICK1)


def pick2_prepare(board):
    print('pick2 到位，先 22-23-(24+100)，再 21，再 24', flush=True)
    temp = dict(PICK2)
    temp[24] = PICK2[24] + 100
    set_servos(board, temp, [22, 23, 24])
    set_servos(board, PICK2, [21])
    set_servos(board, PICK2, [24])
    return dict(PICK2)


def place1_prepare(board):
    print('place1 到位', flush=True)
    set_servos(board, PLACE1, [22, 23, 21, 24])
    return dict(PLACE1)


def main():
    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    arm_state = dict(OFFICIAL_ARM)
    pick_index = 0
    place_index = 0

    print('NO2 启动，共 %d 个路线动作' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions, 1):
        name = act.get('action')

        if name == 'pick':
            pick_index += 1
            print('%d/%d pick%d' % (i, len(actions), pick_index), flush=True)
            if pick_index == 1:
                arm_state = pick1_prepare(board)
            else:
                arm_state = pick2_prepare(board)
            arm_fine_tune(board, arm_state, 'pick')
            continue

        if name == 'place':
            place_index += 1
            if place_index == 1:
                arm_state = place1_prepare(board)
            else:
                print('%d/%d place2 未配置，从当前姿态微调' % (i, len(actions)), flush=True)
                arm_state = dict(OFFICIAL_ARM)
            arm_fine_tune(board, arm_state, 'place')
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
