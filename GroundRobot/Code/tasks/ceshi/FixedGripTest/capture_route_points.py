#!/usr/bin/python3
# coding=utf8
"""自动走 JSON 路线，在 pick/place 标记处停下记录机械臂脉宽。

流程：
    - 按顺序执行 fixed_route.json 里的前进/后退/横移/转弯；
    - 遇到 pick 或 place 时停止，由你手动用 App 调机械臂；
    - 调好后按 f 记录夹取、按 g 记录放下；
    - 脚本读取 21-25 当前脉宽，替换原 JSON 里对应的 pick/place 记录；
    - 然后机械臂 21-24 恢复官方初始位；pick 时 25 保持夹紧，place 时 25 松开。
"""

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


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}
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


def read_pulse(board, servo_id):
    """读取总线舵机当前脉宽；读不到返回 None。"""
    try:
        value = board.bus_servo_read_position(servo_id)
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            return int(value[-1]) if value else None
        return int(value)
    except Exception:
        return None


def restore_arm_after_record(board, kind):
    """记录完成后恢复机械臂，25 根据 kind 决定保持夹紧还是松开。"""
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [21, 22, 23, 24]])
    time.sleep(1.5)
    gripper = GRIPPER_CLOSE if kind == 'pick' else GRIPPER_OPEN
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def capture_point(board, kind):
    """等待用户按 f/g，读取 21-25 脉宽并返回记录 dict。"""
    print('已到达 %s 点，请手动用 App 调整机械臂' % kind, flush=True)
    print('调好后：按 f 记录夹取；按 g 记录放下', flush=True)

    while True:
        ch = getch().lower()
        if ch == 'f':
            kind = 'pick'
            break
        if ch == 'g':
            kind = 'place'
            break

    pulses = {}
    for sid in [21, 22, 23, 24]:
        pulses[str(sid)] = read_pulse(board, sid)
        print('读取舵机 %d: %s' % (sid, pulses[str(sid)]), flush=True)
    pulses['25'] = GRIPPER_CLOSE if kind == 'pick' else GRIPPER_OPEN
    print('夹爪 25 使用固定值: %s' % pulses['25'], flush=True)

    restore_arm_after_record(board, kind)
    print('已恢复机械臂，更新 JSON 中的 %s 记录' % kind, flush=True)
    return {'action': kind, 'pulses': pulses}


def main():
    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)

    print('共 %d 个动作，开始自动走点并记录夹取/放下' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions):
        name = act.get('action')

        if name in ('pick', 'place'):
            actions[i] = capture_point(board, name)
            with open(ROUTE_PATH, 'w', encoding='utf-8') as f:
                json.dump(actions, f, ensure_ascii=False, indent=2)
            continue

        speed = int(act.get('speed', 50))
        print('%d/%d %s' % (i + 1, len(actions), act), flush=True)

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

    with open(ROUTE_PATH, 'w', encoding='utf-8') as f:
        json.dump(actions, f, ensure_ascii=False, indent=2)
    print('完成，JSON 已更新', flush=True)


if __name__ == '__main__':
    main()
