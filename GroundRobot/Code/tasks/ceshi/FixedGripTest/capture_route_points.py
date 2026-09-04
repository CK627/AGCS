#!/usr/bin/python3
# coding=utf8
"""自动走 JSON 路线，在 pick/place 点停下来标定并保存成功点位。

流程：
    - 按 fixed_route.json 执行移动；
    - 遇到 pick/place 时停下；
    - 手动用 App 调机械臂；
    - pick 点按 f 执行夹取并观察；place 点按 g 执行放下并观察；
    - 成功：输入 :f1 / :f2 / :g1 / :g2 保存到 success_points.json；
    - 失败：按 u 恢复官方初始位置，重新调机械臂再试。

机械臂读取/恢复顺序固定为 22-23-24-21。
25 不读取：pick=700，place=400。
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
SUCCESS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'success_points.json')

RESTORE_ORDER = [22, 23, 24, 21]
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


def read_command():
    print(':', end='', flush=True)
    buf = ''
    while True:
        ch = getch()
        if ch in ('\r', '\n'):
            print('', flush=True)
            return buf
        if ch == '\x03':
            return None
        if ch in ('\x7f', '\b'):
            if buf:
                buf = buf[:-1]
                print('\b \b', end='', flush=True)
            continue
        buf += ch
        print(ch, end='', flush=True)


def read_pulse(board, servo_id):
    try:
        value = board.bus_servo_read_position(servo_id)
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            return int(value[-1]) if value else None
        return int(value)
    except Exception:
        return None


def read_arm_pulses(board):
    pulses = {}
    for sid in RESTORE_ORDER:
        pulses[str(sid)] = read_pulse(board, sid)
        print('读取舵机 %d: %s' % (sid, pulses[str(sid)]), flush=True)
    return pulses


def restore_arm(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in RESTORE_ORDER])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def load_success():
    if os.path.exists(SUCCESS_PATH):
        with open(SUCCESS_PATH, 'r', encoding='utf-8') as f:
            return json.load(f)
    return {}


def save_success(data):
    with open(SUCCESS_PATH, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def handle_point(board, kind, success_data):
    trigger = 'f' if kind == 'pick' else 'g'
    allowed = ('f1', 'f2') if kind == 'pick' else ('g1', 'g2')
    gripper = GRIPPER_CLOSE if kind == 'pick' else GRIPPER_OPEN

    while True:
        print('已到达 %s 点，请手动用 App 调整机械臂' % kind, flush=True)
        print('调好后按 %s 执行%s并观察' % (trigger, '夹取' if kind == 'pick' else '放下'), flush=True)

        while getch().lower() != trigger:
            pass

        pulses = read_arm_pulses(board)
        pulses['25'] = gripper
        print('夹爪 25 使用固定值: %d' % gripper, flush=True)
        board.bus_servo_set_position(0.8, [[25, gripper]])
        time.sleep(0.8)
        print('观察结果：成功请输入 :%s 或 :%s；失败按 u 重试'
              % (allowed[0], allowed[1]), flush=True)

        cmd = read_command()
        if cmd is None:
            continue
        cmd = cmd.strip().lower()

        if cmd == 'u':
            print('本次失败，恢复官方初始位置并重试', flush=True)
            restore_arm(board, GRIPPER_OPEN)
            continue

        if cmd in allowed:
            key = ('pick' if kind == 'pick' else 'place') + cmd[-1]
            success_data[key] = {'action': kind, 'pulses': pulses}
            save_success(success_data)
            print('已保存 %s 到 %s' % (key, SUCCESS_PATH), flush=True)
            restore_arm(board, gripper)
            return

        print('命令错误，请输入 :%s 或 :%s；失败按 u' % allowed, flush=True)


def main():
    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    success_data = load_success()

    print('共 %d 个动作，开始自动走点并标定' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions):
        name = act.get('action')
        if name in ('pick', 'place'):
            handle_point(board, name, success_data)
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

    print('标定完成，成功点位已保存到 %s' % SUCCESS_PATH, flush=True)


if __name__ == '__main__':
    main()
