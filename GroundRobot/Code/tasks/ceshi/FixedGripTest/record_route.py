#!/usr/bin/python3
# coding=utf8
"""手动录制固定路线：按 w/a/s/d 单步控制六足，并保存动作序列。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/FixedGripTest/record_route.py
    python3 tasks/ceshi/FixedGripTest/record_route.py --step 50 --angle 10 --speed 50

按键：
    w  前进一步
    s  后退一步
    a  左横移一次
    d  右横移一次
    q  左转 10 度
    e  右转 10 度
    :w50  临时前进一步，步长 50
    :a50  临时左横移一次，步长 50
    :q10  临时左转 10 度
    f  标记：夹取
    g  标记：放下
    r  恢复立正（会记录 stand）
    u  撤销上一个动作（并反向执行）
    c  清空全部记录
    l  查看已记录的动作
    x  保存并退出

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


def read_command():
    """读取冒号后的临时命令，回车结束。"""
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


def apply_one_off(ik, action, value, speed, actions):
    """执行并记录一个临时参数动作。"""
    if action == 'w':
        ik.go_forward(ik.initial_pos, 2, value, speed, 1)
        actions.append({'index': len(actions) + 1, 'action': 'forward', 'step': value, 'speed': speed})
        print('记录 %d: forward %d' % (len(actions), value), flush=True)
    elif action == 's':
        ik.back(ik.initial_pos, 2, value, speed, 1)
        actions.append({'index': len(actions) + 1, 'action': 'back', 'step': value, 'speed': speed})
        print('记录 %d: back %d' % (len(actions), value), flush=True)
    elif action == 'a':
        ik.left_move(ik.initial_pos, 2, value, speed, 1)
        actions.append({'index': len(actions) + 1, 'action': 'left_move', 'step': value, 'speed': speed})
        print('记录 %d: left_move %d' % (len(actions), value), flush=True)
    elif action == 'd':
        ik.right_move(ik.initial_pos, 2, value, speed, 1)
        actions.append({'index': len(actions) + 1, 'action': 'right_move', 'step': value, 'speed': speed})
        print('记录 %d: right_move %d' % (len(actions), value), flush=True)
    elif action == 'q':
        ik.turn_left(ik.initial_pos, 2, value, speed, 1)
        actions.append({'index': len(actions) + 1, 'action': 'turn_left', 'angle': value, 'speed': speed})
        print('记录 %d: turn_left %d' % (len(actions), value), flush=True)
    elif action == 'e':
        ik.turn_right(ik.initial_pos, 2, value, speed, 1)
        actions.append({'index': len(actions) + 1, 'action': 'turn_right', 'angle': value, 'speed': speed})
        print('记录 %d: turn_right %d' % (len(actions), value), flush=True)
    else:
        print('不支持的临时动作: %s' % action, flush=True)


def save_route(path, actions):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(actions, f, ensure_ascii=False, indent=2)


def inverse_ik(ik, act):
    """反向执行一个已记录动作，用于撤销。"""
    name = act.get('action')
    if name in ('pick', 'place'):
        return
    speed = int(act.get('speed', 50))
    if name == 'forward':
        ik.back(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
    elif name == 'back':
        ik.go_forward(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
    elif name == 'turn_left':
        ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 30)), speed, 1)
    elif name == 'turn_right':
        ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 30)), speed, 1)
    elif name == 'left_move':
        ik.right_move(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
    elif name == 'right_move':
        ik.left_move(ik.initial_pos, 2, int(act.get('step', 50)), speed, 1)
    elif name == 'stand':
        ik.stand(ik.initial_pos, t=500)


def print_help():
    print('w=前进  s=后退  a=左横移  d=右横移  q=左转10度  e=右转10度', flush=True)
    print('临时参数: :w50 :s50 :a50 :d50 :q10 :e10', flush=True)
    print('f=标记夹取  g=标记放下  r=恢复立正  u=撤销上一步  c=清空记录  l=查看记录  x=保存退出', flush=True)


def main():
    parser = argparse.ArgumentParser(description='手动录制六足固定路线')
    parser.add_argument('--step', type=int, default=50,
                        help='每次前进/后退距离，默认 50')
    parser.add_argument('--angle', type=int, default=10,
                        help='每次左转/右转角度，默认 10')
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
        if ch == ':':
            cmd = read_command()
            if cmd is None:
                print('取消临时命令', flush=True)
            elif len(cmd) < 2:
                print('临时命令格式错误，示例: :w50', flush=True)
            else:
                action = cmd[0].lower()
                try:
                    value = int(cmd[1:])
                except ValueError:
                    print('临时命令数值错误: %s' % cmd, flush=True)
                else:
                    apply_one_off(ik, action, value, args.speed, actions)
        elif ch == 'w':
            ik.go_forward(ik.initial_pos, 2, args.step, args.speed, 1)
            actions.append({'index': len(actions) + 1, 'action': 'forward', 'step': args.step, 'speed': args.speed})
            print('记录 %d: forward %d' % (len(actions), args.step), flush=True)
        elif ch == 's':
            ik.back(ik.initial_pos, 2, args.step, args.speed, 1)
            actions.append({'index': len(actions) + 1, 'action': 'back', 'step': args.step, 'speed': args.speed})
            print('记录 %d: back %d' % (len(actions), args.step), flush=True)
        elif ch == 'a':
            ik.left_move(ik.initial_pos, 2, args.step, args.speed, 1)
            actions.append({'index': len(actions) + 1, 'action': 'left_move', 'step': args.step, 'speed': args.speed})
            print('记录 %d: left_move %d' % (len(actions), args.step), flush=True)
        elif ch == 'd':
            ik.right_move(ik.initial_pos, 2, args.step, args.speed, 1)
            actions.append({'index': len(actions) + 1, 'action': 'right_move', 'step': args.step, 'speed': args.speed})
            print('记录 %d: right_move %d' % (len(actions), args.step), flush=True)
        elif ch == 'q':
            ik.turn_left(ik.initial_pos, 2, args.angle, args.speed, 1)
            actions.append({'index': len(actions) + 1, 'action': 'turn_left', 'angle': args.angle, 'speed': args.speed})
            print('记录 %d: turn_left %d' % (len(actions), args.angle), flush=True)
        elif ch == 'e':
            ik.turn_right(ik.initial_pos, 2, args.angle, args.speed, 1)
            actions.append({'index': len(actions) + 1, 'action': 'turn_right', 'angle': args.angle, 'speed': args.speed})
            print('记录 %d: turn_right %d' % (len(actions), args.angle), flush=True)
        elif ch == 'f':
            actions.append({'index': len(actions) + 1, 'action': 'pick'})
            print('记录 %d: 夹取' % len(actions), flush=True)
        elif ch == 'g':
            actions.append({'index': len(actions) + 1, 'action': 'place'})
            print('记录 %d: 放下' % len(actions), flush=True)
        elif ch == 'r':
            ik.stand(ik.initial_pos, t=500)
            actions.append({'index': len(actions) + 1, 'action': 'stand'})
            print('记录 %d: stand' % len(actions), flush=True)
        elif ch == 'u':
            if not actions:
                print('没有可以撤销的动作', flush=True)
            else:
                act = actions.pop()
                inverse_ik(ik, act)
                print('已撤销: %s，剩余 %d 个动作' % (act, len(actions)), flush=True)
        elif ch == 'c':
            actions.clear()
            ik.stand(ik.initial_pos, t=500)
            print('已清空记录，并恢复立正', flush=True)
        elif ch == 'l':
            print('--- 已记录动作 ---', flush=True)
            for i, act in enumerate(actions, 1):
                print('%d: %s' % (i, act), flush=True)
            print('------------------', flush=True)
        elif ch == 'x':
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
