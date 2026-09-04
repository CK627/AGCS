#!/usr/bin/python3
# coding=utf8
"""路线工具：查看、部分运行、从指定 index 重新录入。"""

import argparse
import json
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
import record_route as rr


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')


def load_route():
    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_route(actions):
    for i, act in enumerate(actions, 1):
        act['index'] = i
    with open(ROUTE_PATH, 'w', encoding='utf-8') as f:
        json.dump(actions, f, ensure_ascii=False, indent=2)


def run_action(ik, act):
    name = act.get('action')
    speed = int(act.get('speed', 30))
    if name == 'forward':
        ik.go_forward(ik.initial_pos, 2, int(act.get('step', 100)), speed, 1)
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
    elif name in ('pick', 'place'):
        print('遇到 %s，跳过机械臂动作' % name, flush=True)
    time.sleep(0.08)


def preview(actions, idx):
    i = idx - 1
    if i < 0 or i >= len(actions):
        print('index 超出范围', flush=True)
        return
    print('当前 %d/%d: %s' % (idx, len(actions), actions[i]), flush=True)
    if i + 1 < len(actions):
        print('下一步 %d/%d: %s' % (idx + 1, len(actions), actions[i + 1]), flush=True)
    else:
        print('没有下一步', flush=True)


def manual_record_tail(board, ik, prefix):
    print('从指定 index 开始重新录入。', flush=True)
    print('w/s/a/d/q/e 移动；f=夹取 g=放下 r=立正 u=撤销 c=清空 x=保存退出', flush=True)
    new_actions = []
    while True:
        ch = rr.getch().lower()
        if ch == 'w':
            ik.go_forward(ik.initial_pos, 2, 100, 30, 1)
            new_actions.append({'action': 'forward', 'step': 100, 'speed': 30})
        elif ch == 's':
            ik.back(ik.initial_pos, 2, 50, 30, 1)
            new_actions.append({'action': 'back', 'step': 50, 'speed': 30})
        elif ch == 'a':
            ik.left_move(ik.initial_pos, 2, 50, 30, 1)
            new_actions.append({'action': 'left_move', 'step': 50, 'speed': 30})
        elif ch == 'd':
            ik.right_move(ik.initial_pos, 2, 50, 30, 1)
            new_actions.append({'action': 'right_move', 'step': 50, 'speed': 30})
        elif ch == 'q':
            ik.turn_left(ik.initial_pos, 2, 10, 30, 1)
            new_actions.append({'action': 'turn_left', 'angle': 10, 'speed': 30})
        elif ch == 'e':
            ik.turn_right(ik.initial_pos, 2, 10, 30, 1)
            new_actions.append({'action': 'turn_right', 'angle': 10, 'speed': 30})
        elif ch == 'f':
            new_actions.append({'action': 'pick'})
        elif ch == 'g':
            new_actions.append({'action': 'place'})
        elif ch == 'r':
            ik.stand(ik.initial_pos, t=500)
            new_actions.append({'action': 'stand'})
        elif ch == 'u':
            if new_actions:
                rr.inverse_ik(ik, new_actions.pop())
            else:
                print('没有可撤销动作', flush=True)
        elif ch == 'c':
            new_actions.clear()
            ik.stand(ik.initial_pos, t=500)
        elif ch == 'x':
            result = prefix + new_actions
            save_route(result)
            print('已保存，总动作 %d' % len(result), flush=True)
            return
        time.sleep(0.08)


def main():
    parser = argparse.ArgumentParser(description='路线部分运行/重新录入')
    parser.add_argument('--preview', type=int, help='查看指定 index 及其下一步')
    parser.add_argument('--start', type=int, default=1, help='运行起始 index，1 开始')
    parser.add_argument('--end', type=int, default=None, help='运行结束 index，含')
    parser.add_argument('--record-from', type=int, help='从该 index 开始重新录入并替换后续')
    args = parser.parse_args()

    actions = load_route()

    if args.preview is not None:
        preview(actions, args.preview)
        return

    board = make_board()
    ik = make_ik(board)
    ik.stand(ik.initial_pos, t=500)

    if args.record_from is not None:
        idx = args.record_from - 1
        if idx < 0:
            idx = 0
        prefix = actions[:idx]
        if idx < len(actions):
            print('将从 index %d 开始替换，当前该步是：%s'
                  % (args.record_from, actions[idx]), flush=True)
        manual_record_tail(board, ik, prefix)
        return

    start = max(1, args.start) - 1
    end = len(actions) if args.end is None else min(len(actions), args.end)
    for idx in range(start, end):
        act = actions[idx]
        print('%d/%d %s' % (idx + 1, len(actions), act), flush=True)
        run_action(ik, act)


if __name__ == '__main__':
    main()
