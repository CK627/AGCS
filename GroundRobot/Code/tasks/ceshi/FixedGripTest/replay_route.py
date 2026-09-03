#!/usr/bin/python3
# coding=utf8
"""按原顺序重放 record_route.py 录制的 fixed_route.json。"""

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


def main():
    parser = argparse.ArgumentParser(description='重放录制的固定路线')
    parser.add_argument('--route', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json'),
        help='record_route.py 生成的路线文件')
    args = parser.parse_args()

    with open(args.route, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)

    print('共 %d 个动作，开始按原顺序重放' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions, 1):
        name = act.get('action')
        speed = int(act.get('speed', 50))

        if name == 'forward':
            step = int(act.get('step', 50))
            print('%d/%d forward %d' % (i, len(actions), step), flush=True)
            ik.go_forward(ik.initial_pos, 2, step, speed, 1)
        elif name == 'back':
            step = int(act.get('step', 50))
            print('%d/%d back %d' % (i, len(actions), step), flush=True)
            ik.back(ik.initial_pos, 2, step, speed, 1)
        elif name == 'turn_left':
            angle = int(act.get('angle', 30))
            print('%d/%d turn_left %d' % (i, len(actions), angle), flush=True)
            ik.turn_left(ik.initial_pos, 2, angle, speed, 1)
        elif name == 'turn_right':
            angle = int(act.get('angle', 30))
            print('%d/%d turn_right %d' % (i, len(actions), angle), flush=True)
            ik.turn_right(ik.initial_pos, 2, angle, speed, 1)
        elif name == 'left_move':
            step = int(act.get('step', 50))
            print('%d/%d left_move %d' % (i, len(actions), step), flush=True)
            ik.left_move(ik.initial_pos, 2, step, speed, 1)
        elif name == 'right_move':
            step = int(act.get('step', 50))
            print('%d/%d right_move %d' % (i, len(actions), step), flush=True)
            ik.right_move(ik.initial_pos, 2, step, speed, 1)
        elif name == 'stand':
            print('%d/%d stand' % (i, len(actions)), flush=True)
            ik.stand(ik.initial_pos, t=500)
        else:
            print('跳过未知动作: %s' % act, flush=True)

        time.sleep(0.08)

    ik.stand(ik.initial_pos, t=500)
    print('路线重放完成', flush=True)


if __name__ == '__main__':
    main()
