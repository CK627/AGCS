#!/usr/bin/python3
# coding=utf8
"""只重放完整路线，跳过 pick/place 标记，用于先测试完整一圈。"""

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
    route_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')
    with open(route_path, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)

    print('共 %d 个动作，开始完整路线测试（跳过 pick/place）' % len(actions), flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    for i, act in enumerate(actions, 1):
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
