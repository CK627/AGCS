#!/usr/bin/python3
# coding=utf8
"""CZ1 固定夹取搬运测试：纯硬编码直线流程。"""

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


if __name__ == '__main__':
    board = make_board()
    ik = make_ik(board)

    print('1. 恢复官方初始位置', flush=True)
    ik.stand(ik.initial_pos, t=500)
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    board.bus_servo_set_position(1.0, [[25, 120]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
    time.sleep(0.5)

    print('2. 六足前进（20 x 50，速度5）', flush=True)
    ik.go_forward(ik.initial_pos, 2, 20, 5, 50)
    time.sleep(1)

    print('3. 固定夹取：21=510 22=440 23=415 24=300，然后 25=700', flush=True)
    board.bus_servo_set_position(1.2, [[21, 510], [22, 440], [23, 415], [24, 300]])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[25, 700]])
    time.sleep(0.8)

    print('4. 六足后退（20 x 50，速度5），25 保持夹紧', flush=True)
    ik.back(ik.initial_pos, 2, 20, 5, 50)
    time.sleep(1)

    print('5. 恢复六足和 21-24 官方初始位，25 继续保持 700', flush=True)
    ik.stand(ik.initial_pos, t=500)
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, 700]])
    time.sleep(0.5)

    print('6. 六足左转 90 度（30度 x 3）', flush=True)
    ik.turn_left(ik.initial_pos, 2, 30, 30, 3)
    time.sleep(1)

    print('7. 六足前进（20 x 50，速度5）', flush=True)
    ik.go_forward(ik.initial_pos, 2, 20, 5, 50)
    time.sleep(1)

    print('8. 机械臂放下：21=500 22=365 23=105 24=280，然后 25=120', flush=True)
    ik.stand(ik.initial_pos, t=500)
    board.bus_servo_set_position(1.5, [[21, 500], [22, 365], [23, 105], [24, 280]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.8, [[25, 120]])
    time.sleep(0.8)

    print('完成', flush=True)
