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

    print('2. 六足前进 30cm（50mm x 6，速度50）', flush=True)
    ik.go_forward(ik.initial_pos, 2, 50, 50, 6)
    time.sleep(1)

    print('3. 固定夹取：21=510 22=455 23=415 24=300，然后 25=700', flush=True)
    board.bus_servo_set_position(1.2, [[21, 500], [22, 540], [23, 170], [24, 445]])
    time.sleep(1.2)
    board.bus_servo_set_position(0.8, [[25, 700]])
    time.sleep(0.8)

    print('已夹住：直接回车=继续搬运；输入 q 再回车=取消并恢复官方初始位置', flush=True)
    try:
        user_input = input()
    except EOFError:
        user_input = ''

    if user_input.strip().lower() == 'q':
        print('取消后续流程，恢复官方初始位置', flush=True)
        ik.stand(ik.initial_pos, t=500)
        board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
        board.bus_servo_set_position(1.0, [[25, 120]])
        time.sleep(1.5)
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        time.sleep(0.5)
        sys.exit(0)

    print('4. 六足后退 20cm（50mm x 4，速度50），25 保持夹紧', flush=True)
    ik.back(ik.initial_pos, 2, 50, 50, 4)
    time.sleep(1)

    print('5. 恢复六足和 21-24 官方初始位，25 继续保持 700', flush=True)
    ik.stand(ik.initial_pos, t=500)
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, 700]])
    time.sleep(0.5)

    print('6. 六足左转 90 度（30度 x 3，速度100）', flush=True)
    ik.turn_left(ik.initial_pos, 2, 30, 100, 3)
    time.sleep(1)

    print('7. 六足前进 40cm（50mm x 8，速度50）', flush=True)
    ik.go_forward(ik.initial_pos, 2, 50, 50, 8)
    time.sleep(1)

    print('8. 机械臂放下：21=500 22=365 23=105 24=280，然后 25=120', flush=True)
    ik.stand(ik.initial_pos, t=500)
    board.bus_servo_set_position(1.5, [[21, 500], [22, 365], [23, 105], [24, 280]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.8, [[25, 120]])
    time.sleep(0.8)

    print('完成', flush=True)
