#!/usr/bin/python3
# coding=utf8
"""标定：22(肩)/23(肘) 脉宽 → 夹爪离地高度(cm)。

跑法（先 sudo systemctl stop spiderpi）：
    python3 calib_reach.py

分两轮扫：
  A 轮：23 固定 500（肘伸到夹取位），22 从 350 扫到 650；
  B 轮：22 固定 400（肩降到夹取位），23 从 350 扫到 550。

每档停 6 秒，你用尺子量「夹爪离地高度」(cm)，记成 (脉宽, 高度)。
两轮都量完，把表发给 Claude 拟合出 (22,23) → 离地高度 的映射。
"""
import time
from common.ros_robot_controller_sdk import Board

SWEEP_22 = [350, 400, 450, 500, 550, 600, 650]   # A 轮：22 不同脉宽
SWEEP_23 = [350, 400, 450, 500, 550]             # B 轮：23 不同脉宽


def main():
    board = Board()
    # 固定 21 朝前、24 朝下、25 夹爪张开
    board.bus_servo_set_position(1.5, [[21, 500], [24, 250], [25, 120]])
    time.sleep(1.6)

    print('===== A 轮：23=500 固定，扫 22（肩） =====', flush=True)
    for p in SWEEP_22:
        board.bus_servo_set_position(1.2, [[23, 500], [22, p]])
        time.sleep(1.4)
        print('\n>>> 22=%d, 23=500   （量夹爪离地高度 cm）' % p, flush=True)
        time.sleep(6)

    print('\n===== B 轮：22=400 固定，扫 23（肘） =====', flush=True)
    for p in SWEEP_23:
        board.bus_servo_set_position(1.2, [[22, 400], [23, p]])
        time.sleep(1.4)
        print('\n>>> 22=400, 23=%d   （量夹爪离地高度 cm）' % p, flush=True)
        time.sleep(6)

    # 复位
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    time.sleep(1.6)
    print('\n标定结束，把 (脉宽, 高度) 表发给 Claude', flush=True)


if __name__ == '__main__':
    main()
