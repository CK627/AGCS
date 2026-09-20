#!/usr/bin/python3
# coding=utf8
"""标定：22(肩)/23(肘) 脉宽 → 夹爪位置（离地高度 + 水平距离）。

跑法（先 sudo systemctl stop spiderpi）：
    python3 calib_reach.py

分两轮扫：
  A 轮：23 固定 500，22 从 350 扫到 650；
  B 轮：22 固定 400，23 从 350 扫到 550。

每档停 8 秒，用尺子量两个数，记成 (脉宽, 离地高度cm, 水平距离cm)：
  离地高度 = 夹爪最低点到地面的垂直距离；
  水平距离 = 机器人中心到夹爪正下方地面点的水平距离。

两轮量完，把表发给 Claude，拟合 (22,23) → (高度, 水平) 的映射，反推夹取该用的脉宽。
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
        print('\n>>> 22=%d, 23=500   （量离地高度 + 水平距离）' % p, flush=True)
        time.sleep(8)

    print('\n===== B 轮：22=400 固定，扫 23（肘） =====', flush=True)
    for p in SWEEP_23:
        board.bus_servo_set_position(1.2, [[22, 400], [23, p]])
        time.sleep(1.4)
        print('\n>>> 22=400, 23=%d   （量离地高度 + 水平距离）' % p, flush=True)
        time.sleep(8)

    # 复位
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    time.sleep(1.6)
    print('\n标定结束，把 (脉宽, 离地高度, 水平距离) 表发给 Claude', flush=True)


if __name__ == '__main__':
    main()
