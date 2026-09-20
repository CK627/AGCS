#!/usr/bin/python3
# coding=utf8
"""标定：22 号（肩）脉宽 → 夹爪到机器人中心的水平距离(cm)。

跑法（先 sudo systemctl stop spiderpi）：
    python3 calib_reach.py

脚本把 22 号（肩）依次移到不同脉宽，每档停 6 秒。你用尺子量「机器人中心到夹爪
正下方地面点」的水平距离(cm)，记成 (脉宽, 距离) 表。跑完把表发给 Claude，拟合出
映射，就能反推「要够到目标需要 22 多少脉宽」。

其它舵机固定：21=500（朝前）、23=500（肘伸到夹取位）、24=250（腕朝下）。
"""
import time
from common.ros_robot_controller_sdk import Board

PULSES_22 = [350, 400, 450, 500, 550, 600, 650, 705]  # 22 号不同脉宽（肩高度）


def main():
    board = Board()
    # 固定 21 朝前、23 肘伸到夹取位、24 朝下、25 夹爪张开
    board.bus_servo_set_position(1.5, [[21, 500], [23, 500], [24, 250], [25, 120]])
    time.sleep(1.6)

    print('开始标定。每档停 6 秒，量「机器人中心到夹爪正下方」的水平距离(cm)。', flush=True)
    for p in PULSES_22:
        board.bus_servo_set_position(1.2, [[22, p]])
        time.sleep(1.4)
        print('\n>>> 22 号脉宽 = %d   （量水平距离 cm）' % p, flush=True)
        time.sleep(6)

    # 复位
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    time.sleep(1.6)
    print('\n标定结束，把 (脉宽, 距离) 表发给 Claude', flush=True)


if __name__ == '__main__':
    main()
