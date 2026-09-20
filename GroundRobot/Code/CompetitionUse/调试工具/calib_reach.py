#!/usr/bin/python3
# coding=utf8
"""标定：22(肩)/23(肘) 脉宽 → 夹爪位置（离地高度 + 水平距离），交互式输入。

跑法（先 sudo systemctl stop spiderpi）：
    python3 calib_reach.py

脚本把舵机移到指定位置，提示你输入「离地高度 水平距离」(cm)，空格隔开：
  离地高度 = 夹爪最低点到地面的垂直距离；
  水平距离 = 机器人中心到夹爪正下方地面点的水平距离。

输完回车，脚本记录并移到下一个位置。全部扫完打印数据表并存到 calib_data.txt。
"""
import time
from common.ros_robot_controller_sdk import Board

# 扫描的 (22, 23) 组合：先 23=500 扫 22，再 22=400 扫 23（去重）
SWEEP = [
    (350, 500), (400, 500), (450, 500), (500, 500), (550, 500), (600, 500), (650, 500),
    (400, 350), (400, 400), (400, 450), (400, 550),
]


def main():
    board = Board()
    # 固定 21 朝前、24 朝下、25 夹爪张开
    board.bus_servo_set_position(1.5, [[21, 500], [24, 250], [25, 120]])
    time.sleep(1.6)

    data = []
    print('开始标定。每个位置输入「离地高度 水平距离」(cm)，空格隔开，直接回车跳过。', flush=True)
    for (p22, p23) in SWEEP:
        board.bus_servo_set_position(1.2, [[22, p22], [23, p23]])
        time.sleep(1.4)
        print('\n>>> 22=%d, 23=%d' % (p22, p23), flush=True)
        s = input('  离地高度(cm) 水平距离(cm)：').strip()
        h = d = None
        if s:
            parts = s.split()
            if len(parts) >= 2:
                try:
                    h = float(parts[0])
                    d = float(parts[1])
                except ValueError:
                    pass
        data.append((p22, p23, h, d))

    # 复位
    board.bus_servo_set_position(1.5, [[21, 500], [22, 705], [23, 90], [24, 330]])
    time.sleep(1.6)

    # 打印数据表 + 存文件
    print('\n===== 收集到的数据 =====', flush=True)
    lines = ['22,23,离地高度cm,水平距离cm']
    for (p22, p23, h, d) in data:
        hs = ('%.1f' % h) if h is not None else ''
        ds = ('%.1f' % d) if d is not None else ''
        print('22=%d, 23=%d → 高度 %s, 水平 %s' % (p22, p23, hs, ds), flush=True)
        lines.append('%d,%d,%s,%s' % (p22, p23, hs, ds))

    with open('calib_data.txt', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n数据已存到 calib_data.txt', flush=True)


if __name__ == '__main__':
    main()
