#!/usr/bin/python3
# coding=utf8
"""IMU 遥控行走：WASD 前后左右，Q/E 每次精确转 10°（IMU 闭环）。

比 manual_control_capture.py 的开环转身准：转身不用固定步数硬转，而是用 IMU 积分
航向闭环转到目标角——每次按下 q/e 都以「当前实际朝向 ± 10°」为目标，转多了/转少了
自动补步，误差 <=1° 才停，所以连续按 q 十次不会越转越偏（开环会，六足每一步都有
转角偏差，累积起来一圈能差十几度）。

用法（先 sudo systemctl stop spiderpi，串口 /dev/ttyAMA0 同一时刻只能一个进程占）：
    python3 manual_control_imu.py
    python3 manual_control_imu.py --step 40 --angle 10

按键：
    w 前进   s 后退   a 左横移   d 右横移
    q 左转 --angle°   e 右转 --angle°
    r 立正   y 航向清零   x 退出
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (ImuYaw, make_board, make_ik, stand,
                      turn_left, turn_right, go_forward, go_back)

POLL_S = 0.02  # 主循环节奏，约 50Hz，保证 yaw 积分不漏


def getch_timeout(timeout=POLL_S):
    """带超时地读一个键；超时返回 None（不阻塞，好让 IMU 持续积分）。"""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    return sys.stdin.read(1)


def turn_by(ik, imu, delta_deg, tol=1.0, timeout=8.0, speed=60):
    """IMU 闭环旋转 delta_deg（正=左转，负=右转）。返回 (是否到位, 当前 yaw)。

    目标从「当前实际朝向」算起，所以每次按下都实实在在转 delta_deg，
    不受之前累计误差影响；单步限幅 5°，避免一次转过头。
    """
    target = imu.update() + delta_deg
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        yaw = imu.update()
        err = target - yaw
        if abs(err) <= tol:
            return True, yaw
        step = max(1, min(5, int(round(abs(err)))))
        if err > 0:
            turn_left(ik, angle=step, speed=speed)
        else:
            turn_right(ik, angle=step, speed=speed)
        time.sleep(0.15)
    return False, imu.update()


def main():
    parser = argparse.ArgumentParser(description='IMU 遥控行走（Q/E 每次精确转 10°）')
    parser.add_argument('--step', type=int, default=40, help='直行/横移一步的距离 mm')
    parser.add_argument('--angle', type=float, default=10.0, help='q/e 每次转角 deg')
    parser.add_argument('--speed', type=int, default=50, help='行走速度')
    parser.add_argument('--turn-speed', type=int, default=60, help='转身速度')
    parser.add_argument('--tol', type=float, default=1.0, help='转角容差 deg')
    args = parser.parse_args()

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)  # 等站稳再标定零漂

    imu = ImuYaw(board)
    n = imu.calibrate()
    if n == 0:
        print('IMU 读不到数据：确认已 sudo systemctl stop spiderpi（串口被占），'
              '且板子已 enable_reception', flush=True)
        return 1
    print('IMU 零漂标定完成（%d 样本，bias=%.4f）' % (n, imu.bias), flush=True)

    print('=== IMU 遥控模式 ===', flush=True)
    print('w前进 s后退 a左横移 d右横移  q左转%.0f° e右转%.0f°  r立正 y航向清零 x退出'
          % (args.angle, args.angle), flush=True)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            imu.update()  # 不按键时也持续积分，yaw 始终跟得上真实朝向
            ch = getch_timeout()
            if ch is None:
                continue
            ch = ch.lower()

            if ch == 'w':
                go_forward(ik, step=args.step, speed=args.speed, times=1)
                print('前进 %dmm' % args.step, flush=True)
            elif ch == 's':
                go_back(ik, step=args.step, speed=args.speed)
                print('后退 %dmm' % args.step, flush=True)
            elif ch == 'a':
                ik.left_move(ik.initial_pos, 2, args.step, args.speed, 1)
                print('左横移 %dmm' % args.step, flush=True)
            elif ch == 'd':
                ik.right_move(ik.initial_pos, 2, args.step, args.speed, 1)
                print('右横移 %dmm' % args.step, flush=True)
            elif ch in ('q', 'e'):
                delta = args.angle if ch == 'q' else -args.angle
                ok, yaw = turn_by(ik, imu, delta, tol=args.tol, speed=args.turn_speed)
                print('%s转 %.0f° %s（yaw=%.1f°）'
                      % ('左' if ch == 'q' else '右', abs(delta),
                         '已到位' if ok else '超时未到位', yaw), flush=True)
            elif ch == 'r':
                stand(ik)
                print('立正', flush=True)
            elif ch == 'y':
                imu.reset()
                print('航向已清零', flush=True)
            elif ch in ('x', '\x03'):
                print('退出', flush=True)
                break
            else:
                print('未识别: %r' % ch, flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        stand(ik)
    print('结束，最终 yaw=%.1f°' % imu.yaw, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
