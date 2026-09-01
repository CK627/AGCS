#!/usr/bin/python3
# coding=utf8
"""固定夹取测试 CZ1：恢复官方初始位 -> 六足前进 -> 超声波 <1cm -> 固定夹取 -> 搬运放下。

流程：
    1. 恢复官方初始位置（立正 + 机械臂复位 + 云台回中 + 夹爪张开）；
    2. 六足持续前进，每走一步读一次超声波；
    3. 超声波读数 < 1cm 时停止前进（负数也视为到达）；
    4. 调用 agcs_lib.ClampRemoval.fixed_clamp()：
       21=510, 22=440, 23=415, 24=300，随后 25=700 夹紧；
    5. 六足后退 20cm，随后六足立正、21-24 恢复到官方初始位；
    6. 25 号夹爪保持 700，六足左转 90°；
    7. 六足前进 40cm；
    8. 机械臂 21-24 放到 500/365/105/280；
    9. 25 号夹爪松开，放下物体。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/FixedGripTest/CZ1.py
    python3 tasks/ceshi/FixedGripTest/CZ1.py --distance 1.0 --step 15 --speed 50

搬运到放置点后 25 松开。加 --hold 可让程序等回车再结束。
"""
import argparse
import os
import sys
import time

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    make_board,
    make_ik,
    make_arm_ik,
    load_params,
    make_ultrasonic,
    dist_cm,
    go_forward,
    go_back,
    turn_left,
    stand,
    fixed_clamp,
)
from agcs_lib.logs import setup_logger, action_msg
from agcs_lib.restore import Restore


def main():
    parser = argparse.ArgumentParser(description='固定夹取测试 CZ1')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='超声波触发夹取的距离阈值 cm，默认 1.0')
    parser.add_argument('--step', type=int, default=15,
                        help='每次六足前进步幅 mm，默认 15')
    parser.add_argument('--speed', type=int, default=50,
                        help='六足前进速度，默认 50')
    parser.add_argument('--max-steps', type=int, default=0,
                        help='最大前进步数，0=不限制')
    parser.add_argument('--back-step', type=int, default=200,
                        help='夹紧后六足后退总距离 mm，默认 200（20cm）')
    parser.add_argument('--back-speed', type=int, default=50,
                        help='夹紧后六足后退速度，默认 50')
    parser.add_argument('--turn-angle', type=int, default=90,
                        help='后退后向左转角度 deg，默认 90')
    parser.add_argument('--turn-speed', type=int, default=60,
                        help='左转速度，默认 60')
    parser.add_argument('--forward-distance', type=int, default=400,
                        help='左转后六足前进总距离 mm，默认 400（40cm）')
    parser.add_argument('--forward-step', type=int, default=50,
                        help='左转后每次六足前进步幅 mm，默认 50')
    parser.add_argument('--forward-speed', type=int, default=50,
                        help='左转后六足前进速度，默认 50')
    parser.add_argument('--hold', action='store_true',
                        help='搬运放下后等待回车结束')
    args = parser.parse_args()

    logger = setup_logger('CZ1')
    logger.info('[CZ1] %s', action_msg(
        '启动固定夹取测试',
        action='distance=%.1fcm step=%dmm speed=%d max_steps=%s back=%dmm '
               'turn=%ddeg forward=%dmm'
        % (args.distance, args.step, args.speed,
           args.max_steps or '无限', args.back_step,
           args.turn_angle, args.forward_distance)))

    params = load_params()
    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(board, params)
    restore = Restore(board, ik, ak, params)

    ultrasonic = make_ultrasonic()
    if ultrasonic is None:
        logger.error('[CZ1] %s', action_msg(
            '超声波初始化失败', reason='I2C 0x77 未连接或 sensor SDK 不可用'))
        return 1

    try:
        logger.info('[CZ1] %s', action_msg('恢复官方初始位置'))
        restore.restore_initial_state()

        steps = 0
        while True:
            distance = dist_cm(ultrasonic)

            if distance < args.distance:
                logger.info('[CZ1] %s', action_msg(
                    '超声波到位',
                    action='距离 %.2fcm < %.2fcm，停止六足前进（负数也视为到达）'
                    % (distance, args.distance)))
                break

            if args.max_steps and steps >= args.max_steps:
                logger.info('[CZ1] %s', action_msg(
                    '达到最大前进步数',
                    action='已走 %d 步，恢复初始位置' % steps))
                restore.restore_initial_state()
                return 3

            logger.info('[CZ1] %s', action_msg(
                '六足前进',
                action='第 %d 步，当前距离 %.2fcm，step=%dmm speed=%d'
                % (steps + 1, distance, args.step, args.speed)))
            go_forward(ik, step=args.step, speed=args.speed, times=1)
            steps += 1
            time.sleep(0.15)

        logger.info('[CZ1] %s', action_msg(
            '开始固定夹取',
            action='21=510 22=440 23=415 24=300，随后 25=700'))
        fixed_clamp(board)
        logger.info('[CZ1] %s', action_msg(
            '固定夹取完成',
            action='21-24 已到固定位，25=700 已夹紧'))

        logger.info('[CZ1] %s', action_msg(
            '开始退回',
            action='六足后退总距离 %dmm speed=%d，25 保持夹紧'
            % (args.back_step, args.back_speed)))
        back_remaining = args.back_step
        back_per_move = 50
        while back_remaining > 0:
            move = min(back_per_move, back_remaining)
            go_back(ik, step=move, speed=args.back_speed)
            back_remaining -= move
            time.sleep(0.15)

        # 恢复官方初始状态，但 25 号夹爪不松开。
        stand(ik, t=500)
        reset_pulses = params['arm']['reset_pulses']
        board.bus_servo_set_position(
            1.5, [[sid, reset_pulses[sid]] for sid in [21, 22, 23, 24]])
        time.sleep(1.5)
        board.bus_servo_set_position(0.5, [[25, 700]])
        time.sleep(0.5)
        logger.info('[CZ1] %s', action_msg(
            '后退并恢复其他关节',
            action='六足立正，21-24 回官方初始位，25=700 保持夹紧'))

        logger.info('[CZ1] %s', action_msg(
            '开始左转',
            action='向左转 %d 度，25 保持夹紧' % args.turn_angle))
        turn_left(ik, angle=args.turn_angle, speed=args.turn_speed)
        time.sleep(0.5)

        logger.info('[CZ1] %s', action_msg(
            '开始前进',
            action='六足前进总距离 %dmm，step=%dmm speed=%d，25 保持夹紧'
            % (args.forward_distance, args.forward_step, args.forward_speed)))
        forward_remaining = args.forward_distance
        while forward_remaining > 0:
            move = min(args.forward_step, forward_remaining)
            go_forward(ik, step=move, speed=args.forward_speed, times=1)
            forward_remaining -= move
            time.sleep(0.15)

        # 到达放置点后立正，机械臂放到放置位，最后 25 松开。
        stand(ik, t=500)
        place_pulses = {21: 500, 22: 365, 23: 105, 24: 280}
        board.bus_servo_set_position(
            1.5, [[sid, place_pulses[sid]] for sid in [21, 22, 23, 24]])
        time.sleep(1.5)
        open_pulse = int(params['arm'].get('gripper_open', 120))
        board.bus_servo_set_position(0.8, [[25, open_pulse]])
        time.sleep(0.8)
        logger.info('[CZ1] %s', action_msg(
            '放下物体',
            action='21-24 -> 500/365/105/280，25 -> %d 松开' % open_pulse))

        if args.hold:
            try:
                input('搬运并放下完成。按回车结束...')
            except EOFError:
                logger.info('[CZ1] 非交互运行，搬运放下完成后结束')

        return 0

    except KeyboardInterrupt:
        logger.info('[CZ1] %s', action_msg('用户中断', action='恢复官方初始位置'))
        restore.restore_initial_state()
        return 130
    except Exception as e:
        logger.error('[CZ1] %s', action_msg('异常退出', reason=str(e), action='尝试恢复官方初始位置'))
        try:
            restore.restore_initial_state()
        except Exception:
            logger.error('[CZ1] 恢复官方初始位置失败', exc_info=True)
        return 4


if __name__ == '__main__':
    sys.exit(main())
