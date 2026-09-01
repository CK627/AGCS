#!/usr/bin/python3
# coding=utf8
"""固定夹取测试 CZ1：恢复官方初始位 -> 六足前进 -> 超声波 <1cm -> 固定夹取。

流程：
    1. 恢复官方初始位置（立正 + 机械臂复位 + 云台回中 + 夹爪张开）；
    2. 六足持续前进，每走一步读一次超声波；
    3. 超声波检测到距离 < 1cm 时停止前进；
    4. 调用 agcs_lib.ClampRemoval.fixed_clamp()：
       21=510, 22=440, 23=415, 24=300，随后 25=700 夹紧。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/FixedGripTest/CZ1.py
    python3 tasks/ceshi/FixedGripTest/CZ1.py --distance 1.0 --step 15 --speed 50

夹取完成后默认保持夹紧。若要观察后再恢复官方初始位置，加 --hold，
程序会等回车后松开夹爪并复位。
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
    parser.add_argument('--invalid-limit', type=int, default=20,
                        help='超声波连续读数无效多少次后停止，默认 20')
    parser.add_argument('--hold', action='store_true',
                        help='夹取成功后等待回车，再恢复官方初始位置')
    args = parser.parse_args()

    logger = setup_logger('CZ1')
    logger.info('[CZ1] %s', action_msg(
        '启动固定夹取测试',
        action='distance=%.1fcm step=%dmm speed=%d max_steps=%s'
        % (args.distance, args.step, args.speed, args.max_steps or '无限')))

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
        invalid_streak = 0
        while True:
            distance = dist_cm(ultrasonic)

            if 0 < distance < args.distance:
                logger.info('[CZ1] %s', action_msg(
                    '超声波到位',
                    action='距离 %.2fcm < %.2fcm，停止六足前进'
                    % (distance, args.distance)))
                break

            if distance < 0:
                invalid_streak += 1
                logger.info('[CZ1] %s', action_msg(
                    '超声波读数无效',
                    reason='连续 %d 次，超过 %d 次将停止'
                    % (invalid_streak, args.invalid_limit)))
                if invalid_streak >= args.invalid_limit:
                    logger.error('[CZ1] %s', action_msg(
                        '超声波持续无效', action='停止前进并恢复初始位置'))
                    restore.restore_initial_state()
                    return 2
            else:
                invalid_streak = 0

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

        if args.hold:
            try:
                input('固定夹取完成，保持夹紧。按回车松开夹爪并恢复官方初始位置...')
            except EOFError:
                logger.info('[CZ1] 非交互运行，保持夹紧并结束')
                return 0
            restore.restore_initial_state()
            logger.info('[CZ1] %s', action_msg('已恢复官方初始位置'))

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
