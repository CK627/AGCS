#!/usr/bin/python3
# coding=utf8
"""固定夹取：按现场标定的固定舵机脉宽执行夹取，不做视觉/逆运动学。

机械臂 21-24 先到固定夹取位，25 号夹爪随后闭合。脉宽为现场手动标定值：
    21=510, 22=440, 23=415, 24=300, 25=700
"""
import time

from agcs_lib.logs import get_logger, action_msg


FIXED_ARM_PULSES = {21: 510, 22: 440, 23: 415, 24: 300}
GRIPPER_CLOSE_PULSE = 700


def fixed_clamp(board, pulses=None, gripper_pulse=GRIPPER_CLOSE_PULSE,
                arm_move_time=1.2, arm_settle=1.0,
                gripper_move_time=0.8, gripper_settle=0.8):
    """执行固定夹取：21-24 先到固定位，25 再闭合。

    board: 已初始化的控制板对象。
    pulses: 21-24 固定夹取位脉宽，默认使用 FIXED_ARM_PULSES。
    返回 (实际使用的 21-24 脉宽 dict, 实际使用的 25 脉宽)。
    """
    log = get_logger('ClampRemoval')
    pulses = {int(k): int(v) for k, v in (pulses or FIXED_ARM_PULSES).items()}
    missing = [sid for sid in [21, 22, 23, 24] if sid not in pulses]
    if missing:
        raise ValueError('固定夹取脉宽缺少舵机: %s' % missing)

    gripper_pulse = int(gripper_pulse)
    arm_order = [21, 22, 23, 24]

    board.bus_servo_set_position(
        arm_move_time, [[sid, pulses[sid]] for sid in arm_order])
    log.info('[ClampRemoval] %s', action_msg(
        '固定夹取位', action='21-24 -> %s' % [pulses[sid] for sid in arm_order]))
    time.sleep(arm_settle)

    board.bus_servo_set_position(gripper_move_time, [[25, gripper_pulse]])
    time.sleep(gripper_settle)
    log.info('[ClampRemoval] %s', action_msg(
        '夹爪闭合', action='25 -> %d' % gripper_pulse))

    return pulses, gripper_pulse


def clamp_removal(board, **kwargs):
    """fixed_clamp 的兼容别名。"""
    return fixed_clamp(board, **kwargs)
