#!/usr/bin/python3
# coding=utf8
"""深度摄像头抓取：靠近 + 机械臂夹取逻辑。"""

import math
import time


def should_approach(
    x_cm,
    y_cm,
    z_cm,
    reach_x=8.0,
    reach_y_min=6.0,
    reach_y_max=22.0,
    reach_z_max=25.0,
):
    """根据机械臂坐标判断目标是否可夹，返回需要的动作。"""
    if y_cm > reach_y_max:
        return 'forward'      # 太远，前进
    if y_cm < reach_y_min:
        return 'back'         # 太近，后退
    if abs(x_cm) > reach_x:
        return 'left' if x_cm > 0 else 'right'  # 横向修正
    if z_cm > reach_z_max:
        return 'too_high'     # 太高，够不到
    return 'ok'


def approach_loop(get_target, move, max_iters=200):
    """底盘靠近闭环，直到目标进入机械臂可及范围。

    get_target: 无参回调，返回 (x_mm, y_mm, z_mm, normal) 或 None。
    move: 动作回调，接收 'forward'/'back'/'left'/'right'/'too_high'/'search'。
    """
    for _ in range(max_iters):
        target = get_target()
        if target is None:
            move('search')
            continue

        x_mm, y_mm, z_mm, normal = target
        x_cm = x_mm / 10.0
        y_cm = y_mm / 10.0
        z_cm = z_mm / 10.0

        action = should_approach(x_cm, y_cm, z_cm)
        if action == 'ok':
            return x_cm, y_cm, z_cm, normal

        move(action)

    return None


def pitch_from_normal(normal):
    """把表面法向量简化成夹爪俯仰角 alpha。"""
    if normal is None:
        return -90.0

    _, ny, nz = normal
    return -90.0 + math.degrees(math.atan2(ny, -nz))


def arm_pick_by_depth(
    board,
    ak,
    x_cm,
    y_cm,
    z_cm,
    normal=None,
    approach_z_cm=8.0,
    gripper_open=120,
    gripper_close=550,
    raise_pose=(12.0, 24.0, 5.0),
    release_pose=(12.0, 24.0, -5.0),
):
    """机械臂到位、下降、夹取、抬起、松开。"""
    board.bus_servo_set_position(0.5, [[25, gripper_open]])
    time.sleep(0.5)

    alpha = pitch_from_normal(normal)

    res = ak.setPitchRangeMoving(
        (x_cm, y_cm, z_cm + approach_z_cm),
        alpha, -90, 100, 1.0,
    )
    time.sleep(1.0)
    if res is False:
        return False

    res = ak.setPitchRangeMoving(
        (x_cm, y_cm, z_cm + 1.0),
        alpha, -90, 100, 0.8,
    )
    time.sleep(0.8)
    if res is False:
        return False

    board.bus_servo_set_position(0.5, [[25, gripper_close]])
    time.sleep(0.8)

    ak.setPitchRangeMoving(raise_pose, -90, -90, 100, 1.0)
    time.sleep(1.0)

    ak.setPitchRangeMoving(release_pose, -90, -90, 100, 1.0)
    time.sleep(1.0)
    board.bus_servo_set_position(0.5, [[25, gripper_open]])
    time.sleep(0.5)

    return True


def depth_pick_loop(get_target, move, board, ak):
    """靠近 + 夹取的完整流程。"""
    result = approach_loop(get_target, move)
    if result is None:
        return False

    x_cm, y_cm, z_cm, normal = result
    return arm_pick_by_depth(board, ak, x_cm, y_cm, z_cm, normal)
