#!/usr/bin/python3
# coding=utf8
"""官方 block_fetch.py 的色块定点夹取。

这里只保留官方 Demo 的夹取闭环，不再加入体态升降、超声波融合、
固定点兜底、重扫等二开逻辑：

1. 机械臂先回到官方检测位 (0, 15, 5)，俯仰角 -90°；
2. 连续多帧识别目标，中心点抖动小于 8px 且连续超过 10 帧；
3. 用 camera_to_world / pixel_to_arm_coord 换算机械臂坐标；
4. 执行官方 move()：到位 -> 夹爪闭合 -> 抬起 -> 放置 -> 松开 -> 回检测位。

供 tasks/auto_fetch.py 在搜索逼近完成后直接调用。
"""
import time

from agcs_lib.logs import get_logger, action_msg
from agcs_lib.sensors import show_status
from agcs_lib.vision import pixel_to_arm_coord


def official_color_grab(board, ak, params, detect, K, R, T,
                        display=None, min_area=500, attempts=3):
    """执行官方色块定点夹取。

    detect: 返回 dict(center=(cx, cy), area=...) 或 None 的检测函数。
    返回 True 表示已执行官方夹取动作并回到检测位。
    """
    arm = params['arm']
    walk = params.get('walk', {})
    detect_pose = tuple(float(v) for v in arm.get('detect_pose', [0, 15, 5]))
    pick_z = 5.0  # 官方 block_fetch.py 固定夹取高度
    raise_pose = (12.0, 24.0, 5.0)
    release_pose = (12.0, 24.0, -5.0)

    gripper_open = 120
    gripper_close = 550
    reach_x = float(walk.get('reach_x', 8.0))
    reach_y = float(walk.get('reach_y', 24.0))
    stable_need = 11   # 官方 num > 10
    stable_jitter = 8  # 官方 abs(center-old) < 8

    log = get_logger('grab_official')

    # 官方 init_move()：张开夹爪，并用逆运动学回到色块检测姿态。
    board.bus_servo_set_position(0.5, [[25, gripper_open]])
    ak.setPitchRangeMoving(detect_pose, -90, -90, 100, 2)
    time.sleep(2)

    for attempt in range(attempts):
        log.info('[grab-official] %s', action_msg(
            '官方色块定点夹取', action='第 %d/%d 次' % (attempt + 1, attempts)))
        if display is not None:
            show_status(display, 2)

        # 目标稳定判定，和官方 color_detect() 一致。
        num = 0
        old_x = old_y = 0
        last = None
        while num < stable_need:
            r = detect(min_area=min_area)
            if r is None:
                num = 0
                time.sleep(0.05)
                continue
            cx, cy = r['center']
            if abs(cx - old_x) < stable_jitter and abs(cy - old_y) < stable_jitter:
                num += 1
            else:
                num = 0
                old_x, old_y = cx, cy
            last = r
            time.sleep(0.05)

        if last is None:
            log.info('[grab-official] %s', action_msg(
                '未检测到稳定目标', reason='检测器连续返回空'))
            continue

        # 官方 camera_to_world 转机械臂坐标。pixel_to_arm_coord 内部会加上
        # initial_coord，因此这里传检测位 x/y，返回的就是机械臂绝对坐标。
        x, y = pixel_to_arm_coord(
            K, R, T, last['center'],
            initial_coord=(detect_pose[0], detect_pose[1]))
        log.info('[grab-official] %s', action_msg(
            '目标坐标', action='像素=%s x=%.1fcm y=%.1fcm z=%.1fcm'
            % (last['center'], x, y, pick_z)))

        # 官方原版的可及范围判断有 and/or 笔误，这里按正常工作空间判断，
        # 超出范围就放弃本次，不硬发逆运动学。
        if abs(x - detect_pose[0]) > reach_x or y > reach_y or y < 0:
            log.info('[grab-official] %s', action_msg(
                '超出机械臂可及范围',
                reason='x=%.1fcm y=%.1fcm，可及范围 |dx|<=%.1f 0<=y<=%.1f'
                % (x, y, reach_x, reach_y)))
            continue

        # 官方 move() 动作时序。
        board.bus_servo_set_position(0.5, [[25, gripper_open]])
        res = ak.setPitchRangeMoving((x, y, pick_z), -90, -90, 100, 1)
        if res is False:
            log.info('[grab-official] %s', action_msg(
                '夹取失败', reason='逆运动学无解'))
            continue
        time.sleep(3)

        board.bus_servo_set_position(0.5, [[25, gripper_close]])
        time.sleep(2)

        ak.setPitchRangeMoving(raise_pose, -90, -90, 100, 1.5)
        time.sleep(1.5)
        ak.setPitchRangeMoving(release_pose, -90, -90, 100, 1)
        time.sleep(1)

        board.bus_servo_set_position(0.5, [[25, gripper_open]])
        time.sleep(0.5)

        ak.setPitchRangeMoving(raise_pose, -90, -90, 100, 1)
        time.sleep(1)
        ak.setPitchRangeMoving(detect_pose, -90, -90, 100, 2)
        time.sleep(2)

        log.info('[grab-official] %s', action_msg(
            '官方夹取完成', reason='已按 block_fetch.py 动作时序执行'))
        if display is not None:
            show_status(display, 3)
        return True

    return False
