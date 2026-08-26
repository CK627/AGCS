#!/usr/bin/python3
# coding=utf8
"""自动取物编排：搜索(search) → 定位判定(bgas) → 纯机械臂夹取(grab)，三个阶段严格分离。

- search：找目标 + 大致接近（只动 1-20 + 云台 21/24）；
- bgas（Before-Grab-After-Search）：判定/调整到"可抓取"状态（只动 1-20，云台锁定检测位）；
- grab：只动 21-25 伸臂夹取，绝不动 1-20。
"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def main():
    import cv2
    from agcs_lib import (
        make_board, make_ik, make_arm_ik, load_params,
        load_lab_data, load_block_params, detect_color, correct_camera,
        load_undistort_maps, make_ultrasonic, make_display, open_camera,
        capture, stand,
    )
    from agcs_lib.search import Searcher
    from agcs_lib.bgas import Bgas
    from agcs_lib.grab import Grabber
    from agcs_lib.sensors import show_status

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    args = parser.parse_args()
    color = args.color

    from agcs_lib.logs import setup_logger, action_msg
    logger = setup_logger()
    logger.info('启动 auto_fetch：color=%s', color)

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    cam = open_camera()

    def detect(min_area=300):
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, color, min_area=min_area)

    stand(ik)
    # 机械臂复位（21/22/23/24 回 reset_pulses，夹爪张开）
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(2)

    # 1) 搜索
    searcher = Searcher(board, ik, ak, params, detect, ultrasonic, display)
    center, cy = searcher.run()
    if center is None:
        logger.info('[search] %s', action_msg('未找到目标', reason='颜色=%s' % color))
        cam.camera_close()
        show_status(display, 0)
        return

    # 2) bgas：定位/可抓取判定（只动 1-20）
    import agcs_lib.bgas as _b
    logger.debug('[VER] BGAS_FILE=%s' % _b.__file__)
    bgas = Bgas(board, ik, ak, params, K, R, T, detect, display, ultrasonic=ultrasonic)
    context, reason = bgas.run(cy=cy, x_dis=searcher.x_dis, y_dis=searcher.y_dis)
    if context is None:
        logger.info('[bgas] %s', action_msg('定位失败', reason=reason))
        cam.camera_close()
        show_status(display, 0)
        return

    # 3) grab：纯机械臂夹取（只动 21-25）
    import agcs_lib.grab as _g
    logger.debug('[VER] GRAB_FILE=%s BGAS_OK=1' % _g.__file__)
    grabber = Grabber(board, ik, ak, params, K, R, T, detect, display, ultrasonic=ultrasonic)
    ok = grabber.run(context=context, body_z=context['body_z'])

    cam.camera_close()
    if ok:
        logger.info('[grab] %s', action_msg('完成', action='%s 已夹取并放下' % color))
        show_status(display, 3)
        time.sleep(5)
        show_status(display, 0)
    else:
        logger.info('[grab] %s', action_msg('夹取失败', reason='颜色=%s' % color))
        show_status(display, 0)


if __name__ == '__main__':
    main()
