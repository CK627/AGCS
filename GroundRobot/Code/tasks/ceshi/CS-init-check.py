#!/usr/bin/python3
# coding=utf8
"""开机初始化分步检查：1) 恢复初始状态  2) 摄像头启动。

流程（一步一检，能行就添上，不行就删掉重来）：
    1) 恢复初始状态：立正 + 机械臂复位(reset_pulses) + 云台回中(21=500/24=260) + 夹爪张开
    2) 摄像头启动：打开相机 → 取帧 → 畸变校正 → 关闭
每一步独立记录 PASS/FAIL，任一步失败立即退出。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/CS-init-check.py
"""
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2

from agcs_lib import (
    make_board, make_ik, make_arm_ik, load_params, stand,
    correct_camera, load_undistort_maps, open_camera, capture,
)
from agcs_lib.logs import setup_logger


def main():
    logger = setup_logger('CS-init-check')
    logger.info('初始化分步检查开始')
    params = load_params()

    # ---------------- 第 1 步：恢复初始状态 ----------------
    try:
        board = make_board()
        ik = make_ik(board)
        ak = make_arm_ik(params)

        stand(ik, t=500)                       # 六足立正
        arm_pulses = params['arm']['reset_pulses']
        board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
        board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
        time.sleep(1.5)
        board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])   # 云台回中
        time.sleep(0.5)
        logger.info('[1] PASS 恢复初始状态：立正 + 机械臂复位 + 云台回中 + 夹爪张开')
    except Exception as e:
        logger.error('[1] FAIL 恢复初始状态: %s', e)
        return 1

    # ---------------- 第 2 步：摄像头启动 ----------------
    try:
        rotate = params['vision'].get('camera_rotate', 0)
        mapx, mapy = load_undistort_maps()
        cam = open_camera()
        f = capture(cam, tries=10)
        if f is None:
            cam.camera_close()
            logger.error('[2] FAIL 摄像头启动：取不到帧（可能被其他进程占用，检查 spiderpi/CS-video）')
            return 1
        frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        cam.camera_close()
        logger.info('[2] PASS 摄像头启动：取帧 OK，shape=%s', frame.shape)
    except Exception as e:
        logger.error('[2] FAIL 摄像头启动: %s', e)
        return 1

    logger.info('初始化检查全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
