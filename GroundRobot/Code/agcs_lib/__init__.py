#!/usr/bin/python3
# coding=utf8
"""AGCS 地面机器人自研封装库。

把官方 SDK（common / calibration / arm_ik / sensor）封装成我们自己的接口：
业务脚本（tasks/）只 import 本库，不直接 import 官方 SDK。
"""

from agcs_lib.params import load_params
from agcs_lib.hardware import make_board
from agcs_lib.arm import make_arm_ik
from agcs_lib.motion import make_ik, stand, move_body, turn_left, turn_right, go_forward, go_back
from agcs_lib.sensors import make_ultrasonic, dist_cm, make_display, show_status
from agcs_lib.camera import open_camera, capture
from agcs_lib.vision import (
    load_lab_data,
    load_block_params,
    detect_color,
    pixel_to_arm_coord,
    load_undistort_maps,
    correct_camera,
)
from agcs_lib.orientation import block_orientation, orientation_error
