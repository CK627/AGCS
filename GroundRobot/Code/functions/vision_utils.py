#!/usr/bin/python3
# coding=utf8
"""兼容层：re-export agcs_lib.vision（旧代码 from functions.vision_utils import ... 仍可用）。"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.vision import (
    load_lab_data,
    load_block_params,
    get_area_max_contour,
    detect_color,
    block_orientation,
    orientation_error,
    camera_to_world,
    pixel_to_arm_coord,
    load_undistort_maps,
    correct_camera,
    range_rgb,
)

__all__ = [
    'load_lab_data', 'load_block_params', 'get_area_max_contour',
    'detect_color', 'block_orientation', 'orientation_error',
    'camera_to_world', 'pixel_to_arm_coord', 'load_undistort_maps',
    'correct_camera', 'range_rgb',
]
