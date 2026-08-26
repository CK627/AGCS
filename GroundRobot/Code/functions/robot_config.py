#!/usr/bin/python3
# coding=utf8
"""兼容层：re-export agcs_lib（旧代码 from functions.robot_config import ... 仍可用）。"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.params import load_params
from agcs_lib.arm import make_arm_ik

__all__ = ['load_params', 'make_arm_ik']
