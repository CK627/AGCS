#!/usr/bin/python3
# coding=utf8
"""参数加载：封装 config/robot_params.yaml。"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import yaml

_DEFAULT_PATH = os.path.join(_PKG_ROOT, 'config', 'robot_params.yaml')


def load_params(path=_DEFAULT_PATH):
    """读取机器人行为参数，返回 dict。"""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)
