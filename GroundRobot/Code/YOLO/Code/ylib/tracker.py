#!/usr/bin/python3
# coding=utf8
"""云台跟踪：复用 agcs_lib.tracker.ColorTracker，对 YOLO detect 结果做 PID 锁定。"""
import os
import sys

_SPIDERPI_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if _SPIDERPI_ROOT not in sys.path:
    sys.path.insert(0, _SPIDERPI_ROOT)

from agcs_lib.tracker import ColorTracker

__all__ = ['ColorTracker']
