#!/usr/bin/python3
# coding=utf8
"""YOLO ONNX 检测器封装。接口与现有 functions.yolo_detect_onnx 一致。"""
import os
import sys

_SPIDERPI_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if _SPIDERPI_ROOT not in sys.path:
    sys.path.insert(0, _SPIDERPI_ROOT)

from functions.yolo_detect_onnx import YoloDetector

__all__ = ['YoloDetector']
