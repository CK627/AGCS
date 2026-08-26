#!/usr/bin/python3
# coding=utf8
"""方块朝向角检测：用于夹取前的对齐（独立模块，按需复用）。

block_orientation：估算方形目标（俯视为正方形）的朝向角，范围 [0, 90) 度。
orientation_error：朝向角误差，周期 90 度的差，返回 [-45, 45) 度。
"""
import math

import cv2
import numpy as np


def block_orientation(contour, eps_factor=0.035):
    """估算方形目标的朝向角，范围 [0, 90) 度；返回 (angle, pts)，无效返回 (None, None)。"""
    if contour is None or len(contour) < 4:
        return None, None
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return None, None
    approx = cv2.approxPolyDP(contour, eps_factor * perimeter, True)
    pts = approx.reshape(-1, 2).astype(np.float64)
    if len(pts) < 4:
        return None, None

    edge_angles = []
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
        edge_angles.append(ang % 90.0)

    phases = np.deg2rad(np.asarray(edge_angles, dtype=np.float64) * 4.0)
    mean4 = math.degrees(math.atan2(float(np.sin(phases).sum()),
                                    float(np.cos(phases).sum())))
    return (mean4 / 4.0) % 90.0, pts


def orientation_error(angle, ref_angle=0.0):
    """朝向角误差：周期 90 度的差，返回 [-45, 45) 度。"""
    d = (angle - ref_angle) % 90.0
    if d > 45.0:
        d -= 90.0
    return d
