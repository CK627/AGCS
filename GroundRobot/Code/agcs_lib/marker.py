#!/usr/bin/python3
# coding=utf8
"""ArUco 标记检测 + 位姿估计（OpenCV 内置，无额外依赖）。

用于 S4 走固定路线的视觉定位：彩色相机（OpenCV /dev/video0）识别标记，
估算标记相对相机的位姿。

注意：当前用近似内参（沿用深度内参 570.34/320/240），精确位姿需先标定
彩色相机。tvec 为 OpenCV 相机坐标系（X 右/Y 下/Z 前，单位 mm）。
"""
import cv2
import numpy as np

DEFAULT_FX, DEFAULT_FY = 570.34, 570.34
DEFAULT_CX, DEFAULT_CY = 320.0, 240.0


class MarkerDetector:
    def __init__(self, marker_size_mm=100.0, fx=DEFAULT_FX, fy=DEFAULT_FY,
                 cx=DEFAULT_CX, cy=DEFAULT_CY, dict_id=cv2.aruco.DICT_4X4_50):
        self.marker_size_mm = float(marker_size_mm)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        self.camera_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float64)

    def detect(self, bgr):
        """检测标记，返回 list[dict(id, corners, rvec, tvec)]。

        tvec 单位 mm（相机坐标系：X 右 / Y 下 / Z 前，Z 即距离）。
        """
        corners, ids, _ = self.detector.detectMarkers(bgr)
        if ids is None or len(ids) == 0:
            return []
        # 标记四角在标记坐标系下的 3D 坐标（匹配 detectMarkers 的角点顺序）
        s = self.marker_size_mm / 2.0
        objp = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                        dtype=np.float64)
        out = []
        for i, mid in enumerate(ids.flatten()):
            ok, rvec, tvec = cv2.solvePnP(
                objp, corners[i][0], self.camera_matrix, self.dist_coeffs)
            if not ok:
                continue
            out.append({
                "id": int(mid),
                "corners": corners[i][0],
                "rvec": rvec.ravel(),
                "tvec": tvec.ravel(),
            })
        return out

    def center(self, mark):
        """标记中心的像素坐标。"""
        return mark["corners"].mean(axis=0)
