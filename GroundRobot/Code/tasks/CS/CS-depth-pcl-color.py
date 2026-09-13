#!/usr/bin/python3
# coding=utf8
"""彩色点云单测：深度点云 + 彩色纹理，存彩色 .ply。

深度走 OpenNI2，彩色走 OpenCV /dev/video0，同像素粗对齐（深度/彩色
约 25mm 基线暂忽略，精对齐留给 SLAM 阶段）。生成的文件可用 MeshLab /
CloudCompare / Open3D 打开。

用法：
    python3 CS-depth-pcl-color.py
"""
import os
import sys

import cv2
import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.depth import DepthCamera


def save_ply_ascii_color(path, pts, rgb):
    """把 (N,3) 世界坐标 + (N,3) RGB 存成 ASCII 彩色 PLY。"""
    n = len(pts)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write("element vertex %d\n" % n)
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(n):
            p = pts[i]
            c = rgb[i]
            f.write("%.2f %.2f %.2f %d %d %d\n"
                    % (p[0], p[1], p[2], c[0], c[1], c[2]))


def main():
    cam = DepthCamera()
    cam.open()
    cam.start_depth()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    try:
        # 预热，让深度/彩色都稳定
        for _ in range(5):
            cam.read_depth(timeout_ms=2000)
            cap.read()

        d = cam.read_depth(timeout_ms=2000)
        ok, bgr = cap.read()
        if d is None or not ok:
            print("FAIL: 读深度或彩色失败")
            return

        pcl = cam.depth_to_pointcloud(d)
        valid = ~np.isnan(pcl[:, :, 0])
        pts = pcl[valid].reshape(-1, 3)
        print("几何点云: %d 点, X %.0f~%.0f, Y %.0f~%.0f, Z %.0f~%.0f mm" % (
            len(pts), pts[:, 0].min(), pts[:, 0].max(),
            pts[:, 1].min(), pts[:, 1].max(),
            pts[:, 2].min(), pts[:, 2].max()))

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        colors = rgb[valid].reshape(-1, 3)

        save_ply_ascii_color("/tmp/depth_color.ply", pts, colors)
        print("彩色点云已存 /tmp/depth_color.ply (%d 点)" % len(pts))
    finally:
        cap.release()
        cam.close()
    print("DONE")


if __name__ == "__main__":
    main()
