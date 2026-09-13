#!/usr/bin/python3
# coding=utf8
"""深度点云单测：取深度内参 + 深度图转点云，存 .ply。

用法：
    python3 CS-depth-pcl.py
"""
import os
import sys

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.depth import DepthCamera


def save_ply_ascii(path, xyz):
    """把 (N,3) 世界坐标存成 ASCII PLY。"""
    n = len(xyz)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write("element vertex %d\n" % n)
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for p in xyz:
            f.write("%.2f %.2f %.2f\n" % (p[0], p[1], p[2]))


def main():
    cam = DepthCamera()
    cam.open()
    cam.start_depth()
    try:
        fx, fy, cx, cy = cam.get_depth_intrinsics()
        print("深度内参: fx=%.3f fy=%.3f cx=%.1f cy=%.1f" % (fx, fy, cx, cy))

        d = cam.read_depth(timeout_ms=2000)
        h, w = d.shape
        fov_h = 2 * np.degrees(np.arctan(w / 2.0 / fx))
        fov_v = 2 * np.degrees(np.arctan(h / 2.0 / fy))
        print("FOV: %.1f°H x %.1f°V" % (fov_h, fov_v))

        pcl = cam.depth_to_pointcloud(d)
        valid = ~np.isnan(pcl[:, :, 0])
        pts = pcl[valid].reshape(-1, 3)
        print("点云: %d 有效点 (%.1f%%), Z 范围 %.0f~%.0fmm" % (
            len(pts), 100.0 * len(pts) / (w * h), pts[:, 2].min(), pts[:, 2].max()))

        save_ply_ascii("/tmp/depth.ply", pts)
        print("点云已存 /tmp/depth.ply (%d 点)" % len(pts))
    finally:
        cam.close()
    print("DONE")


if __name__ == "__main__":
    main()
