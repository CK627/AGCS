#!/usr/bin/python3
# coding=utf8
"""深度摄像头单测：读深度图 + 彩色图，验证 Astra Pro 深度流在 Python 里可用。

用法：
    python3 CS-depth.py
"""
import os
import sys
import time

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.depth import DepthCamera, DepthCameraError


def main():
    cam = DepthCamera()
    try:
        cam.open()
        cam.start_depth()
        print("深度流启动 OK")

        # 彩色流走 OpenNI2，若与 uvcvideo 冲突则退回 OpenCV
        color_mode = "openni2"
        try:
            cam.start_color()
            print("彩色流(OpenNI2)启动 OK")
        except DepthCameraError as e:
            color_mode = "opencv"
            print("彩色流(OpenNI2)失败: %s（改用 OpenCV 读 /dev/video0）" % e)

        # 预热几帧，让深度稳定
        for _ in range(5):
            cam.read_depth(timeout_ms=2000)
        time.sleep(0.5)

        depths = []
        print("---- 连续读 10 帧深度 ----")
        for i in range(10):
            d = cam.read_depth(timeout_ms=2000)
            if d is None:
                print("帧 %d: 超时" % i)
                continue
            center = d[d.shape[0] // 2, d.shape[1] // 2]
            valid = d[d > 0]
            depths.append(d)
            mean = valid.mean() if valid.size else 0
            print("帧 %d: shape=%s center=%dmm 非零像素=%d 均值=%.0fmm"
                  % (i, d.shape, center, valid.size, mean))

        if not depths:
            print("FAIL: 没读到任何有效深度帧")
            sys.exit(1)

        import cv2
        d = depths[-1]
        vis = np.zeros(d.shape, dtype=np.uint8)
        nz = d > 0
        if nz.any():
            vis[nz] = np.clip(d[nz] / 4000.0 * 255, 0, 255).astype(np.uint8)
        cv2.imwrite("/tmp/depth.png", vis)
        print("深度图已存 /tmp/depth.png (%s，0 表示无效像素)" % str(d.shape))

        # 彩色图：OpenNI2 彩色会被 uvcvideo 占用而超时，回退 OpenCV
        if color_mode == "openni2":
            c = cam.read_color(timeout_ms=2000)
            if c is not None:
                cv2.imwrite("/tmp/color_oni.png", cv2.cvtColor(c, cv2.COLOR_RGB2BGR))
                print("彩色图(OpenNI2)已存 /tmp/color_oni.png %s" % str(c.shape))
            else:
                print("彩色流(OpenNI2)超时（uvcvideo 占用），回退 OpenCV")
                color_mode = "opencv"
        if color_mode == "opencv":
            cap = cv2.VideoCapture(0)
            ok, bgr = cap.read()
            cap.release()
            if ok:
                cv2.imwrite("/tmp/color_ocv.png", bgr)
                print("彩色图(OpenCV)已存 /tmp/color_ocv.png %s" % str(bgr.shape))

        print("PASS")
    finally:
        cam.close()
        print("DONE")


if __name__ == "__main__":
    main()
