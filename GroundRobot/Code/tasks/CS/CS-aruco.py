#!/usr/bin/python3
# coding=utf8
"""ArUco 标记检测单测：彩色相机识别标记并估算距离/方向。

用法：
    python3 CS-aruco.py [--size 100]
先把标记图显示在屏幕（或打印）放在相机前方，运行看检测结果。

标记图：/tmp/aruco_id0.png（边长按 --size 传入的毫米数打印/显示）。
"""
import argparse
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.marker import MarkerDetector


def main():
    parser = argparse.ArgumentParser(description="ArUco 标记检测单测")
    parser.add_argument("--size", type=float, default=100.0, help="标记物理边长 mm")
    args = parser.parse_args()

    det = MarkerDetector(marker_size_mm=args.size)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    try:
        print("连续检测 15 帧（约 7 秒），请把标记放在相机前方...")
        found = 0
        for i in range(15):
            ok, bgr = cap.read()
            if not ok:
                print("帧 %d: 读帧失败" % i)
                continue
            marks = det.detect(bgr)
            if marks:
                found += 1
                for m in marks:
                    t = m["tvec"]
                    cx, cy = det.center(m)
                    print("帧 %d: id=%d 像素中心=(%.0f,%.0f) 距离Z≈%.0fmm X=%.0f Y=%.0f"
                          % (i, m["id"], cx, cy, t[2], t[0], t[1]))
            else:
                print("帧 %d: 未检测到标记" % i)
            time.sleep(0.4)
        print("检测到标记的帧数: %d/15" % found)
        print("PASS" if found > 0 else "FAIL")
    finally:
        cap.release()
    print("DONE")


if __name__ == "__main__":
    main()
