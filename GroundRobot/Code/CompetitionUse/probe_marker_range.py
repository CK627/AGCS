#!/usr/bin/python3
# coding=utf8
"""读红色方块到相机的距离（marker_range），单位 mm。

用 Astra Pro 深度相机直接读方块处的深度。跑之前把方块放到导航时它所在的位置，
机器人摆到导航姿态（臂复位、相机朝前下方）。

用法（在机器人上）：
    sudo systemctl stop spiderpi
    python3 CompetitionUse/probe_marker_range.py --color red

会持续打印 marker_range，Ctrl+C 退出。把打印的值填进 1.py 的 --marker-range。
"""

import argparse
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    load_params,
    load_lab_data,
    load_undistort_maps,
    detect_color,
    correct_camera,
    open_camera,
    capture,
    DepthCamera,
)


def _depth_median(d, cx, cy, r=2):
    """取 (cx,cy) 附近深度中位数（中心无效时往邻域扩）。返回 None 表示整片无效。"""
    xi, yi = int(round(cx)), int(round(cy))
    h, w = d.shape
    for radius in range(0, r + 1):
        vals = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                yy, xx = yi + dy, xi + dx
                if 0 <= xx < w and 0 <= yy < h:
                    v = int(d[yy, xx])
                    if v > 0:
                        vals.append(v)
        if vals:
            vals.sort()
            return vals[len(vals) // 2]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--color', default='red',
                    choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    args = ap.parse_args()

    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()

    cam = open_camera()
    depth_cam = DepthCamera()
    depth_cam.open()
    depth_cam.start_depth()
    print('相机 + 深度已开，找 %s 方块…（Ctrl+C 退出）' % args.color, flush=True)

    try:
        # 预热，让彩色/深度都稳定
        for _ in range(5):
            capture(cam)
            depth_cam.read_depth(timeout_ms=2000)

        while True:
            f = capture(cam)
            d = depth_cam.read_depth(timeout_ms=2000)
            if f is None or d is None:
                time.sleep(0.5)
                continue
            frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
            result = detect_color(frame, lab, args.color, min_area=50)
            if result is None:
                print('  未检测到 %s 方块' % args.color, flush=True)
                time.sleep(0.5)
                continue
            cx, cy = result['center']   # center 是映射回 640 分辨率的像素
            z = _depth_median(d, cx, cy)
            if z is None:
                print('  方块中心深度无效（可能太近/贴边）', flush=True)
                time.sleep(0.5)
                continue
            print('方块 center=(%d,%d) 深度=%dmm  →  marker_range=%dmm'
                  % (cx, cy, z, z), flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        cam.camera_close()
        depth_cam.close()


if __name__ == '__main__':
    main()
