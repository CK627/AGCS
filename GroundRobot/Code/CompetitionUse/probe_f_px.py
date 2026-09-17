#!/usr/bin/python3
# coding=utf8
"""测融合导航的 f_px（320 坐标系焦距）。

方法：把一个已知宽度 W(mm) 的物体（比如红色方块，先量它的实际宽度）放到
相机正前方已知距离 D(mm) 处，脚本检测它的像素宽度 w_px（320 坐标），然后
f_px = w_px × D / W。

用法（在机器人上）：
    sudo systemctl stop spiderpi
    python3 CompetitionUse/probe_f_px.py --color red
    看打印的 w_px，按提示输入方块实际宽度 W(mm) 和到相机的距离 D(mm)

把算出的 f_px 填进 1.py 的 --f-px。
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
)


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
    print('相机已开，找 %s 方块…（Ctrl+C 退出）' % args.color, flush=True)

    try:
        for _ in range(5):
            capture(cam)

        while True:
            f = capture(cam)
            if f is None:
                time.sleep(0.5)
                continue
            frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
            result = detect_color(frame, lab, args.color, min_area=50)
            if result is None:
                print('  未检测到 %s 方块' % args.color, flush=True)
                time.sleep(0.5)
                continue

            # contour 是 320 坐标（detect_color 内部 resize 到 320×240），
            # 融合导航的 bbox_center_x / f_px 也都在 320 坐标，所以用这个宽度
            x, y, w, h = cv2.boundingRect(result['contour'])
            print('方块像素宽 w_px=%d（320 坐标，高 %d）' % (w, h), flush=True)
            try:
                W = float(input('  输入方块实际宽度 W(mm)：').strip())
                D = float(input('  输入方块到相机距离 D(mm)：').strip())
            except (ValueError, EOFError):
                print('  输入无效，重试\n', flush=True)
                continue
            if W <= 0 or D <= 0:
                print('  W/D 必须 > 0\n', flush=True)
                continue
            f_px = w * D / W
            print('  >>> f_px = %d × %.0f / %.0f = %.1f'
                  % (w, D, W, f_px), flush=True)
            print('  >>> 填 --f-px %.0f\n' % f_px, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cam.camera_close()


if __name__ == '__main__':
    main()
