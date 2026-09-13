#!/usr/bin/python3
# coding=utf8
"""多视角点云采集：机器人原地转身，每个视角拍一帧深度+彩色点云存成 .npz。

安全须知（务必确认）：
1. 机器人周围要空旷，平台稳固，转身时别撞到东西/别掉下桌。
2. 先停自启服务释放串口：sudo systemctl stop spiderpi
3. 转身角度是名义值（六足转身有误差），配准靠 ICP 精修，不需要精确。

用法：
    python3 CS-multiview-capture.py --step 30 --views 12 --out /tmp/pcl_views
    # 只测采集、不转身：
    python3 CS-multiview-capture.py --no-motion
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera, make_board, make_ik, stand, turn_left


def capture_view(cam, cap, outdir, idx):
    """拍一帧深度+彩色，转成点云存 npz。返回 True/False。"""
    d = cam.read_depth(timeout_ms=2000)
    ok, bgr = cap.read()
    if d is None or not ok:
        return False
    pcl = cam.depth_to_pointcloud(d)
    valid = ~np.isnan(pcl[:, :, 0])
    pts = pcl[valid].reshape(-1, 3).astype(np.float32)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)[valid].reshape(-1, 3).astype(np.uint8)
    np.savez_compressed(os.path.join(outdir, "view_%02d.npz" % idx), pts=pts, rgb=rgb)
    return True


def main():
    parser = argparse.ArgumentParser(description="多视角点云采集")
    parser.add_argument("--step", type=float, default=30.0, help="每次转身角度(度)")
    parser.add_argument("--views", type=int, default=12, help="视角数")
    parser.add_argument("--out", default="/tmp/pcl_views")
    parser.add_argument("--no-motion", action="store_true", help="只采集不转身(测试)")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    ik = None
    if not args.no_motion:
        board = make_board()
        ik = make_ik(board)
        stand(ik)
        time.sleep(1.5)

    try:
        ok_count = 0
        for i in range(args.views):
            time.sleep(1.0)  # 站稳
            ok = capture_view(cam, cap, args.out, i)
            ok_count += ok
            print("视角 %d/%d 采集 %s" % (i + 1, args.views, "OK" if ok else "FAIL"))
            if i < args.views - 1 and not args.no_motion:
                turn_left(ik, angle=args.step, speed=60)
                time.sleep(1.5)  # 转身后站稳
        print("采集完成: %d/%d 成功，输出目录 %s" % (ok_count, args.views, args.out))
    finally:
        if ik is not None:
            stand(ik)  # 复位站姿
        cap.release()
        cam.close()
    print("DONE")


if __name__ == "__main__":
    main()
