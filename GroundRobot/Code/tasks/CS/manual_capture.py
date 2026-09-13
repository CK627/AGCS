#!/usr/bin/python3
# coding=utf8
"""连续采集深度点云：回车开始 → 围着场地走一圈回到起点 → 回车结束。

开始后每 interval 秒自动采一帧深度点云；走完一圈回到起点再回车结束（闭合回环，
配准更稳）。相邻帧要有重叠（走慢点），ICP 才配得上。

只存 pts（不存位姿），位姿由建图时的 ICP 配准估算。

用法（先 sudo systemctl stop spiderpi）：
    python3 manual_capture.py --out /tmp/pcl_manual --interval 1.0
"""
import argparse
import os
import sys
import threading
import time

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='/tmp/pcl_manual')
    parser.add_argument('--interval', type=float, default=1.0, help='采样间隔(秒)')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    input('按回车开始连续采集（每 %.1f 秒一帧），走一圈回到起点后再回车结束...'
          % args.interval)

    stop_event = threading.Event()
    threading.Thread(target=lambda: (input('走完回到起点后，按回车结束...'),
                                     stop_event.set()), daemon=True).start()

    idx = 0
    try:
        while not stop_event.is_set():
            d = cam.read_depth(timeout_ms=2000)
            if d is not None:
                pcl = cam.depth_to_pointcloud(d)
                valid = ~np.isnan(pcl[:, :, 0])
                pts = pcl[valid].reshape(-1, 3).astype(np.float32)
                np.savez_compressed(os.path.join(args.out, 'view_%03d.npz' % idx), pts=pts)
                print('已存 view_%03d (%d 点)' % (idx, len(pts)), flush=True)
                idx += 1
            else:
                print('深度读取失败，跳过', flush=True)
            time.sleep(args.interval)
    finally:
        cam.close()
    print('采集完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)


if __name__ == '__main__':
    main()
