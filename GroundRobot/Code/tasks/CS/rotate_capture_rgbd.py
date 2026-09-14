#!/usr/bin/python3
# coding=utf8
"""原地转一圈采集 RGB-D 帧（建图用），IMU 反馈保证每步转角准确。

把机器人放到作业空间大致中心（不放植株），原地转身 360°，每转 step 度拍一帧
深度 + 彩色。用 IMU 反馈转角（转到目标航向，误差 <=1°），保证转完一圈确实
回到原位，不会因固定步数转角偏差越偏越多。

用法（先 sudo systemctl stop spiderpi）：
    python3 rotate_capture_rgbd.py --out /tmp/rgbd_360 --views 24
"""
import argparse
import json
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import DepthCamera, make_board, make_ik, stand


def read_gz(board):
    """读 IMU gz 角速度。"""
    try:
        data = board.get_imu()
        if data is None:
            return None
        return float(data[5])
    except Exception:
        return None


def init_imu(board):
    """标定 gz 零漂，返回 imu_state。"""
    board.enable_reception()
    vals = []
    while len(vals) < 200:
        gz = read_gz(board)
        if gz is not None:
            vals.append(gz)
        time.sleep(0.005)
    return {'bias': sum(vals) / len(vals) if vals else 0.0,
            'yaw': 0.0, 'last_t': time.monotonic()}


def update_yaw(state, board):
    now = time.monotonic()
    dt = now - state['last_t']
    state['last_t'] = now
    gz = read_gz(board)
    if gz is None:
        return
    state['yaw'] += (gz - state['bias']) * dt * 1.18


def turn_to_heading(ik, board, imu_state, target_yaw):
    """IMU 反馈转到目标航向（误差 <= 1°）。"""
    for _ in range(40):
        update_yaw(imu_state, board)
        err = target_yaw - imu_state['yaw']
        if abs(err) <= 1.0:
            return
        step = min(5.0, abs(err))
        if err > 0:
            ik.turn_left(ik.initial_pos, 2, int(step), 60, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, int(step), 60, 1)
        time.sleep(0.25)


def main():
    parser = argparse.ArgumentParser(description='原地转一圈采集 RGB-D（IMU 反馈转角）')
    parser.add_argument('--out', default='/tmp/rgbd_360')
    parser.add_argument('--views', type=int, default=24, help='360° 视角数')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    for _ in range(5):
        cap.read()

    fx, fy, cx, cy = cam.get_depth_intrinsics()
    with open(os.path.join(args.out, 'intrinsic.json'), 'w') as f:
        json.dump({'width': 640, 'height': 480, 'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy},
                  f, indent=2)
    print('深度内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f' % (fx, fy, cx, cy), flush=True)

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)
    imu_state = init_imu(board)

    step = 360.0 / args.views
    target_yaw = 0.0
    idx = 0
    try:
        for v in range(args.views):
            time.sleep(0.5)  # 站稳
            d = cam.read_depth(timeout_ms=2000)
            ok, bgr = cap.read()
            if d is None or not ok:
                print('视角 %d/%d 读取失败' % (v + 1, args.views), flush=True)
            else:
                cv2.imwrite(os.path.join(args.out, 'depth_%05d.png' % idx), d)
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                cv2.imwrite(os.path.join(args.out, 'color_%05d.jpg' % idx), rgb,
                            [cv2.IMWRITE_JPEG_QUALITY, 90])
                print('视角 %d/%d 已存 yaw=%.1f°' % (v + 1, args.views, imu_state['yaw']),
                      flush=True)
                idx += 1
            if v < args.views - 1:
                target_yaw += step
                turn_to_heading(ik, board, imu_state, target_yaw)
                time.sleep(0.5)
    finally:
        cap.release()
        cam.close()
        stand(ik)
    print('采集完成，共 %d 帧，最终 yaw=%.1f° -> %s'
          % (idx, imu_state['yaw'], args.out), flush=True)


if __name__ == '__main__':
    main()
