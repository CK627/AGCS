#!/usr/bin/python3
# coding=utf8
"""方案 A：深度 3D 抓取 —— 颜色找目标 → 深度测 3D → 3D 逼近 → 3D 夹取。

高度不再是问题：深度直接给出目标在机械臂坐标系的 (x, y, z)，机械臂 IK 伸到
那个 3D 点夹取，不再假设「目标在地面」。

前提：
    1. 先跑 calib_cam2arm.py 标定好 config/cam2arm.yaml（R_cam2arm / t_cam2arm）；
    2. 标定和抓取时云台(21/24)位姿要一致（本版暂不加云台角度补偿）。

用法：
    python3 depth_3d_grasp.py --color red
"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import numpy as np

import _common
import depth_approach_pick
import depth_camera_grasp_algo as dcg
from agcs_lib.logs import setup_logger, action_msg

try:
    from communication import task_server
except ImportError:
    task_server = None


def load_cam2arm():
    """读 config/cam2arm.yaml，返回 (R_cam2arm(3x3), t_cam2arm(3x1))。"""
    import yaml
    path = os.path.join(_PKG_ROOT, 'config', 'cam2arm.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    c = data['cam2arm']
    R = np.asarray(c['R'], dtype=np.float64).reshape(3, 3)
    t = np.asarray(c['t'], dtype=np.float64).reshape(3, 1)
    return R, t


def main():
    parser = argparse.ArgumentParser(description='深度 3D 抓取（方案 A）')
    parser.add_argument('--color', default='yellow',
                        choices=['red', 'green', 'blue', 'yellow'])
    parser.add_argument('--detector', default='color', choices=['color', 'yolo'])
    parser.add_argument('--model', default='models/fake_bug.onnx')
    parser.add_argument('--conf', type=float, default=0.35)
    args = parser.parse_args()

    logger = setup_logger('depth_3d_grasp')
    logger.info('深度 3D 抓取启动：color=%s', args.color)

    publish = task_server.publish_frame if task_server is not None else None
    rt = _common.build_runtime(args, logger, publish_frame=publish)
    if rt is None:
        return
    if task_server is not None:
        task_server.start_server()
        task_server.set_status(state='FETCH', message='depth_3d 抓取，颜色=%s' % args.color)

    # 手眼标定
    try:
        R_cam2arm, t_cam2arm = load_cam2arm()
    except Exception as e:
        logger.error('加载 cam2arm.yaml 失败，请先跑 calib_cam2arm.py 标定: %s', e)
        _common.close_runtime(rt)
        return

    # 深度内参 K
    fx, fy, cx, cy = rt.depth.get_depth_intrinsics()
    K_depth = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    from agcs_lib import stand, go_forward, go_back, turn_left, turn_right
    from agcs_lib.search import Searcher
    from agcs_lib.sensors import show_status

    stand(rt.ik)
    _common.reset_arm(rt.board, rt.params)
    show_status(rt.display, 1)

    # 1) 搜索：复用 Searcher 的云台扫视找到目标（只 search，不做它的 2D 逼近）
    searcher = Searcher(rt.board, rt.ik, rt.ak, rt.params, rt.detect,
                        rt.ultrasonic, rt.display, tof=rt.tof, depth=rt.depth)
    det = searcher.search()
    if det is None:
        logger.info('[depth_3d] %s', action_msg('未找到目标', reason='颜色=%s' % args.color))
        searcher.reset_pose()
        _common.reset_arm(rt.board, rt.params)
        _common.close_runtime(rt)
        show_status(rt.display, 0)
        return

    # 搜索到位后启动云台跟踪：ColorTracker 持续把目标锁在画面中心（逼近时不丢）
    searcher.tracker.start(searcher.x_dis, searcher.y_dis)

    # 2) get_target：读目标深度（前向距离），驱动"靠近"
    def get_target():
        r = rt.detect()
        if r is None:
            logger.debug('[depth_3d] get_target: 颜色未检测到')
            return None
        cx, cy = r['center']
        d = rt.depth.read_depth(timeout_ms=200)
        if d is None:
            logger.debug('[depth_3d] get_target: 深度读超时')
            return None
        h, w = d.shape
        # 色块中心往往反射不到结构光（深度=0），取中心周围一块区域，
        # 用有效深度的中位数，绕过色块中心的「深度黑洞」
        rr = 30
        x0 = max(0, int(cx) - rr); x1 = min(w, int(cx) + rr)
        y0 = max(0, int(cy) - rr); y1 = min(h, int(cy) + rr)
        valid = d[y0:y1, x0:x1]
        valid = valid[valid > 0]
        if valid.size == 0:
            # 深度全无效但颜色还在：说明目标已太近（低于深度相机最小测距），视为已靠近
            logger.debug('[depth_3d] get_target: 深度全无效，视为已靠近 (cx=%d cy=%d)' % (cx, cy))
            return 0, 100, 0, None  # 100mm=10cm，落入 should_approach 的 ok 区间，停下
        z_mm = int(np.median(valid))
        # 只用深度前向距离驱动"靠近"（横向/高低交给云台跟踪），不做机械臂坐标换算
        return 0, z_mm, 0, None

    # 3) move：把 3D 逼近的动作映射成六足运动
    walk_mm = 40
    walk_speed = 50
    turn_deg = 10
    turn_speed = 80

    def move(action):
        logger.info('[depth_3d] 动作=%s', action)
        if action == 'forward':
            go_forward(rt.ik, walk_mm, walk_speed, 1)
        elif action == 'back':
            go_back(rt.ik, walk_mm, walk_speed, 1)
        elif action == 'left':
            turn_left(rt.ik, turn_deg, turn_speed)
        elif action == 'right':
            turn_right(rt.ik, turn_deg, turn_speed)
        elif action == 'too_high':
            logger.info('[depth_3d] %s', action_msg('目标太高，够不到'))
        elif action == 'search':
            logger.info('[depth_3d] %s', action_msg('目标丢失，重新搜索'))
        time.sleep(0.2)

    # 4) 3D 逼近 + 夹取
    result = depth_approach_pick.approach_loop(get_target, move)
    searcher.stop()

    _common.reset_arm(rt.board, rt.params)
    _common.close_runtime(rt)
    show_status(rt.display, 3 if result else 0)
    if result:
        logger.info('[depth_3d] %s', action_msg('已靠近到位'))
    else:
        logger.info('[depth_3d] %s', action_msg('逼近失败'))


if __name__ == '__main__':
    main()
