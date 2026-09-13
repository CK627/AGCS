#!/usr/bin/python3
# coding=utf8
"""比赛流程公共初始化（2.1~2.5 共用）。

把 tasks/auto_fetch.py 里重复的初始化（board/IK/机械臂/相机/检测闭包/传感器）
抽到这里，各步骤脚本只 import 本模块，做到「每步独立脚本」又不重复。
"""
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from types import SimpleNamespace


def build_runtime(args, logger, publish_frame=None):
    """初始化机器人运行环境。

    返回 namespace(board, ik, ak, params, cam, detect, display, ultrasonic,
    tof, depth, K, R, T)。其中 detect(min_area) 是闭包：取一帧 → 畸变校正 →
    颜色/YOLO 检测 → 返回 dict(center, area, color, ...) 或 None。

    publish_frame：可选回调，每取到一帧矫正后调用（用于推给地面站 task_server）。
    """
    import cv2
    from agcs_lib import (
        make_board, make_ik, make_arm_ik, load_params,
        load_lab_data, load_block_params, detect_color, correct_camera,
        load_undistort_maps, make_ultrasonic, make_display, open_camera,
        capture,
    )

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(board)
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    cam = open_camera()

    # 自检：取不到帧直接报错退出，避免「摄像头被占用还在空转」难排查
    if capture(cam, tries=10) is None:
        logger.error('摄像头取帧失败：可能被其他进程占用（spiderpi 服务 / CS-video.py）。'
                     '请先 sudo systemctl stop spiderpi，并确认没有其它程序打开摄像头')
        cam.camera_close()
        return None

    # 红外(VL53L0X)已拆除，Astra Pro 深度相机自带测距，不再初始化
    tof = None

    # 深度相机(Astra Pro)初始化（寻路逼近判距用）；失败退回面积判距
    depth = None
    try:
        from agcs_lib.depth import DepthCamera
        depth = DepthCamera()
        depth.open()
        depth.start_depth()
        logger.info('深度相机初始化成功')
    except Exception as e:
        logger.info('深度相机初始化失败，退回面积判距: %s' % e)
        depth = None

    detector = None
    if getattr(args, 'detector', 'color') == 'yolo':
        from functions.yolo_detect_onnx import YoloDetector
        detector = YoloDetector(args.model, conf=args.conf)

    color = getattr(args, 'color', 'red')

    def detect(min_area=50):
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        if publish_frame is not None:
            try:
                publish_frame(f)
            except Exception:
                pass
        if detector is not None:
            return detector.detect(f, min_area=min_area)
        return detect_color(f, lab, color, min_area=min_area)

    return SimpleNamespace(
        board=board, ik=ik, ak=ak, params=params, cam=cam, detect=detect,
        display=display, ultrasonic=ultrasonic, tof=tof, depth=depth, K=K, R=R, T=T,
    )


def reset_arm(board, params):
    """机械臂复位（21/22/23/24 回 reset_pulses，夹爪张开）。"""
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(2)


def reset_gimbal(board):
    """云台回中（21=500 / 24=260）。"""
    board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
    time.sleep(0.6)


def close_runtime(rt):
    """关闭运行时资源（相机 + 深度相机）。"""
    try:
        rt.cam.camera_close()
    except Exception:
        pass
    if getattr(rt, 'depth', None) is not None:
        try:
            rt.depth.close()
        except Exception:
            pass
