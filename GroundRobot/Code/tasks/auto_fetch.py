#!/usr/bin/python3
# coding=utf8
"""自动取物编排：搜索(search) → 逼近(approach) → 前伸+红外下降夹取(grab)。

- search：找目标 + 走路逼近（只动 1-20 + 云台 21/24）；
- grab：前伸 + 画面占比闭环 + 红外下降夹取；
- 运行期间本进程独占摄像头，并把带标注画面推给 task_server /video.mjpeg，
  地面站可直接观看（http://<机器人IP>:5000/video.mjpeg）。
  不要再同时运行 CS-video.py：摄像头同一时刻只能被一个进程打开。
"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

try:
    from communication import task_server
except ImportError:
    task_server = None


def main():
    import cv2
    from agcs_lib import (
        make_board, make_ik, make_arm_ik, load_params,
        load_lab_data, load_block_params, detect_color, correct_camera,
        load_undistort_maps, make_ultrasonic, make_display, open_camera,
        capture, stand,
    )
    from agcs_lib.search import Searcher
    from agcs_lib.grab import Grabber
    from agcs_lib.sensors import show_status

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--detector', default='color', choices=['color', 'yolo'])
    parser.add_argument('--model', default='models/worm_best.onnx', help='yolo 时的 ONNX 模型路径')
    parser.add_argument('--conf', type=float, default=0.35, help='yolo 置信度阈值')
    parser.add_argument('--ratio', type=float, default=0.3, help='夹取占比阈值（目标占画面 320x240 比例 >= 此值夹取）')
    args = parser.parse_args()
    color = args.color

    from agcs_lib.logs import setup_logger, action_msg
    from agcs_lib.params import summarize
    logger = setup_logger('auto_fetch')
    logger.info('启动 auto_fetch：color=%s', color)

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(board)
    params = load_params()
    params['grab']['grab_area_ratio'] = args.ratio  # 命令行 --ratio 覆盖 yaml 占比阈值
    logger.info('关键生效参数：\n%s', summarize(params))
    if task_server is not None:
        task_server.start_server()
        task_server.set_status(state='FETCH', message='auto_fetch 启动，目标颜色=%s' % color)
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    # 红外(VL53L0X)初始化（寻路 + 夹取共享）
    tof = None
    try:
        import board as adafruit_board
        import busio
        import adafruit_vl53l0x
        i2c = busio.I2C(adafruit_board.SCL, adafruit_board.SDA)
        tof = adafruit_vl53l0x.VL53L0X(i2c)
        logger.info('红外初始化成功')
    except Exception as e:
        logger.info('红外初始化失败: %s' % e)
    cam = open_camera()
    # 启动自检：取不到帧直接报错退出，避免“摄像头被占用还在空转”难排查
    if capture(cam, tries=10) is None:
        logger.error(
            '摄像头取帧失败：可能被其他进程占用（spiderpi 服务 / CS-video.py）。'
            '请先 sudo systemctl stop spiderpi，并确认没有其它程序打开摄像头')
        cam.camera_close()
        return

    detector = None
    if args.detector == 'yolo':
        from functions.yolo_detect_onnx import YoloDetector
        detector = YoloDetector(args.model, conf=args.conf)

    def detect(min_area=150):
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        if detector is not None:
            r = detector.detect(f, min_area=min_area)
        else:
            r = detect_color(f, lab, color, min_area=min_area)
        # 把带标注的画面推给地面站 /video.mjpeg（内部 10fps 限流，线程安全）
        if task_server is not None:
            task_server.publish_frame(f)
        return r

    stand(ik)
    # 机械臂复位（21/22/23/24 回 reset_pulses，夹爪张开）
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(2)

    # 1) 搜索
    searcher = Searcher(board, ik, ak, params, detect, ultrasonic, display, tof=tof)
    center, _ = searcher.run()
    if center is None:
        logger.info('[search] %s', action_msg('未找到目标', reason='颜色=%s' % color))
        # 先停掉可能还在跑的追踪/避障线程，避免它们和复位抢舵机
        searcher.stop()
        # 恢复官方初始机器姿态：云台回中 + 机械臂复位 + 立正
        searcher.reset_pose()
        arm_pulses = params['arm']['reset_pulses']
        board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
        board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
        time.sleep(1.5)
        stand(ik)
        cam.camera_close()
        show_status(display, 0)
        return

    # 寻路刚停，停止追踪（夹取阶段不再调 21/24 居中，只判断面积夹取，避免 21 号抽搐）
    searcher.tracker.stop()
    time.sleep(1)
    grabber = Grabber(board, ik, ak, params, K, R, T, detect, display, ultrasonic=ultrasonic, tof=tof)
    ok = grabber.run()
    searcher.stop()  # 夹取结束，清理追踪线程 + 避障线程

    # 夹住保持，人工确认夹稳后再松开（测试阶段：回车前一直夹紧）
    if ok:
        try:
            input('已夹住，保持夹紧。敲回车松开夹爪并复位...')
        except EOFError:
            # 非交互运行（SSH 自动化测试）：2 秒后自动松开，避免卡住
            logger.info('非交互运行，2 秒后自动松开夹爪并复位')
            time.sleep(2)

    # 归位到官方初始位置（机械臂复位 + 夹爪张开）
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(1.5)

    cam.camera_close()
    if ok:
        logger.info('[grab] %s', action_msg('完成', action='%s 已夹取' % color))
        show_status(display, 3)
        if task_server is not None:
            task_server.set_status(last_result='done', message='夹取完成')
        time.sleep(5)
        show_status(display, 0)
    else:
        logger.info('[grab] %s', action_msg('夹取失败', reason='颜色=%s' % color))
        show_status(display, 0)
        if task_server is not None:
            task_server.set_status(last_result='failed', message='夹取失败')


if __name__ == '__main__':
    main()
