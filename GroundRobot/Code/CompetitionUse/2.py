#!/usr/bin/python3
# coding=utf8
"""2.py —— 固定姿态持续检测 + 夹取（不走路线、不扫描、不深度定位）。

跑法（在机器人上）：
    sudo systemctl stop spiderpi
    cd ~/spiderpi/CompetitionUse
    python3 2.py --conf 0.4

流程：
    1. 固定 21=875、24=400（都不动），打开相机 + YOLO 模型
    2. 一直检测：检测到就打印 conf/bbox 并夹取（推流里也能看框）
    3. 夹取后敲回车恢复原位，再摆回固定姿态继续检测；Ctrl+C 退出
"""

import argparse
import importlib.util
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import make_board

SERVO21_POS = 875   # 21 号固定（维持）
SERVO24_POS = 200   # 24 号固定


def _load_no7():
    """把 Auto-capture-1.py 当模块加载，复用 open_vision / ModelDetector / restore_travel / 常量。"""
    path = os.path.join(_HERE, 'Auto-capture-1.py')
    if not os.path.exists(path):
        raise SystemExit('找不到 %s —— 2.py 必须和 Auto-capture-1.py 放同一目录' % path)
    spec = importlib.util.spec_from_file_location('no7_for_2py', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['no7_for_2py'] = mod
    spec.loader.exec_module(mod)
    return mod


no7 = _load_no7()


def grab(board, pull_up_pulse):
    """闭合夹爪 25 → 拔起 22。不恢复原位（恢复交给调用方）。"""
    board.bus_servo_set_position(2.0, [[25, no7.GRIPPER_CLOSE]])
    time.sleep(2.0)
    time.sleep(0.5)
    pulse = max(0, min(1000, int(pull_up_pulse)))
    board.bus_servo_set_position(1.0, [[22, pulse]])
    time.sleep(1.0)


def debug_probe(model_det):
    """启动诊断：模型加载 / 相机出帧 / 推理耗时 / 最大置信度。"""
    print('--- 模型/相机诊断 ---', flush=True)
    try:
        for i in model_det.sess.get_inputs():
            print('  模型输入: %s %s' % (i.name, i.shape), flush=True)
        for o in model_det.sess.get_outputs():
            print('  模型输出: %s %s' % (o.name, o.shape), flush=True)
    except Exception as e:
        print('  读模型信息失败: %s' % e, flush=True)

    frame = model_det.read_frame()
    if frame is None:
        print('  ❌ 相机 read_frame 返回 None —— 相机没出帧', flush=True)
        print('--- 诊断结束 ---', flush=True)
        return

    print('  相机帧: %s 平均亮度 %.1f' % (frame.shape, float(np.asarray(frame).mean())), flush=True)
    canvas, r, pad_x, pad_y = model_det._letterbox(frame)
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    t0 = time.time()
    out = model_det.sess.run([model_det.output_name], {model_det.input_name: blob})[0][0]
    dt = time.time() - t0
    scores = out[4]
    max_score = float(scores.max()) if len(scores) else 0.0
    print('  单次推理耗时 %.2fs；8400 个框最大置信度 = %.4f（当前阈值 %.2f）'
          % (dt, max_score, model_det.conf), flush=True)
    if max_score < 0.05:
        print('  → 最大置信度接近 0：模型在这个画面里完全没认出目标（角度/外观/距离不对）', flush=True)
    elif max_score < model_det.conf:
        print('  → 有东西但低于阈值：目标在画面里，只是置信度不够（试更低 --conf）', flush=True)
    else:
        print('  → 最大置信度已过阈值：detect() 应该能返回结果', flush=True)
    print('--- 诊断结束 ---', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=no7.DEFAULT_MODEL, help='YOLO 模型路径')
    ap.add_argument('--conf', type=float, default=no7.MODEL_CONF,
                    help='模型置信度阈值（检测不到先降到 0.4 试）')
    ap.add_argument('--classes', default='', help='目标类别，逗号分隔；留空=接受所有类别')
    ap.add_argument('--color', default='red', help='推流 LAB 显示用色（只影响显示）')
    ap.add_argument('--s21', type=int, default=SERVO21_POS, help='21 号固定脉宽（默认 %(default)d）')
    ap.add_argument('--s24', type=int, default=SERVO24_POS, help='24 号固定脉宽（默认 %(default)d）')
    ap.add_argument('--pull-up', type=int, default=no7.PULL_UP_22,
                    help='夹取后 22 号肩拔起脉宽（默认 %(default)d）')
    args = ap.parse_args()

    board = make_board()

    # 固定 21、24，都不动
    board.bus_servo_set_position(0.5, [[21, args.s21], [24, args.s24]])
    time.sleep(1.0)
    print('固定 21=%d 24=%d，开始持续检测' % (args.s21, args.s24), flush=True)

    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    cam, read_frame, _color_detector, publish = no7.open_vision(args.color, 1)
    model_det = no7.ModelDetector(model_path, args.conf, classes, read_frame, publish)

    print('模型加载成功: %s' % model_path, flush=True)
    debug_probe(model_det)

    if no7.task_server is not None:
        no7.task_server.start_server()

    print('=' * 64, flush=True)
    print('2.py：固定姿态持续检测，检测到就夹取（Ctrl+C 退出）', flush=True)
    print('=' * 64, flush=True)

    miss = 0
    try:
        while True:
            det = model_det.detect()
            if det is not None:
                print('✅ 检测到 conf=%.2f bbox=%dx%d@(%d,%d)，夹取'
                      % (det['conf'], det['w'], det['h'], det['x'], det['y']), flush=True)
                grab(board, args.pull_up)
                input('夹取完成，敲回车恢复原位…')
                no7.restore_travel(board, no7.GRIPPER_OPEN)
                # 恢复后重新摆回固定姿态，继续检测
                board.bus_servo_set_position(0.5, [[21, args.s21], [24, args.s24]])
                time.sleep(1.0)
            else:
                miss += 1
                if miss % 10 == 0:
                    print('  未检测到（已 %d 次）' % miss, flush=True)
                time.sleep(0.3)
    finally:
        cam.camera_close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
