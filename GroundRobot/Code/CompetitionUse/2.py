#!/usr/bin/python3
# coding=utf8
"""2.py —— 单独测 YOLO 检测 + 夹取（不走路线，带启动诊断）。

跑法（在机器人上）：
    sudo systemctl stop spiderpi
    cd ~/spiderpi/CompetitionUse
    python3 2.py                 # 检测 + 夹取
    python3 2.py --conf 0.4      # 检测不到先降置信度门槛

流程：
    1. 不恢复原位
    2. 先转 21 号到虫子所在的左侧位置（--s21，默认 875）
    3. 再扫 24 号俯仰（150→450，小=低头、大=抬头），每个角度做一次 YOLO 检测
    4. 检测到 → 夹取（闭合 25 → 拔起 22）→ 保持姿态 → 敲回车 → 恢复原位 → 退出
    5. 扫完（--tries 轮）都没检测到 → 恢复原位 → 退出

启动时会先打一段诊断：模型是否加载（输入/输出形状）、相机是否出帧、
单次推理耗时、以及 8400 个框的最大置信度。最大置信度若接近 0，说明不是
「太快/卡顿」，而是这个画面里模型根本没认出目标（角度/外观不对）。
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

# 21 号底座横转：虫子固定在机器人左侧，先转 21 到这个角度，再扫 24 俯仰。
SERVO21_POS = 875
# 24 号腕俯仰扫描序列：小=低头看地、大=抬头，从低往高扫。
SERVO24_SCAN = (150, 200, 250, 300, 350, 400, 450)


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
    """夹取（复刻 1.py 夹取开始到夹取结束：闭合夹爪 25 → 拔起 22）。不恢复原位，恢复交给调用方。"""
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
        print('  ❌ 相机 read_frame 返回 None —— 相机没出帧，模型看不到任何画面', flush=True)
        print('--- 诊断结束 ---', flush=True)
        return

    print('  相机帧: %s 平均亮度 %.1f' % (frame.shape, float(np.asarray(frame).mean())), flush=True)
    h0, w0 = frame.shape[:2]
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
        print('  → 最大置信度已过阈值：detect() 应该能返回结果，往下看扫描', flush=True)
    print('--- 诊断结束 ---', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=no7.DEFAULT_MODEL, help='YOLO 模型路径')
    ap.add_argument('--conf', type=float, default=no7.MODEL_CONF,
                    help='模型置信度阈值（检测不到先降到 0.4 试）')
    ap.add_argument('--classes', default='', help='目标类别，逗号分隔；留空=接受所有类别')
    ap.add_argument('--color', default='red', help='推流 LAB 显示用色（只影响显示）')
    ap.add_argument('--s21', type=int, default=SERVO21_POS,
                    help='21 号转到的左侧虫子位置（默认 %(default)d）')
    ap.add_argument('--pull-up', type=int, default=no7.PULL_UP_22,
                    help='夹取后 22 号肩拔起脉宽（默认 %(default)d）')
    ap.add_argument('--tries', type=int, default=3,
                    help='24 号俯仰扫几轮（默认 %(default)d）')
    ap.add_argument('--dwell', type=float, default=1.0,
                    help='每个 24 角度停留秒（默认 %(default)s）')
    args = ap.parse_args()

    board = make_board()

    # 不恢复原位；先转 21 到虫子左侧（0.5s 快速到位 + 0.5s 静置），再扫 24 俯仰
    board.bus_servo_set_position(0.5, [[21, args.s21]])
    time.sleep(1.0)
    print('21 号已转到 %d（虫子左侧位置），开始扫 24 号俯仰' % args.s21, flush=True)

    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    cam, read_frame, _color_detector, publish = no7.open_vision(args.color, 1)
    model_det = no7.ModelDetector(model_path, args.conf, classes, read_frame, publish)

    print('模型加载成功: %s' % model_path, flush=True)
    debug_probe(model_det)

    if no7.task_server is not None:
        no7.task_server.start_server()

    print('=' * 64, flush=True)
    print('2.py：转 21=%d → 扫 24 俯仰，检测到就夹取' % args.s21, flush=True)
    print('conf=%.2f 扫 %d 轮 / 每角度 %.1fs' % (args.conf, args.tries, args.dwell), flush=True)
    print('=' * 64, flush=True)

    for attempt in range(1, args.tries + 1):
        for p24 in SERVO24_SCAN:
            board.bus_servo_set_position(0.5, [[24, p24]])
            time.sleep(args.dwell)
            det = model_det.detect()
            if det is not None:
                print('✅ 21=%d 24=%d 检测到虫子 conf=%.2f，执行夹取'
                      % (args.s21, p24, det['conf']), flush=True)
                grab(board, args.pull_up)
                input('夹取完成，保持夹取姿态。敲回车恢复原位…')
                no7.restore_travel(board, no7.GRIPPER_OPEN)
                print('已恢复原位，退出', flush=True)
                cam.camera_close()
                return
            print('  24=%d 未检测到（第 %d/%d 轮）' % (p24, attempt, args.tries), flush=True)
        print('第 %d 轮 24 号俯仰扫完未找到' % attempt, flush=True)

    print('❌ 扫完 24 号俯仰都没检测到虫子，恢复原位后退出', flush=True)
    no7.restore_travel(board, no7.GRIPPER_OPEN)
    cam.camera_close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
