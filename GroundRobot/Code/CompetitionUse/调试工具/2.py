#!/usr/bin/python3
# coding=utf8
"""2.py —— 识别到目标 → 六足直线前进逼近 → 夹取。

跑法（在机器人上）：
    sudo systemctl stop spiderpi
    cd ~/spiderpi/CompetitionUse
    python3 2.py --conf 0.4 --stop-depth 22 --step-mm 30

流程：
    1. 固定 21/22/23/24 摆臂（相机看目标），打开相机 + YOLO 模型 + 深度相机
    2. 检测到目标 → 用深度(主) + bbox 面积(兜底) 判「够近」
    3. 不够近 → 六足直线前进一小段再检测；够近 → 夹取
    4. 目标连续丢帧超 --lost-limit 就停（不盲夹）；Ctrl+C 退出
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

from agcs_lib import make_board, make_ik
from agcs_lib.depth import DepthCamera

SERVO21_POS = 875   # 21 号固定（维持）
SERVO22_POS = 425   # 22 号肩（路线 JSON pick1 的 22=425，决定相机高度）
SERVO23_POS = 295   # 23 号肘（路线 JSON pick1 的 23=295，决定相机前伸）
SERVO24_POS = 300   # 24 号固定
MOVE_SPEED = 50     # 六足直线前进速度（同 1.py）

# 主动追踪（同 agcs_lib/tracker.py 的 ColorTracker）：21 水平 / 24 俯仰，死区 + 比例控制
IMG_CX = 320        # 画面中心（640×480）
IMG_CY = 240
TRACK_P = 0.1       # P 增益（ColorTracker 同款）
TRACK_DEAD_X = 40   # 水平死区（像素）：|cx-320| 小于它不动 21
TRACK_DEAD_Y = 60   # 垂直死区（像素）：|cy-240| 小于它不动 24


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


def depth_cm_at(depth, cx, cy, rotate):
    """读深度相机在彩色像素 (cx, cy) 处的距离(cm)；无有效读数返回 None。

    与 search.py 的 _depth_cm_at 同款：深度帧与彩色帧同为 640×480，按 camera_rotate
    对齐后取目标中心像素深度，0 表示无效（太近/无数据）。深度/彩色传感器 FOV 不完全
    重合，取的是 bbox 中心像素的近似值，够判「到没到夹取距离」用。
    """
    try:
        d = depth.read_depth(timeout_ms=200)
        if d is None:
            return None
        if rotate:
            d = no7.correct_camera(d, rotate)
        h, w = d.shape
        dx = min(w - 1, max(0, int(round(cx))))
        dy = min(h - 1, max(0, int(round(cy))))
        z_mm = int(d[dy, dx])
        if z_mm <= 0:
            return None
        return z_mm / 10.0
    except Exception:
        return None


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
    scores = out[4 + model_det.CLASS_IDX]
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
    ap.add_argument('--s22', type=int, default=SERVO22_POS, help='22 号肩固定脉宽（默认 %(default)d）')
    ap.add_argument('--s23', type=int, default=SERVO23_POS, help='23 号肘固定脉宽（默认 %(default)d）')
    ap.add_argument('--s24', type=int, default=SERVO24_POS, help='24 号固定脉宽（默认 %(default)d）')
    ap.add_argument('--pull-up', type=int, default=no7.PULL_UP_22,
                    help='夹取后 22 号肩拔起脉宽（默认 %(default)d）')
    ap.add_argument('--stop-depth', type=float, default=22.0,
                    help='深度 ≤ N cm 判「够近」（默认 %(default)s）')
    ap.add_argument('--close-area', type=int, default=20000,
                    help='bbox w×h ≥ N 判「够近」（深度读不到时的兜底，默认 %(default)d）')
    ap.add_argument('--step-mm', type=int, default=30,
                    help='每次直线前进的毫米数（默认 %(default)d）')
    ap.add_argument('--lost-limit', type=int, default=8,
                    help='连续丢帧超过 N 次就停（默认 %(default)d）')
    ap.add_argument('--max-steps', type=int, default=30,
                    help='逼近步数上限，安全护栏（默认 %(default)d）')
    ap.add_argument('--no-depth', action='store_true',
                    help='关掉深度相机，退回纯 bbox 面积判近')
    args = ap.parse_args()

    board = make_board()
    ik = make_ik(board)

    # 深度相机（可选）：DepthCamera 走 OpenNI2，彩色走 OpenCV /dev/video0，两者可同时开
    depth = None
    if not args.no_depth:
        try:
            depth = DepthCamera()
            depth.open()
            depth.start_depth()
            print('深度相机已打开', flush=True)
        except Exception as e:
            print('深度相机打开失败（%s），退回纯 bbox 判近' % e, flush=True)
            depth = None

    # 固定 21/22/23/24 摆臂（22/23 决定相机高度/前伸，21/24 决定相机朝向）
    board.bus_servo_set_position(0.5, [[21, args.s21], [22, args.s22], [23, args.s23], [24, args.s24]])
    time.sleep(1.5)
    print('固定 21=%d 22=%d 23=%d 24=%d' % (args.s21, args.s22, args.s23, args.s24), flush=True)

    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    cam, read_frame, _color_detector, publish = no7.open_vision(args.color, 1)
    model_det = no7.ModelDetector(model_path, args.conf, classes, read_frame, publish)

    print('模型加载成功: %s' % model_path, flush=True)
    debug_probe(model_det)

    if no7.task_server is not None:
        no7.task_server.start_server()

    rotate = no7.load_params()['vision'].get('camera_rotate', 0)

    print('=' * 64, flush=True)
    print('2.py：识别 → 直线前进逼近 → 夹取（Ctrl+C 退出）', flush=True)
    print('=' * 64, flush=True)

    try:
        ik.stand(ik.initial_pos, t=500)
        time.sleep(0.5)

        s21 = args.s21   # 追踪状态：从固定摆位起步，之后 21/24 随目标微调
        s24 = args.s24
        steps = 0
        miss = 0
        while steps < args.max_steps:
            det = model_det.detect()
            if det is None:
                miss += 1
                if miss >= args.lost_limit:
                    print('目标连续丢失 %d 帧，停止逼近（不盲夹）' % miss, flush=True)
                    break
                time.sleep(0.3)
                continue
            miss = 0

            cx = det['x'] + det['w'] / 2.0
            cy = det['y'] + det['h'] / 2.0
            area = det['w'] * det['h']
            # 先读深度（云台还没动，深度像素 (cx,cy) 对应目标当前位置）
            dist_cm = depth_cm_at(depth, cx, cy, rotate) if depth is not None else None

            close = False
            if dist_cm is not None and dist_cm < args.stop_depth:
                close = True
            elif dist_cm is None and area >= args.close_area:
                close = True

            if close:
                print('✅ 够近 dist=%s area=%d，夹取（conf=%.2f bbox=%dx%d）'
                      % ('%.1fcm' % dist_cm if dist_cm is not None else 'None',
                         area, det['conf'], det['w'], det['h']), flush=True)
                grab(board, args.pull_up)
                input('夹取完成，敲回车恢复原位…')
                no7.restore_travel(board, no7.GRIPPER_OPEN)
                break

            # 主动追踪：目标偏了就调 21/24 把它拉回画面中心（只调舵机、不转体）
            if abs(cx - IMG_CX) >= TRACK_DEAD_X:
                s21 = max(0, min(1000, s21 + int(TRACK_P * (IMG_CX - cx))))
            if abs(cy - IMG_CY) >= TRACK_DEAD_Y:
                s24 = max(0, min(1000, s24 + int(TRACK_P * (IMG_CY - cy))))
            board.bus_servo_set_position(0.02, [[24, s24], [21, s21]])

            print('  dist=%s area=%d 21=%d 24=%d steps=%d → 前进 %dmm'
                  % ('%.1fcm' % dist_cm if dist_cm is not None else 'None',
                     area, s21, s24, steps, args.step_mm), flush=True)
            ik.go_forward(ik.initial_pos, 2, args.step_mm, MOVE_SPEED, 1)
            steps += 1
            time.sleep(0.1)
        else:
            print('达到步数上限 %d，停止逼近' % args.max_steps, flush=True)
    finally:
        cam.camera_close()
        if depth is not None:
            try:
                depth.close()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
