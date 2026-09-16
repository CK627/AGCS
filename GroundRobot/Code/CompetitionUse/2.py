#!/usr/bin/python3
# coding=utf8
"""2.py —— 深度相机 + 手眼标定的夹取（不走路线）。

跑法（在机器人上）：
    sudo systemctl stop spiderpi
    cd ~/spiderpi/CompetitionUse
    python3 2.py --dry-run       # 先只算坐标不移动，验证 (x,y,z) 对不对
    python3 2.py                 # 算坐标 -> IK 移动夹爪 -> 夹取
    python3 2.py --conf 0.4      # 检测不到先降置信度门槛

流程：
    1. 不恢复原位，先转 21 到虫子左侧（--s21），再扫 24 俯仰，YOLO 找虫子
    2. 检测到 → 读深度 → 深度相机坐标 → cam2arm 换算到机械臂 (x,y,z)
    3. IK 把夹爪移到 (x,y,z) → 闭合夹爪 → 拔起 22 → 敲回车恢复原位

坐标系约定（与 calib_cam2arm.py / grab.py 一致）：
    深度相机：X 右 / Y 上 / Z 前，mm（OpenNI2 工厂标定）
    彩色相机：X 右 / Y 下 / Z 前，mm（OpenCV/ArUco 约定）—— 深度→彩色只翻转 Y
    机械臂  ：x 右 / y 前 / z 上，cm，原点 = 云台中心地面投影
    彩色→机械臂用 config/cam2arm.yaml 的 R/t；深度/彩色约 25mm 基线按「同像素」粗对齐忽略。
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

from agcs_lib import make_board, make_arm_ik, DepthCamera

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


def load_cam2arm():
    """读 config/cam2arm.yaml 的 R(3x3) / t(3x1)，单位 mm。"""
    import yaml
    path = os.path.join(_PKG_ROOT, 'config', 'cam2arm.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    c = data['cam2arm']
    R = np.array(c['R'], dtype=np.float64).reshape(3, 3)
    t = np.array(c['t'], dtype=np.float64).reshape(3, 1)
    return R, t


def _depth_median(d, cx, cy, r=2):
    """取 (cx,cy) 附近深度中位数（中心无效时往邻域扩），返回 (z_mm, xi, yi) 或 (None, xi, yi)。"""
    xi, yi = int(round(cx)), int(round(cy))
    h, w = d.shape
    for radius in range(0, r + 1):
        vals = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                yy, xx = yi + dy, xi + dx
                if 0 <= xx < w and 0 <= yy < h:
                    v = int(d[yy, xx])
                    if v > 0:
                        vals.append(v)
        if vals:
            vals.sort()
            return vals[len(vals) // 2], xi, yi
    return None, xi, yi


def grab(board, pull_up_pulse):
    """夹取（复刻 1.py 夹取开始到夹取结束：闭合夹爪 25 → 拔起 22）。不恢复原位，恢复交给调用方。"""
    board.bus_servo_set_position(2.0, [[25, no7.GRIPPER_CLOSE]])
    time.sleep(2.0)
    time.sleep(0.5)
    pulse = max(0, min(1000, int(pull_up_pulse)))
    board.bus_servo_set_position(1.0, [[22, pulse]])
    time.sleep(1.0)


def depth_to_arm(depth_cam, R, t, det, z_offset):
    """检测框 → 深度 → 机械臂 (x,y,z) cm。失败返回 None。"""
    cx = det['x'] + det['w'] / 2.0
    cy = det['y'] + det['h'] / 2.0

    d = depth_cam.read_depth(timeout_ms=2000)
    if d is None:
        print('❌ 读深度失败', flush=True)
        return None

    hd, wd = d.shape
    # 彩色是 640×480，深度可能是别的分辨率 → 按比例缩放彩色像素到深度像素
    sx = wd / 640.0
    sy = hd / 480.0
    dx, dy = cx * sx, cy * sy

    nz = int((d > 0).sum())
    bxx, byy = min(wd - 1, int(round(dx))), min(hd - 1, int(round(dy)))
    print('深度图 %s 有效 %d/%d(%.0f%%) 中心(%d,%d)=%d bbox(%d,%d)=%d' %
          ((hd, wd), nz, hd * wd, 100.0 * nz / (hd * wd),
           wd // 2, hd // 2, int(d[hd // 2, wd // 2]),
           bxx, byy, int(d[byy, bxx])), flush=True)

    z_mm, xi, yi = _depth_median(d, dx, dy)
    if z_mm is None or z_mm <= 0:
        print('❌ 深度像素(%d,%d)附近深度无效' % (xi, yi), flush=True)
        return None

    w = depth_cam.depth_to_world(float(xi), float(yi), float(z_mm))
    if w is None:
        print('❌ depth_to_world 转换失败', flush=True)
        return None
    wx, wy, wz = w

    # 深度相机(X右,Y上,Z前) → 彩色相机(X右,Y下,Z前)：只翻转 Y
    cam = np.array([wx, -wy, wz], dtype=np.float64).reshape(3, 1)
    # 彩色相机 → 机械臂(mm)，再转 cm
    arm_mm = R @ cam + t
    arm_cm = arm_mm.flatten() / 10.0

    print('深度像素(%d,%d)=%dmm → 相机(%.0f,%.0f,%.0f)mm → 机械臂(%.1f,%.1f,%.1f)cm'
          % (xi, yi, z_mm, wx, wy, wz, arm_cm[0], arm_cm[1], arm_cm[2]), flush=True)

    x = float(arm_cm[0])
    y = float(arm_cm[1])
    z = float(arm_cm[2]) + float(z_offset)
    return (x, y, z)


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
    ap.add_argument('--s21', type=int, default=SERVO21_POS,
                    help='21 号转到的左侧虫子位置（默认 %(default)d）')
    ap.add_argument('--pull-up', type=int, default=no7.PULL_UP_22,
                    help='夹取后 22 号肩拔起脉宽（默认 %(default)d）')
    ap.add_argument('--tries', type=int, default=3,
                    help='24 号俯仰扫几轮（默认 %(default)d）')
    ap.add_argument('--dwell', type=float, default=1.0,
                    help='每个 24 角度停留秒（默认 %(default)s）')
    ap.add_argument('--z-offset', type=float, default=0.0,
                    help='夹取高度偏移 cm（加到算出的 z 上，默认 %(default)s）')
    ap.add_argument('--dry-run', action='store_true',
                    help='只算 (x,y,z) 打印，不移动机械臂')
    args = ap.parse_args()

    board = make_board()
    ak = make_arm_ik(board)
    R, t = load_cam2arm()

    depth_cam = DepthCamera()
    depth_cam.open()
    depth_cam.start_depth()
    print('深度相机已打开', flush=True)

    # 不恢复原位；先转 21 到虫子左侧（0.5s 到位 + 0.5s 静置），再扫 24 俯仰
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
    print('2.py：转 21=%d → 扫 24 俯仰 → 检测到 → 深度定位 → IK 夹取' % args.s21, flush=True)
    print('conf=%.2f 扫 %d 轮 / 每角度 %.1fs%s'
          % (args.conf, args.tries, args.dwell,
             '  [--dry-run 只算坐标]' if args.dry_run else ''), flush=True)
    print('=' * 64, flush=True)

    try:
        for attempt in range(1, args.tries + 1):
            for p24 in SERVO24_SCAN:
                board.bus_servo_set_position(0.5, [[24, p24]])
                time.sleep(args.dwell)
                det = model_det.detect()
                if det is None:
                    print('  24=%d 未检测到（第 %d/%d 轮）' % (p24, attempt, args.tries), flush=True)
                    continue

                print('✅ 21=%d 24=%d 检测到虫子 conf=%.2f，深度定位' % (args.s21, p24, det['conf']), flush=True)
                target = depth_to_arm(depth_cam, R, t, det, args.z_offset)
                if target is None:
                    print('  ❌ 深度定位失败，继续扫', flush=True)
                    continue

                if args.dry_run:
                    print('--dry-run：已算出目标，不移动机械臂，退出', flush=True)
                    return

                # IK 把夹爪末端移到 (x,y,z)，alpha=0 夹持器水平
                res = ak.setPitchRangeMoving(target, 0, -90, 100, 1.0)
                if res is False:
                    print('❌ IK 无解，无法移到 (%s)' % (target,), flush=True)
                    return
                if abs(res[1]) > 0.5:
                    print('⚠️ IK 俯仰降级 alpha=%.1f°，仍尝试夹取' % res[1], flush=True)
                time.sleep(1.0)

                grab(board, args.pull_up)
                input('夹取完成，保持夹取姿态。敲回车恢复原位…')
                no7.restore_travel(board, no7.GRIPPER_OPEN)
                print('已恢复原位，退出', flush=True)
                return

            print('第 %d 轮 24 号俯仰扫完未找到' % attempt, flush=True)

        print('❌ 扫完 24 号俯仰都没检测到虫子，恢复原位后退出', flush=True)
        no7.restore_travel(board, no7.GRIPPER_OPEN)
    finally:
        depth_cam.close()
        cam.camera_close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
