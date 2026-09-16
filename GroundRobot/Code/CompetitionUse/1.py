#!/usr/bin/python3
# coding=utf8
"""1.py —— 临时测试：NO6 的寻路 + YOLO 模型的夹取。

## 跟 NO6 / NO7 差在哪

| | 寻路（走/转/航向/颜色微调） | 夹取 | 放下 |
|---|---|---|---|
| NO6 `Auto-capture.py` | 读 JSON | 读 JSON 的 pick 脉宽，摆到位就夹 | 读 JSON |
| NO7 `Auto-capture-1.py` | 读 JSON | 读 JSON 的 pick 脉宽 + 雅可比精对准 | 读 JSON |
| **本文件 `1.py`** | **读 JSON（与 NO6 完全相同）** | **不读 JSON，改用 YOLO 找目标** | **读 JSON** |

也就是说：**只读 JSON 里的行动路线（forward/back/turn_left/turn_right），不读里面的
pick**；到夹取点时不再按录制好的固定脉宽摆臂，而是让 YOLO 模型（models/best.onnx）
先找目标、转身/平移把目标居中、再走到 bbox 够大，然后才摆臂夹取。放下（place）仍然
按 JSON 执行——放下是「往已知的框里放」，不需要找目标。

## 为什么不复制 NO6/NO7 的代码

本文件用 importlib 把 `Auto-capture-1.py` 整个当模块加载进来，只把里面的 `do_pick`
和 `run_calibrate` 换成 YOLO 版本。好处：寻路那一大段（IMU 航向保持、颜色微调、
电压补偿、放下、上报、运行日志）一行都不用重写，NO7 里修好的东西 1.py 自动跟着走，
不会两边分叉。

之所以要 importlib 而不是 `import`：NO7 的文件名带连字符（`Auto-capture-1.py`），
是非法模块名，普通 import 做不到。

## 跑法

    cd /home/pi/spiderpi/CompetitionUse
    sudo systemctl stop spiderpi            # 串口独占，必须先停
    python3 1.py --color red                # 其余参数原样透传给 NO7
    python3 1.py --color red --pull-up 450  # 第一次夹取后的拔起脉宽
    python3 1.py --color red --stop-w 170   # 让 YOLO 走得更近一点再夹
    python3 1.py --color red --max-steps 0  # 只居中不前进（纯测试检测/居中）
    python3 1.py --calibrate 1              # 在 YOLO 起点姿态上标一次雅可比

## ⚠️ 必读：YOLO 靠近会把「里程」走乱

NO6/NO7 的整条路线是靠固定距离 + IMU 航向推出来的（`advance_pose` 只用于上报）。
`yolo_approach()` 是**看着画面**走的：它走了几步、实际走了多远，路线并不知道。
所以 ——

  · **朝向没问题**：每个直行段开头都会 `target_yaw = imu_state['yaw']`，
    每次转弯后都会 `reset_imu()`，都是以「当前朝向」为新基准，相对转角照常有效。
  · **距离会偏**：夹取点之后那些 forward/back 的距离基准已经不准了。夹取点离放下点
    越远，偏得越多。想减小这个偏差就调小 `--stop-w`（少走点）或 `--step-mm`。

## 夹取起点姿态是内置的，不看 JSON

`PICK_PULSES` 就是摆臂起点。数值参考原路线的夹取位，但 **21 号底座横转从原路线的
875/705（偏左）改成了 500（正前）**——因为 YOLO 是转机身把目标对到正前方的，机身
正对目标时机械臂也该朝正前伸。**这是要现场确认的**：回车夹取前用微调命令修正
（`a`/`d` 调 21，`22w`/`22s`、`23w`/`23s`、`24w`/`24s` 调其它，回车执行夹取）。
现场试出来的好值直接改本文件顶部的 `PICK_PULSES`，不用改 NO6/NO7。
"""

import argparse
import importlib.util
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


# ---------- 本文件自己的参数（改这里，或走命令行）----------

# YOLO 夹取的摆臂起点姿态，**不读 fixed_route.json**。
# 21=底座横转（500=正前、900=左90°）、22=肩、23=肘、24=腕俯仰（相机）。
# 21 用 500：YOLO 已把机身转到正对目标，臂也该朝正前伸（原路线是偏左的 875/705）。
PICK_PULSES = {
    1: {21: 500, 22: 395, 23: 350, 24: 250},
    2: {21: 500, 22: 445, 23: 470, 24: 270},
}

STOP_W = 140        # bbox 宽度到该值(像素)就算够近，停止前进（同 NO7 的 MODEL_STOP_W）
STEP_MM = 30        # YOLO 靠近时每步前进的名义距离，mm
MAX_STEPS = 40      # YOLO 靠近最多走多少步，防止一路走飞
SEARCH_TURN = 15    # 没检测到目标时左转搜索的角度
CENTER_TOL = 40     # 目标中心偏正中心多少像素以内就不左右挪

FALLBACK_TO_JSON_ON_MISS = False   # YOLO 找不到目标时，要不要退回 JSON 脉宽硬夹


def pick_posture(pick_num):
    """第 pick_num 次夹取的摆臂起点姿态（内置，与 JSON 无关）。"""
    if pick_num in PICK_PULSES:
        return dict(PICK_PULSES[pick_num])
    return dict(PICK_PULSES[2] if pick_num > 1 else PICK_PULSES[1])


def _load_no7():
    """把 Auto-capture-1.py 当模块加载（文件名带 '-'，普通 import 做不到）。"""
    path = os.path.join(_HERE, 'Auto-capture-1.py')
    if not os.path.exists(path):
        raise SystemExit('找不到 %s —— 1.py 必须和 Auto-capture-1.py 放在同一目录' % path)
    spec = importlib.util.spec_from_file_location('no7', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['no7'] = mod       # 注册进 sys.modules，模块内的自引用才正常
    spec.loader.exec_module(mod)
    return mod


no7 = _load_no7()


# ---------- 只动 21 号舵机的夹取（虫子固定在机器人左侧） ----------

SERVO21_LEFT = 875    # 21 号转到此值相机才看得到虫子（原路线 21 偏左值，需现场确认具体角度）
GRASP_MODE = 'servo21'   # 'yolo'（原：走到 bbox 够大） | 'servo21'（只动 21 号）


def _detect_pixel(model_det):
    """YOLO 检测一次，返回目标像素中心 (cx, cy)；没检测到返回 None。"""
    det = model_det.detect()
    if det is None:
        return None
    return det['x'] + det['w'] / 2.0, det['y'] + det['h'] / 2.0


def _grasp(board, pick_count, pull_up_pulse=None):
    """夹爪闭合 + 第一次夹取 22 号肩拔起 + 恢复初始姿态（不走手动微调）。"""
    board.bus_servo_set_position(2.0, [[25, no7.GRIPPER_CLOSE]])
    time.sleep(2.0)
    time.sleep(0.5)
    if pick_count == 1:
        pulse = no7.PULL_UP_22 if pull_up_pulse is None else max(0, min(1000, int(pull_up_pulse)))
        board.bus_servo_set_position(1.0, [[22, pulse]])
        time.sleep(1.0)
    no7.restore_travel(board, no7.GRIPPER_CLOSE)


def servo21_pick(board, pick_count, model_det, pull_up_pulse=None):
    """只动 21 号：先转左看虫子（虫子固定左侧），22/23/24 保持固定姿态，YOLO 确认后夹。"""
    nominal = pick_posture(pick_count)
    pulses = dict(nominal)
    pulses[21] = SERVO21_LEFT
    print('21 号转左到 %d，22/23/24 保持 %s' % (SERVO21_LEFT, nominal), flush=True)
    no7.set_servos(board, pulses, [21, 22, 23, 24])

    # 转完 21 后相机才能看到虫子，YOLO 确认
    if _detect_pixel(model_det) is None:
        print('❌ 21 转左后仍未检测到虫子，本次夹取跳过', flush=True)
        no7.restore_travel(board, no7.GRIPPER_OPEN)
        return False
    print('✅ 检测到虫子（21=%d），执行夹取' % SERVO21_LEFT, flush=True)
    _grasp(board, pick_count, pull_up_pulse)
    return True


# ---------- YOLO 靠近（照抄 NO7 的 model_approach，多了步数统计和上限）----------

def yolo_approach(board, ik, model_det, stop_w, step_mm, max_steps):
    """用 YOLO 边找边靠近：目标居中 + 前进，直到 bbox 够大。

    返回 (det, steps)；没找到 / 步数用尽返回 (None, steps)。
    与 NO7 `model_approach` 的区别：返回实际走了几步（里程偏差好估），且 max_steps
    可以传 0 —— 那就只做居中不前进，用来单独验证「检测 + 居中准不准」。
    """
    steps = 0
    for i in range(max(1, max_steps)):
        det = model_det.detect()
        if det is None:
            print('  未检测到目标，左转 %d° 搜索' % SEARCH_TURN, flush=True)
            ik.turn_left(ik.initial_pos, 2, SEARCH_TURN, no7.TURN_SPEED, 1)
            time.sleep(0.4)
            continue

        cx = det['x'] + det['w'] / 2.0
        w = det['w']
        print('  检测 #%d cx=%.1f w=%d conf=%.2f'
              % (i, cx, w, det.get('conf', 0.0)), flush=True)
        if w >= stop_w:
            print('  目标已够近（w=%d ≥ %d），停止靠近' % (w, stop_w), flush=True)
            return det, steps

        if cx < 320 - CENTER_TOL:
            ik.left_move(ik.initial_pos, 2, no7.LEFT_CORRECT_MM, no7.MOVE_SPEED, 1)
            print('  目标偏左，左移 %dmm' % no7.LEFT_CORRECT_MM, flush=True)
        elif cx > 320 + CENTER_TOL:
            ik.right_move(ik.initial_pos, 2, no7.COLOR_CORRECT_MM, no7.MOVE_SPEED, 1)
            print('  目标偏右，右移 %dmm' % no7.COLOR_CORRECT_MM, flush=True)

        if steps < max_steps:
            ik.go_forward(ik.initial_pos, 2, step_mm, no7.MOVE_SPEED, 1)
            steps += 1
            print('  前进第 %d 步（名义 %dmm）' % (steps, step_mm), flush=True)
        time.sleep(0.1)

    print('  靠近步数用尽（%d 步），仍未到目标' % steps, flush=True)
    if max_steps <= 0:
        return model_det.detect(), steps     # 只居中模式：把当前检测结果交给上层
    return None, steps


# ---------- 替换 NO7 的 do_pick / run_calibrate ----------

def yolo_do_pick(board, pick_count, pulses, model_det, calib, pull_up_pulse=None):
    """替换 NO7 的 `do_pick`：不按 JSON 脉宽摆臂，改用 YOLO 找目标再夹。

    NO7 的 main() 以完全相同的签名调用本函数，所以寻路/放下/上报一行都不用改。
    传进来的 `pulses` 是 JSON 的 pick 脉宽 —— 本函数**故意不用它**（这就是本文件的
    全部意义），只在失败回退时才会碰。
    """
    nominal = pick_posture(pick_count)
    print('=' * 64, flush=True)
    print('第 %d 次夹取：走 YOLO（忽略 JSON 的 pick 脉宽 %s）' % (pick_count, pulses),
          flush=True)
    print('YOLO 摆臂起点姿态：%s' % nominal, flush=True)

    # 只动 21 号模式：先转左看虫子再夹，跳过「走到 bbox 够大 + 固定姿态 + 手动微调」
    if GRASP_MODE == 'servo21':
        servo21_pick(board, pick_count, model_det, pull_up_pulse)
        return

    # do_pick 的签名里没有 ik，但 YOLO 靠近要 ik 才能走/转。IK 不持有状态，
    # 只按 initial_pos 算完脉宽发给 board，所以这里另建一个和 main 里那个并存没问题。
    ik = no7.make_ik(board)

    det, steps = yolo_approach(board, ik, model_det,
                               args.stop_w, args.step_mm, args.max_steps)
    if det is None:
        print('❌ YOLO 没找到目标，本次夹取跳过', flush=True)
        if not FALLBACK_TO_JSON_ON_MISS or not pulses:
            print('   （未回退到 JSON 脉宽：回退等于把测试又拉回固定路线，'
                  '结论会被污染）', flush=True)
            return
        print('⚠️ 回退到 JSON 脉宽硬夹 %s' % pulses, flush=True)
        nominal = dict(pulses)

    if det is not None:
        print('✅ YOLO 锁定目标：cx=%.1f w=%d conf=%.2f，靠近走了 %d 步'
              % (det['x'] + det['w'] / 2.0, det['w'], det.get('conf', 0.0), steps),
              flush=True)
        if steps:
            print('   注意：这 %d 步是「看着画面」走的，名义 %dmm，实际距离未知；'
                  '夹取点之后的路线里程基准已偏。' % (steps, steps * args.step_mm),
                  flush=True)

    # 摆臂：先 21，再 22-23-24（顺序沿用 NO7，别改——印错顺序会撞）
    if pick_count == 1:
        state = no7.pick1_prepare(board, nominal)
    else:
        state = no7.pick2_prepare(board, nominal)

    # 雅可比精对准：需要 calib_pick<N>.json。机器人上目前**没有**这个文件，
    # 所以这一步会跳过，直接进手动微调。想启用就先跑一次 --calibrate。
    if calib is not None:
        print('阶段二：模型 + 雅可比精对准', flush=True)
        if not no7.fine_align(board, model_det, calib, state):
            print('精对准未完全居中，进入手动微调', flush=True)
    else:
        print('无 calib_pick%d.json，跳过雅可比精对准（可先跑 python3 1.py --calibrate %d）'
              % (pick_count, pick_count), flush=True)

    # 手动微调 + 回车夹取（含 22 号肩拔起），与 NO6/NO7 一致
    no7.arm_fine_tune(board, state, 'pick', pull_up=(pick_count == 1),
                      pull_up_pulse=pull_up_pulse)


def yolo_run_calibrate(board, model_det, pick_num, actions):
    """替换 NO7 的 `run_calibrate`：在 YOLO 起点姿态上标定雅可比。

    NO7 原版是拿 JSON 的 pick 脉宽当标称姿态标的。既然夹取已经不用 JSON 脉宽了，
    标定也必须落在同一姿态上，否则雅可比是在别的姿态下量的，精对准会用错。
    """
    nominal = pick_posture(pick_num)
    print('在第 %d 次 YOLO 夹取起点姿态上标定：%s' % (pick_num, nominal), flush=True)
    print('请确认：目标已放在夹取位置，机械臂将先摆到该姿态。', flush=True)
    no7.set_servos(board, nominal, no7.ARM_SERVOS)
    time.sleep(1.0)

    J, anchor, ref_size = no7.calibrate_jacobian(board, model_det, nominal)
    data = {
        'pick_index': None,          # 已与 JSON 的 index 脱钩
        'nominal_pulses': {str(k): int(v) for k, v in nominal.items()},
        'anchor': {'cx': round(anchor[0], 2), 'cy': round(anchor[1], 2)},
        'ref_size': {'w': round(ref_size[0], 2), 'h': round(ref_size[1], 2)},
        'jacobian': [[round(v, 4) for v in row] for row in J],
    }
    no7.save_calib(pick_num, data)
    print('标定完成，已保存：%s' % no7.calib_path(pick_num), flush=True)


# ---------- 入口 ----------

args = None      # 本文件自己的参数，main() 里填；yolo_do_pick 要用


def main():
    global args
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--stop-w', type=int, default=STOP_W,
                    help='YOLO 靠近的停止阈值，bbox 宽度(像素)到该值就停（默认 %d）' % STOP_W)
    ap.add_argument('--step-mm', type=int, default=STEP_MM,
                    help='YOLO 靠近每步前进的名义距离 mm（默认 %d）' % STEP_MM)
    ap.add_argument('--max-steps', type=int, default=MAX_STEPS,
                    help='YOLO 靠近最多走几步；传 0 = 只居中不前进（默认 %d）' % MAX_STEPS)
    ap.add_argument('-h', '--help', action='store_true', help='显示本文件参数 + NO7 参数')
    args, rest = ap.parse_known_args()

    if args.help:
        print(__doc__)
        print('=' * 64)
        print('本文件自己的参数见上；下面这些是透传给 NO7 (Auto-capture-1.py) 的：')
        sys.argv = [sys.argv[0], '-h']
        try:
            no7.main()
        except SystemExit:
            pass
        return

    # 把 1.py 自己的参数摘掉，剩下的原样交给 NO7 的 main() 去 parse
    sys.argv = [sys.argv[0]] + rest

    no7.do_pick = yolo_do_pick
    no7.run_calibrate = yolo_run_calibrate

    print('=' * 64, flush=True)
    print('1.py：NO6 寻路 + YOLO 夹取（忽略 JSON 的 pick，place 仍按 JSON）', flush=True)
    print('YOLO 靠近参数：stop_w=%d step_mm=%d max_steps=%d'
          % (args.stop_w, args.step_mm, args.max_steps), flush=True)
    print('摆臂起点姿态：pick1=%s pick2=%s' % (pick_posture(1), pick_posture(2)), flush=True)
    print('=' * 64, flush=True)

    no7.main()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
