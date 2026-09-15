#!/usr/bin/python3
# coding=utf8
"""步幅体检：用深度相机当尺子，量「走一步实际走了多远」。

## 为什么需要它

官方 demo（`spiderpi_sdk/common_sdk/common/kinematics_control_demo.py:22-23`）
把 `go_forward` 的参数写明了：

    参数3：步幅，单位mm （转弯时为角度，单位度）
    参数4：速度，单位mm/s
    参数5：执行次数，单位0时表示无限循环

所以 `go_forward(pos, 2, 100, 50, 1)` = **执行 1 次、单次步幅 100mm**。

注意参数 3 是「**步幅**」不是「距离」：每个周期腿要在 `100mm ÷ 50mm/s = 2s`
内把 100mm 迈完，实际迈出多少取决于舵机有没有在预算时间内转到位、以及落地时
脚滑不滑。**开环，天然不可重复** —— 这就是「每次走动的距离都差一点」的根源。

本仓库早有旁证：`Auto-capture.py` 的 `extra_distance_mm()` 专门补偿「低电压时
走不到标称距离」，`agcs_lib/motion.py` 也写明 `move_body_xyz`「比步态走
（go_forward）快且精确」。

## 怎么量

机器人自己不知道地面走了多远，但**深度相机可以当尺子**：正前方放一个平面，
读画面中心深度 → 走一段 → 再读一次，差值就是真实位移。

    python3 stride_probe.py                      # 前进 5 × 100mm
    python3 stride_probe.py --dir back           # 后退 5 × 100mm
    python3 stride_probe.py --chunks 8 --speed 50

**跑之前：机器人正前方约 1 米放个箱子 / 纸板 / 靠墙**，深度相机要有东西可测。
本脚本只读深度、不会撞上去（深度小于 `--min-depth` 就停）。

## 怎么读

    · 每步实际值都稳定、但系统性小于命令值（比如都在 88~92mm）
      → 步态本身短路，是**标称步幅虚高**，属于系统误差，可以整体放大命令值补偿。
    · 每步实际值上下散（比如 84 / 99 / 91 / 103）
      → 是**打滑/腿相位**造成的随机误差，单靠放大命令值补不了，只能闭环。
    · 脚本打印的「本电压下路线脚本会补」跟历史日志里的补偿值对不上
      → 电压读数不可比，先解决取样时机。
"""

import argparse
import sys
import time

sys.path.insert(0, '/home/pi/spiderpi')

import numpy as np   # noqa: E402

LOW_VOLTAGE = 11.3
VOLTAGE_EXTRA_MIN = 40
VOLTAGE_EXTRA_MAX = 80
VOLTAGE_EXTRA_LOW_V = 10.3


def extra_distance_mm(voltage):
    """与 Auto-capture.py 完全一致的补偿曲线，用来对照日志。"""
    if voltage >= LOW_VOLTAGE:
        return 0
    if voltage <= VOLTAGE_EXTRA_LOW_V:
        return VOLTAGE_EXTRA_MAX
    ratio = (LOW_VOLTAGE - voltage) / (LOW_VOLTAGE - VOLTAGE_EXTRA_LOW_V) * 0.75
    return int(VOLTAGE_EXTRA_MIN + (VOLTAGE_EXTRA_MAX - VOLTAGE_EXTRA_MIN) * ratio)


def read_voltage(board, samples=15, interval=0.03):
    """连续采样电池电压，返回中位数（伏）。"""
    vals = []
    for _ in range(samples):
        try:
            v = board.get_battery()
            if v is not None:
                vals.append(int(v))
        except Exception:
            pass
        time.sleep(interval)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2] / 1000.0


def center_depth(cam, half=40, tries=3):
    """取画面中心方块的深度中位数（mm），无效返回 None。

    只统计合理量程内的像素，避开洞（0）和远端噪声。
    """
    for _ in range(tries):
        frame = cam.read_depth()
        if frame is None:
            time.sleep(0.1)
            continue
        h, w = frame.shape
        roi = frame[max(0, h // 2 - half):h // 2 + half,
                    max(0, w // 2 - half):w // 2 + half]
        valid = roi[(roi > 200) & (roi < 6000)]
        if valid.size >= 50:
            return float(np.median(valid))
    return None


def main():
    ap = argparse.ArgumentParser(description='步幅体检（深度相机当尺子）')
    ap.add_argument('--chunks', type=int, default=5, help='走几段，默认 5')
    ap.add_argument('--mm', type=int, default=100, help='每段标称步幅，默认 100')
    ap.add_argument('--speed', type=int, default=50, help='速度 mm/s，默认 50（同 MOVE_SPEED）')
    ap.add_argument('--dir', choices=['forward', 'back'], default='forward',
                    help='前进还是后退，默认 forward')
    ap.add_argument('--pitch', type=int, default=330,
                    help='24 号腕俯仰脉宽，默认 330（同 restore_travel，相机朝前）')
    ap.add_argument('--min-depth', type=int, default=350,
                    help='深度小于它就停，避免撞上，默认 350mm')
    args = ap.parse_args()

    from agcs_lib import make_board, make_ik, DepthCamera

    board = make_board()
    board.enable_reception()
    ik = make_ik(board)

    # 同 restore_travel：把机械臂收起来，免得走路时拖地/挡视野
    board.bus_servo_set_position(1.5, [[22, 705], [23, 90], [21, 500], [24, args.pitch]])
    time.sleep(1.5)

    cam = DepthCamera()
    cam.open()
    cam.start_depth()
    time.sleep(1.0)

    d0 = center_depth(cam)
    if d0 is None:
        print('=' * 60)
        print('深度相机在画面中心读不到有效深度。')
        print('**请在机器人正前方约 1 米处放一个箱子 / 纸板，或者让它正对一面墙**，')
        print('然后重跑。（相机在机械臂上，`--pitch` 可调俯仰，默认 330）')
        print('=' * 60)
        cam.close()
        return

    v0 = read_voltage(board)
    print('=' * 60)
    print('初始：中心深度 %.0fmm　电压 %s'
          % (d0, ('%.3fV' % v0) if v0 else '读不到'))
    if v0:
        print('（本电压下 Auto-capture.py 会做 %.0fmm 的距离补偿）' % extra_distance_mm(v0))
    print('方向 %s　标称步幅 %dmm × %d 段　速度 %dmm/s'
          % (args.dir, args.mm, args.chunks, args.speed))
    print('-' * 60)
    print('%6s  %12s  %12s' % ('段', '实际位移', '与标称差'))

    fn = ik.go_forward if args.dir == 'forward' else ik.back
    sign = 1.0 if args.dir == 'forward' else -1.0
    prev = d0
    actuals = []
    aborted = None

    for k in range(args.chunks):
        fn(ik.initial_pos, 2, args.mm, args.speed, 1)
        time.sleep(0.6)                     # 等机身晃动静下来再读深度
        d = center_depth(cam)
        if d is None:
            print('  第 %d 段后读不到深度，停止' % (k + 1))
            aborted = 'depth-lost'
            break
        if d < args.min_depth:
            print('  第 %d 段后深度只剩 %.0fmm，太近了，停止以免撞上' % (k + 1, d))
            aborted = 'too-close'
            break
        moved = (prev - d) * sign            # 前进时深度变小 = 走了正距离
        actuals.append(moved)
        print('%6d  %+11.1fmm  %+11.1fmm' % (k + 1, moved, moved - args.mm), flush=True)
        prev = d

    v1 = read_voltage(board)
    cam.close()

    print('-' * 60)
    if not actuals:
        print('没量到任何一段。')
        print('=' * 60)
        return

    mean = sum(actuals) / len(actuals)
    print('实际平均 %.1fmm / 标称 %dmm　→ 比例 %.3f'
          % (mean, args.mm, mean / args.mm))
    print('离散：最小 %.1f　最大 %.1f　极差 %.1fmm'
          % (min(actuals), max(actuals), max(actuals) - min(actuals)))
    print('累计：标称 %dmm　实际 %.1fmm　差 %+.1fmm'
          % (args.mm * len(actuals), sum(actuals), sum(actuals) - args.mm * len(actuals)))
    if v0 and v1:
        print('电压：%.3fV → %.3fV' % (v0, v1))
    print('=' * 60)
    print('怎么读：')
    print('  · 每步都稳定、但系统性偏小/偏大（极差只有几 mm）')
    print('    → 是**标称步幅虚高**，系统误差。按上面的「比例」整体放大命令值即可补。')
    print('  · 每步上下散（极差十几~几十 mm）')
    print('    → 是**打滑/腿相位**的随机误差，放大命令值补不了，只能闭环')
    print('      （走完再用深度/色块量一次离目标多远，差多少补多少）。')
    print('  · 若极差大而比例≈1 → 平均没错、单次不准，同样是随机误差。')
    if aborted:
        print('（本次提前结束：%s，样本只有 %d 段）' % (aborted, len(actuals)))


if __name__ == '__main__':
    main()
