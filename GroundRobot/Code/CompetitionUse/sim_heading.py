#!/usr/bin/python3
# coding=utf8
"""离线对照仿真：现在的「IMU+颜色双回路」 vs 「融合估计 + 单一控制器」。

不需要机器人、不需要摄像头，纯数学，本地直接跑：

    python3 CompetitionUse/sim_heading.py
    python3 CompetitionUse/sim_heading.py --trials 200 --turn-residual 2.0

## 它想说明什么

1. **色块像素偏移解不开**：`β = atan2(−cross, R) − e` 一个观测两个未知数，
   旧算法把它全当成横向偏差去平移，航向误差永远躺在那儿 → 一路偏出去。
2. **「纠偏后重置 IMU」确实治标不治本**：变体 B 只改控制律、仍每次归零，
   数值好看但机器人照歪；变体 C 不归零才真正收敛。
3. 三个变体共用完全相同的噪声种子，差别只来自算法。

## 模型参数都是可调的，请按现场实测量替换

尤其是 `CRAB_YAW_KICK`（横移踢航向的幅度）和 `GAIT_YAW_BIAS`（直行步态的
系统性偏转）—— 这两个是「IMU 数值乱」的主要嫌疑，但现场没实测过，
是假设值。把它们设成 0 结论依然成立（主漂移源是转弯残差 + 步态偏转）。
"""

import argparse
import math
import random

import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

def _load_fusion():
    """heading_fusion 是纯计算模块，不碰硬件。

    这里故意不走 `import agcs_lib`：agcs_lib/__init__ 会连带拉起 yaml、
    common、摄像头 SDK 那一整套，在没有机器人环境的笔记本上直接 ImportError。
    按文件路径加载，仿真在哪都能跑。
    """
    import importlib.util
    path = os.path.join(_PKG_ROOT, 'agcs_lib', 'heading_fusion.py')
    spec = importlib.util.spec_from_file_location('heading_fusion', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_fusion = _load_fusion()
LaneFusion = _fusion.LaneFusion
LaneController = _fusion.LaneController

_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi

# ---------------- 现场待标定的模型参数 ----------------
TURN_GAIN = 1.0        # 命令 15° 实际转多少倍
TURN_NOISE = 0.5       # 单次转向随机误差 σ（度）
GAIT_YAW_BIAS = 0.12   # 每走 100mm 的系统性偏转（度）—— 左右腿不对称
GAIT_YAW_NOISE = 0.5   # 每走 100mm 的随机偏转 σ（度）
CRAB_YAW_KICK = 0.4    # 每发一次横移，航向被踢多少度（六足很难纯平移）
GYRO_NOISE = 0.15      # 每小块 IMU 读数噪声 σ（度）
TURN_RESIDUAL = 1.5    # 每次转弯残留（imu_turn 只修 1° 的直接后果）
PIXEL_NOISE = 1.5      # 色块中心像素噪声 σ
P_DROP = 0.15          # 单帧丢检概率
RANGE_ERR = 25.0       # 到色块距离的估计误差 σ（mm）

F_PX = 277.0           # 320 宽画面下的等效焦距（约 60° 水平视场）
CX0 = 160.0
R_SPAWN = 1500.0       # 新色块出现时的前向距离（mm）
R_RECYCLE = 250.0      # 走过去之后换下一个

CHUNK_MM = 100.0       # 每个小块前进距离
CHUNKS_PER_SEGMENT = 10  # 每段 10 小块 = 1000mm

# 旧算法的参数（与 Auto-capture.py 保持一致）
OLD_CX_TOL = 3.0
OLD_RIGHT_MM = 7
OLD_LEFT_MM = 10
OLD_TURN_TOL = 3.0
OLD_TURN_GAIN = 0.6
OLD_TURN_MAX = 8.0


class Robot(object):
    """带噪声的真值机器人。x=横向（右正，mm），th=航向（右正，度）。"""

    def __init__(self, rng):
        self.rng = rng
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def turn(self, deg):
        self.th += deg * TURN_GAIN + self.rng.gauss(0.0, TURN_NOISE)

    def forward(self, ds):
        self.th += self.rng.gauss(GAIT_YAW_BIAS, GAIT_YAW_NOISE)
        self.x += ds * math.sin(self.th * _DEG)
        self.y += ds * math.cos(self.th * _DEG)

    def lateral(self, mm):
        self.x += mm * math.cos(self.th * _DEG)
        self.y += -mm * math.sin(self.th * _DEG)
        if mm != 0.0:
            self.th += (1.0 if mm > 0 else -1.0) * CRAB_YAW_KICK \
                + self.rng.gauss(0.0, 0.15)


def _observe(robot, r_true, rng):
    if rng.random() < P_DROP:
        return None
    beta_deg = math.atan2(-robot.x, r_true) * _RAD - robot.th
    cx = CX0 + F_PX * math.tan(beta_deg * _DEG) + rng.gauss(0.0, PIXEL_NOISE)
    return max(0.0, min(319.0, cx))


def run_old(rng, segments, chunk_mm):
    """变体 A：现状 —— 颜色回路只平移、参考点每段重取、IMU 每次转弯归零。"""
    bot = Robot(rng)
    r_true = R_SPAWN
    yaw_imu = 0.0          # 左正
    target_yaw = 0.0
    ref_cx = None
    stats = {'turn': 0, 'lat': 0, 'lat_mm': 0.0, 'max_abs_x': 0.0, 'sum_x2': 0.0, 'n': 0, 'sum_e': 0.0}

    for seg in range(segments):
        if seg > 0:
            bot.turn(0.0)                       # 转弯动作本身
            bot.th += TURN_RESIDUAL             # imu_turn 只修 1° 的残留
            yaw_imu = 0.0                       # ← reset_imu()
            target_yaw = 0.0
            ref_cx = None                       # ← 参考点也重取
        for _ in range(CHUNKS_PER_SEGMENT):
            cx = _observe(bot, r_true, rng)
            if ref_cx is None:
                if cx is not None:
                    ref_cx = cx
            elif cx is not None:
                off = cx - ref_cx
                if off > OLD_CX_TOL:
                    bot.lateral(OLD_RIGHT_MM)
                    stats['lat'] += 1
                    stats['lat_mm'] += OLD_RIGHT_MM
                elif off < -OLD_CX_TOL:
                    bot.lateral(-OLD_LEFT_MM)
                    stats['lat'] += 1
                    stats['lat_mm'] += OLD_LEFT_MM

            err = target_yaw - yaw_imu
            if err > OLD_TURN_TOL:
                step = max(1, int(round(min(err, OLD_TURN_MAX) * OLD_TURN_GAIN)))
                bot.turn(-step)
                stats['turn'] += 1
            elif err < -OLD_TURN_TOL:
                step = max(1, int(round(min(-err, OLD_TURN_MAX) * OLD_TURN_GAIN)))
                bot.turn(step)
                stats['turn'] += 1

            th_before = bot.th
            bot.forward(100.0)
            yaw_imu += -(bot.th - th_before) + rng.gauss(0.0, GYRO_NOISE)
            r_true -= 100.0
            if r_true < R_RECYCLE:
                r_true = R_SPAWN

            stats['max_abs_x'] = max(stats['max_abs_x'], abs(bot.x))
            stats['sum_x2'] += bot.x * bot.x
            stats['sum_e'] += abs(bot.th)
            stats['n'] += 1
    return bot, stats


def run_fusion(rng, segments, do_reset, rebase_cam):
    """变体 B/C：融合估计 + 单一控制器。

    do_reset   ：每次转弯后把状态归零（等价于现在的 `tracker.reset()`）
    rebase_cam ：每段开头把「当前看到的色块位置」当新参考（等价于 `ref_cx = None`）
    """
    bot = Robot(rng)
    r_true = R_SPAWN
    f = LaneFusion(f_px=F_PX, cx0=CX0)
    ctrl = LaneController()
    stats = {'turn': 0, 'lat': 0, 'lat_mm': 0.0, 'max_abs_x': 0.0, 'sum_x2': 0.0, 'n': 0, 'sum_e': 0.0}
    pending_rebase = False

    for seg in range(segments):
        if seg > 0:
            bot.turn(0.0)
            bot.th += TURN_RESIDUAL
            if do_reset:
                f.set_state(0.0, 0.0)           # ← 等价于「纠偏后重置 IMU」
            if rebase_cam:
                pending_rebase = True           # ← 等价于「参考点重取」
        for _ in range(CHUNKS_PER_SEGMENT):
            th_before = bot.th

            cx = _observe(bot, r_true, rng)
            if cx is not None:
                if pending_rebase:
                    f.cx0 = cx                  # 把「现在看到的」当中线
                    pending_rebase = False
                r_est = max(50.0, r_true + rng.gauss(0.0, RANGE_ERR))
                f.update_bearing(cx, r_est)

            turn, lat = ctrl.decide(f.heading_error, f.cross_error)
            if turn != 0.0:
                bot.turn(turn)
                stats['turn'] += 1
            elif lat != 0.0:
                bot.lateral(lat)
                stats['lat'] += 1
                stats['lat_mm'] += abs(lat)

            d_theta_imu = (bot.th - th_before) + rng.gauss(0.0, GYRO_NOISE)
            f.predict(d_theta_imu, ds_mm=100.0, lateral_mm=lat)

            bot.forward(100.0)
            r_true -= 100.0
            if r_true < R_RECYCLE:
                r_true = R_SPAWN

            stats['max_abs_x'] = max(stats['max_abs_x'], abs(bot.x))
            stats['sum_x2'] += bot.x * bot.x
            stats['sum_e'] += abs(bot.th)
            stats['n'] += 1
    return bot, stats


def summarize(stats):
    n = max(1, stats['n'])
    return {
        'end_abs_x': stats['_end_abs_x'],
        'max_abs_x': stats['max_abs_x'],
        'rms_x': math.sqrt(stats['sum_x2'] / n),
        'mean_abs_e': stats['sum_e'] / n,
        'turns': stats['turn'],
        'lats': stats['lat'],
        'lat_mm': stats['lat_mm'],
    }


def main():
    global TURN_RESIDUAL, CRAB_YAW_KICK, GAIT_YAW_BIAS, P_DROP, PIXEL_NOISE
    ap = argparse.ArgumentParser(description='IMU/颜色解耦离线对照仿真')
    ap.add_argument('--trials', type=int, default=60)
    ap.add_argument('--segments', type=int, default=6, help='直行段数')
    ap.add_argument('--turn-residual', type=float, default=TURN_RESIDUAL,
                    help='每次转弯残留的航向误差（度）')
    ap.add_argument('--crab-kick', type=float, default=CRAB_YAW_KICK,
                    help='每次横移踢航向的幅度（度）')
    ap.add_argument('--gait-bias', type=float, default=GAIT_YAW_BIAS,
                    help='每 100mm 直行系统性偏转（度）')
    ap.add_argument('--drop', type=float, default=P_DROP, help='单帧丢检概率')
    ap.add_argument('--pixel-noise', type=float, default=PIXEL_NOISE,
                    help='色块中心像素噪声 σ')
    args = ap.parse_args()

    TURN_RESIDUAL = args.turn_residual
    CRAB_YAW_KICK = args.crab_kick
    GAIT_YAW_BIAS = args.gait_bias
    P_DROP = args.drop
    PIXEL_NOISE = args.pixel_noise

    variants = [
        ('A 现状（颜色只平移 + 参考重取 + 每转归零）',
         lambda r: run_old(r, args.segments, 100)),
        ('B 只换控制律，仍「归零 + 重取参考」',
         lambda r: run_fusion(r, args.segments, True, True)),
        ('C 只去掉归零，保留相机重取参考',
         lambda r: run_fusion(r, args.segments, False, True)),
        ('D 完整方案：两者都不归零',
         lambda r: run_fusion(r, args.segments, False, False)),
    ]

    print('每变体 %d 次随机试验，%d 段 × 1000mm = %dmm 直行'
          % (args.trials, args.segments, args.segments * 1000))
    print('模型：转弯残留 %.1f°/次，横移踢航向 %.1f°，直行系统性偏转 %.2f°/100mm'
          % (TURN_RESIDUAL, CRAB_YAW_KICK, GAIT_YAW_BIAS))
    print()
    print('%-42s %9s %9s %9s %9s' % ('变体', '终点偏差', '最大偏差', 'RMS', '平均|e|'))
    print('-' * 82)

    for name, fn in variants:
        agg = {'end': 0.0, 'max': 0.0, 'rms': 0.0, 'e': 0.0, 'turns': 0, 'lats': 0, 'latmm': 0.0}
        for t in range(args.trials):
            rng = random.Random(1000 + t)
            bot, st = fn(rng)
            st['_end_abs_x'] = abs(bot.x)
            s = summarize(st)
            agg['end'] += s['end_abs_x']
            agg['max'] += s['max_abs_x']
            agg['rms'] += s['rms_x']
            agg['e'] += s['mean_abs_e']
            agg['turns'] += s['turns']
            agg['lats'] += s['lats']
            agg['latmm'] += s['lat_mm']
        n = float(args.trials)
        print('%-42s %8.0fmm %8.0fmm %8.0fmm %8.2f°'
              % (name, agg['end'] / n, agg['max'] / n, agg['rms'] / n, agg['e'] / n))
        print('%42s 转向 %4.1f 次，横移 %4.1f 次（合计 %5.0fmm）'
              % ('', agg['turns'] / n, agg['lats'] / n, agg['latmm'] / n))
    print()
    print('终点偏差 = 跑完全程后离中心线的距离，越小越好（色块越大越需要小）')


if __name__ == '__main__':
    main()
