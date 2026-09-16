#!/usr/bin/python3
# coding=utf8
"""离散 bang-bang vs 连续航向保持：为什么「走 100mm→停→转 1°」一定收敛不了。

三个变体共用同一个被控对象（同一随机种子、同一扰动序列），只换控制器：

  A 现状   ：go_forward 开环走 100mm → 停 → 读 IMU → 转 round(0.6*e)°（死区 3°）
  B 连续   ：25Hz 持续下发 rotation = clamp(kp*e + kd*ė, ±1)（不停、不重启）
  C 标定+连续：同 B，但先把机械零位标定好（系统偏差 bias=0）

想说明的两件事：
  1. 离散方案不是「调参没调好」，是结构上就留了一段 100mm 的开环窗口，
     误差在窗口里自由增长，而停-启动本身还额外注入一次随机偏航。
  2. 标定（前馈）负责干掉偏差的「均值」，闭环负责压住「方差」——
     只做闭环能收敛但会一直抖，只做标定会漂，两个都要。

跑法：
    python3 CompetitionUse/sim_gait_hold.py --trials 200
"""

import argparse
import math
import random


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


class Plant:
    """六足直线行走的极简被控对象。

    yaw 的扰动分三部分：
      bias   —— 系统性偏航（°/s），来自 6 条腿零位不对称 / 重心偏 / 足端打滑方向性。
                **标定能消掉它**。它通常小于死区，所以离散方案根本看不见它，
                看不见就会一路累积 —— 这就是「先直后偏」。
      d      —— OU 有色噪声（°/s），步态周期里的机身摆动，相关时间 tau。
                闭环只能压它的方差，压不掉均值。
      kick   —— 每次停-启动注入的随机偏航（°），只在离散方案里出现。
    """

    def __init__(self, rng, bias_dps, noise_dps, v_mm_s=50.0, hz=100.0,
                 stop_kick_deg=0.5, tau=0.35):
        self.rng = rng
        self.bias = bias_dps
        self.sigma = noise_dps
        self.tau = tau
        self.v = v_mm_s
        self.dt = 1.0 / hz
        self.stop_kick = stop_kick_deg
        self.d = 0.0
        self.yaw = 0.0
        self.x = 0.0          # 横向偏差 mm，右正
        self.s = 0.0          # 已走距离 mm
        self.rate = 0.0

    def tick(self, u_dps=0.0, moving=True):
        """推进一个 dt。u_dps = 控制器下发的偏航角速度。"""
        # OU：d <- d - d/tau*dt + sigma*sqrt(2/tau)*sqrt(dt)*N(0,1)
        self.d += (-self.d / self.tau) * self.dt + \
            self.sigma * math.sqrt(2.0 * self.dt / self.tau) * self.rng.gauss(0.0, 1.0)
        self.rate = u_dps + self.bias + self.d
        self.yaw = wrap180(self.yaw + self.rate * self.dt)
        if moving:
            ds = self.v * self.dt
            self.s += ds
            self.x += math.sin(math.radians(self.yaw)) * ds
        return self.yaw

    def stop_start_shock(self):
        """停-启动一次：机身惯量 + 足端重新咬合注入一次随机偏航。"""
        if self.stop_kick:
            self.yaw = wrap180(self.yaw + self.rng.gauss(0.0, self.stop_kick))


def run_discrete(rng, total_mm, chunk_mm=100.0,
                 tol_deg=3.0, gain=0.6, max_step=8.0,
                 v=50.0, bias=0.8, noise=3.0, stop_kick=0.5):
    """变体 A：现状。开环走 chunk_mm → 停 → 修正一次。"""
    p = Plant(rng, bias, noise, v_mm_s=v, stop_kick_deg=stop_kick)
    turns = 0
    sum_e2 = 0.0
    n = 0
    max_abs_e = 0.0

    while p.s < total_mm:
        # 开环走一段（这段里 SDK 不接受任何反馈）
        t_end = p.s + chunk_mm
        while p.s < t_end:
            e = -p.yaw
            sum_e2 += e * e
            n += 1
            max_abs_e = max(max_abs_e, abs(e))
            p.tick(u_dps=0.0)

        # 停下
        p.stop_start_shock()
        e = -p.yaw
        if abs(e) > tol_deg:
            step = clamp(gain * abs(e), 1.0, max_step)
            step = round(step)                       # 舵机角度是整数度
            p.yaw = wrap180(p.yaw + math.copysign(step, e))
            turns += 1

        # 重新启动
        p.stop_start_shock()
        for _ in range(int(0.25 / p.dt)):            # 转身耗时 0.25s，期间扰动继续
            p.tick(u_dps=0.0, moving=False)

    return dict(final_e=abs(p.yaw), final_x=abs(p.x),
                rms_e=math.sqrt(sum_e2 / max(1, n)),
                max_e=max_abs_e, turns=turns)


def run_continuous(rng, total_mm, hz=25.0, kp=1.2, kd=0.08,
                   rot_max_dps=90.0, deadband=0.3,
                   v=50.0, bias=0.8, noise=3.0, stop_kick=0.5):
    """变体 B/C：25Hz 持续下发 rotation，机器人不停车。"""
    p = Plant(rng, bias, noise, v_mm_s=v, stop_kick_deg=stop_kick)
    sum_e2 = 0.0
    n = 0
    max_abs_e = 0.0
    prev_e = 0.0
    upd = 1.0 / hz
    since_upd = 0.0
    u = 0.0

    while p.s < total_mm:
        e = -p.yaw
        if abs(e) < deadband:
            u_cmd = 0.0
        else:
            u_cmd = clamp(kp * e + kd * (e - prev_e) / max(1e-6, p.dt),
                          -rot_max_dps, rot_max_dps)
        if since_upd >= upd:
            u = u_cmd
            prev_e = e
            since_upd = 0.0
        since_upd += p.dt

        sum_e2 += e * e
        n += 1
        max_abs_e = max(max_abs_e, abs(e))
        p.tick(u_dps=u)

    return dict(final_e=abs(p.yaw), final_x=abs(p.x),
                rms_e=math.sqrt(sum_e2 / max(1, n)),
                max_e=max_abs_e, turns=0)


def main():
    ap = argparse.ArgumentParser(description='离散 vs 连续航向保持对照仿真')
    ap.add_argument('--trials', type=int, default=200)
    ap.add_argument('--distance', type=float, default=6000.0,
                    help='单次试验的总行程 mm')
    ap.add_argument('--bias', type=float, default=0.8,
                    help='系统性偏航 °/s（标定前；典型 0.5~1.5，通常小于死区所以看不见）')
    ap.add_argument('--noise', type=float, default=3.0, help='OU 噪声稳态标准差 °/s')
    ap.add_argument('--stop-kick', type=float, default=0.5,
                    help='每次停-启动注入的随机偏航 °（设为 0 可关掉这项）')
    ap.add_argument('--no-stop-kick', action='store_true',
                    help='等价于 --stop-kick 0：假设停-启动不引入扰动')
    args = ap.parse_args()

    if args.no_stop_kick:
        args.stop_kick = 0.0

    variants = [
        ('A 现状（走100mm→停→转round(0.6e)°，死区3°）',
         lambda r: run_discrete(r, args.distance, bias=args.bias,
                                noise=args.noise, stop_kick=args.stop_kick)),
        ('B 连续航向保持 25Hz（未标定）',
         lambda r: run_continuous(r, args.distance, bias=args.bias,
                                  noise=args.noise)),
        ('C 标定后 + 连续航向保持',
         lambda r: run_continuous(r, args.distance, bias=0.0,
                                  noise=args.noise)),
    ]

    print('行程 %.0f mm/次，%d 次试验；扰动 bias=%.1f°/s noise=%.1f°/s '
          'stop_kick=%.1f°'
          % (args.distance, args.trials, args.bias, args.noise, args.stop_kick))
    print()
    print('%-42s %9s %9s %9s %8s' % ('变体', '终点|e|°', '终点|横偏|mm', 'RMS|e|°', '峰值|e|°'))
    print('-' * 82)

    results = []
    for name, fn in variants:
        acc = {'final_e': 0.0, 'final_x': 0.0, 'rms_e': 0.0, 'max_e': 0.0}
        rng = random.Random(20260916)
        for _ in range(args.trials):
            r = fn(rng)
            for k in acc:
                acc[k] += r[k]
        for k in acc:
            acc[k] /= args.trials
        results.append((name, acc))
        print('%-42s %9.2f %9.1f %9.2f %8.2f'
              % (name, acc['final_e'], acc['final_x'], acc['rms_e'], acc['max_e']))

    print()
    a = results[0][1]['final_x']
    for name, acc in results[1:]:
        if acc['final_x'] > 1e-9:
            print('%s 的终点横偏是现状的 %.2f%%（越小越好）'
                  % (name.split()[0], 100.0 * acc['final_x'] / max(1e-9, a)))
    print()
    print('想验证「离散不是调参问题」：把死区/增益改到任意值，A 都降不下去，')
    print('因为它每 100mm 有一段完全开环的窗口；--no-stop-kick 可以关掉停-启动扰动看另一半原因。')


if __name__ == '__main__':
    main()
