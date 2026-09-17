#!/usr/bin/python3
# coding=utf8
"""闭环仿真：ReferenceFusion 在「方块很近、机器人走过去」的**真实**几何下会不会发散。

不碰硬件，纯数学，本地直接跑：
    python3 CompetitionUse/sim_reference2.py

真实场景：方块在 (bx, 610) 附近，机器人直行 1050mm 去够它（会逼近/越过方块）。
关键：方块前向距离 by 从 610 一路降到 0 附近，β_ref = atan2(bx, by) 会爆炸——
上一版现场 e 从 3° 飙到 149° 就是这个。验证新加的 min_ahead 守卫 + 创新门限能不能压住。
"""

import math
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def _load_fusion():
    import importlib.util
    path = os.path.join(_PKG_ROOT, 'agcs_lib', 'heading_fusion.py')
    spec = importlib.util.spec_from_file_location('heading_fusion', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_fusion = _load_fusion()
ReferenceFusion = _fusion.ReferenceFusion
LaneController = _fusion.LaneController

_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi

F_PX = 419.0
CX0 = 160.0


class Robot(object):
    def __init__(self, gait_bias=0.15):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.gait_bias = gait_bias

    def turn(self, deg):
        self.heading += deg

    def lateral(self, mm):
        self.x += mm * math.cos(self.heading * _DEG)
        self.y -= mm * math.sin(self.heading * _DEG)

    def forward(self, ds):
        self.heading += self.gait_bias
        self.x += ds * math.sin(self.heading * _DEG)
        self.y += ds * math.cos(self.heading * _DEG)


def observe(robot, block_bx, block_by):
    dx = block_bx - robot.x
    dy = block_by - robot.y
    R = math.hypot(dx, dy)
    beta_cam = math.atan2(dx, dy) - robot.heading * _DEG
    cx = CX0 + F_PX * math.tan(beta_cam)
    return cx, R


def run(block_bx, block_by, chunks, gait_bias=0.15, fixed_range=None):
    """走 chunks 块直行，返回 (es, rxs, bys)。fixed_range 非 None 时固定传给 update。"""
    robot = Robot(gait_bias=gait_bias)
    f = ReferenceFusion(f_px=F_PX, cx0=CX0)
    ctrl = LaneController(head_gain=0.9, cross_gain=0.35)
    es, rxs, bys = [], [], []
    for _ in range(chunks):
        cx, R = observe(robot, block_bx, block_by)
        rng = fixed_range if fixed_range is not None else R
        f.update_bearing(cx, rng)

        turn, lateral = ctrl.decide(f.heading_error, f.cross_error)
        if turn:
            robot.turn(turn)
        elif lateral:
            robot.lateral(lateral)

        d_theta_imu = turn + robot.gait_bias
        f.predict(d_theta_imu, ds_mm=100.0, lateral_mm=lateral)
        robot.forward(100.0)

        es.append(f.heading_error)
        rxs.append(robot.x)
        bys.append(f.by)
    return es, rxs, bys


def report(name, es, rxs, bys):
    max_e = max(abs(e) for e in es)
    flag = '❌ 发散' if max_e > 20.0 else '✅ 不发散'
    print('%-42s max|e|=%6.1f°  终点横向=%+6.1f mm  %s'
          % (name, max_e, rxs[-1], flag))
    print('    by 序列: %s' % ' '.join('%+4.0f' % b for b in bys[::2]))
    print('    e  序列: %s' % ' '.join('%+4.1f' % e for e in es[::2]))


def main():
    chunks = 11   # 1100mm（1050mm 分段 + 余量）
    print('=' * 76)
    print('方块很近（610mm），机器人直行走过去。验证 min_ahead 守卫 + 创新门限。')
    print('=' * 76)

    # 场景 1：方块正前方，机器人逼近
    es, rxs, bys = run(0.0, 610.0, chunks)
    report('方块 (0, 610)，逼近', es, rxs, bys)

    # 场景 2：方块偏右 100mm，逼近
    es, rxs, bys = run(100.0, 610.0, chunks)
    report('方块 (100, 610)，逼近', es, rxs, bys)

    # 场景 3：方块偏右 200mm，逼近
    es, rxs, bys = run(200.0, 610.0, chunks)
    report('方块 (200, 610)，逼近', es, rxs, bys)

    # 场景 4：方块实际 1150，但 marker_range 误设 610（距离估错）
    es, rxs, bys = run(100.0, 1150.0, chunks, fixed_range=610.0)
    report('方块实际(100,1150) 但 range=610（估错）', es, rxs, bys)


if __name__ == '__main__':
    main()
