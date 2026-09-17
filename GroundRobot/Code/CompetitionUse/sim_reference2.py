#!/usr/bin/python3
# coding=utf8
"""闭环仿真：ReferenceFusion（B 方案） vs LaneFusion(cx0 hack) 谁会把机器人带偏。

不碰硬件，纯数学，本地直接跑：
    python3 CompetitionUse/sim_reference2.py

场景：方块放在路线**外侧** (bx, 3000)，机器人沿 +y 直行 1500mm，每块 100mm。
机器人有「每块系统偏转」gait_bias（模拟左右腿不对称），IMU 测到实际转角。

对比三件事：
1. ReferenceFusion 会不会发散（上一版 e 飙 -142° 就是发散）；
2. ReferenceFusion 会不会被方块吸过去（LaneFusion cx0 hack 的病）；
3. 谁的终点横向偏差更小。
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
LaneFusion = _fusion.LaneFusion
ReferenceFusion = _fusion.ReferenceFusion
LaneController = _fusion.LaneController

_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi

F_PX = 419.0
CX0 = 160.0


class Robot(object):
    """真值机器人：x=横向(右正 mm)，y=前向(mm)，heading=航向(右正 度)。"""

    def __init__(self, gait_bias=0.15, turn_noise=0.3):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.gait_bias = gait_bias   # 每块直行的系统性偏转（度）
        self.turn_noise = turn_noise

    def turn(self, deg):
        self.heading += deg

    def lateral(self, mm):
        # 六足横移近似纯平移（航向耦合略）
        self.x += mm * math.cos(self.heading * _DEG)
        self.y -= mm * math.sin(self.heading * _DEG)

    def forward(self, ds):
        # 每走一块，heading 先系统性偏转 gait_bias（左右腿不对称），再前进
        self.heading += self.gait_bias
        self.x += ds * math.sin(self.heading * _DEG)
        self.y += ds * math.cos(self.heading * _DEG)


def observe(robot, block_bx, block_by):
    """相机测方块方位（含传感器噪声可另加），返回 (cx_px, range_mm)。"""
    dx = block_bx - robot.x
    dy = block_by - robot.y
    R = math.hypot(dx, dy)
    beta_world = math.atan2(dx, dy)          # 世界方位
    beta_cam = beta_world - robot.heading * _DEG   # 机器人系方位（相机看到的）
    cx = CX0 + F_PX * math.tan(beta_cam)
    return cx, R


def run(fusion, robot, block_bx, block_by, chunks, ds=100.0, cx0_hack=False):
    ctrl = LaneController(head_gain=0.9, cross_gain=0.35)
    es, crosses, rxs = [], [], []
    for i in range(chunks):
        cx, R = observe(robot, block_bx, block_by)
        if cx0_hack and fusion.updates == 0:
            fusion.cx0 = cx                      # LaneFusion 旧 hack：冻初始方位
        fusion.update_bearing(cx, R)

        turn, lateral = ctrl.decide(fusion.heading_error, fusion.cross_error)
        if turn:
            robot.turn(turn)
        elif lateral:
            robot.lateral(lateral)

        # IMU 实测转角 = 控制器转的 turn + 步态系统偏转（模拟每块走歪一点）
        d_theta_imu = turn + robot.gait_bias
        fusion.predict(d_theta_imu, ds_mm=ds, lateral_mm=lateral)

        robot.forward(ds)
        es.append(fusion.heading_error)
        crosses.append(fusion.cross_error)
        rxs.append(robot.x)
    return es, crosses, rxs


def main():
    block_bx, block_by = 200.0, 3000.0     # 方块在路线右侧 200mm、前方 3m
    chunks = 15                             # 1500mm 直行
    gait_bias = 0.15                        # 每块右偏 0.15°

    print('=' * 72)
    print('方块在 (%.0f, %.0f)（路线右外侧），直行 %dmm，每块步态右偏 %.2f°'
          % (block_bx, block_by, chunks * 100, gait_bias))
    print('=' * 72)

    # 1) LaneFusion cx0 hack
    rob = Robot(gait_bias=gait_bias)
    f = LaneFusion(f_px=F_PX, cx0=CX0)
    es1, cs1, rx1 = run(f, rob, block_bx, block_by, chunks, cx0_hack=True)
    print('\n[LaneFusion + cx0 hack]')
    print('  终点横向 rx=%+.1f mm（>0 向右=被方块吸过去）' % rob.x)
    print('  e 末值 %+.2f°，cross 末值 %+.1f mm' % (es1[-1], cs1[-1]))

    # 2) ReferenceFusion
    rob = Robot(gait_bias=gait_bias)
    f = ReferenceFusion(f_px=F_PX, cx0=CX0)
    es2, cs2, rx2 = run(f, rob, block_bx, block_by, chunks)
    rx_ref = rob.x
    print('\n[ReferenceFusion]')
    print('  终点横向 rx=%+.1f mm' % rx_ref)
    print('  e 末值 %+.2f°，cross 末值 %+.1f mm' % (es2[-1], cs2[-1]))
    print('  e 序列: %s' % ' '.join('%+.1f' % e for e in es2[::2]))
    print('  cross 序列: %s' % ' '.join('%+.0f' % c for c in cs2[::2]))

    # 3) 无纠偏基准：机器人自己会漂多少
    rob = Robot(gait_bias=gait_bias)
    for _ in range(chunks):
        rob.forward(100.0)
    rx_bare = rob.x
    print('\n[无纠偏基准] 终点横向 rx=%+.1f mm' % rx_bare)

    # 判定
    print('\n' + '-' * 72)
    max_e = max(abs(e) for e in es2)
    if max_e > 30.0:
        print('❌ ReferenceFusion 发散（max|e|=%.1f° > 30°）' % max_e)
    else:
        print('✅ ReferenceFusion 不发散（max|e|=%.1f°）' % max_e)
    if abs(rx_ref) < abs(rx_bare):
        print('✅ ReferenceFusion 终点偏差 %.0fmm < 无纠偏 %.0fmm（贴路线）'
              % (abs(rx_ref), abs(rx_bare)))
    else:
        print('⚠️  ReferenceFusion 没比无纠偏更贴路线，调增益或重查方程')
    if abs(rx_ref) <= abs(rx1[-1]):
        print('✅ ReferenceFusion %.0fmm <= cx0 hack %.0fmm'
              % (abs(rx_ref), abs(rx1[-1])))
    else:
        print('⚠️  ReferenceFusion 比 cx0 hack 更偏')

    # ---- 反向漂移场景：方块在右、漂移向左（最容易被「吸向方块」骗到的方向）----
    print('\n' + '=' * 72)
    print('反向：方块在右 (%.0f, %.0f)，每块步态**左**偏 %.2f°'
          % (block_bx, block_by, gait_bias))
    print('=' * 72)
    rob = Robot(gait_bias=-gait_bias)
    f = ReferenceFusion(f_px=F_PX, cx0=CX0)
    es3, cs3, rx3 = run(f, rob, block_bx, block_by, chunks)
    print('[ReferenceFusion] 终点横向 rx=%+.1f mm（>0 向右=被方块吸）' % rob.x)
    rob = Robot(gait_bias=-gait_bias)
    for _ in range(chunks):
        rob.forward(100.0)
    print('[无纠偏基准]   终点横向 rx=%+.1f mm' % rob.x)
    print('  max|e|=%.1f°' % max(abs(e) for e in es3))


if __name__ == '__main__':
    main()
