#!/usr/bin/python3
# coding=utf8
"""验证 ReferenceFusion：方块在路线外侧 + 机器人有航向偏置，融合能否正确估出漂移。

对比 LaneFusion 的老毛病：LaneFusion 会「朝方块走」（保持方块方位不变），方块在
路线外侧时把机器人带偏。ReferenceFusion 应该只估出「相对理想路线的漂移」，而不被
方块位置带偏。
"""
import math
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def _load_fusion():
    """heading_fusion 是纯计算模块，不碰硬件；按文件路径加载，仿真在哪都能跑。"""
    import importlib.util
    path = os.path.join(_PKG_ROOT, 'agcs_lib', 'heading_fusion.py')
    spec = importlib.util.spec_from_file_location('heading_fusion', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_fusion = _load_fusion()
ReferenceFusion = _fusion.ReferenceFusion
LaneFusion = _fusion.LaneFusion
LaneController = _fusion.LaneController

_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi

f_px = 419.0
cx0 = 160.0
f_px_full = 838.0


def run_reference(block_wx, block_wy, heading_bias, distance_mm=1000.0, step=100.0):
    """模拟机器人从原点沿 +y 走直线（含航向偏置），用 ReferenceFusion 估漂移。"""
    fusion = ReferenceFusion(f_px=f_px, cx0=cx0, f_px_full=f_px_full, marker_size_mm=0.0)

    rx = ry = 0.0
    n = int(round(distance_mm / step))
    es, crosses, actual_rx = [], [], []

    def observe():
        """相机测方块：方位 = 世界方位 - 机器人当前航向（含偏置）。"""
        dx = block_wx - rx
        dy = block_wy - ry
        R = math.hypot(dx, dy)
        beta_cam = math.atan2(dx, dy) - heading_bias * _DEG  # 相机相对机器人航向
        cx = cx0 + f_px * math.tan(beta_cam)
        return cx, R

    # 初始：在原点先观测一次并 init（对应真机直线段开头第一次检测）
    cx, R = observe()
    fusion.update(cx, R)

    for i in range(n):
        rx += step * math.sin(heading_bias * _DEG)
        ry += step * math.cos(heading_bias * _DEG)
        cx, R = observe()
        fusion.predict(0.0, ds_mm=step, lateral_mm=0.0)
        e, cross = fusion.update(cx, R)

        es.append(e)
        crosses.append(cross)
        actual_rx.append(rx)

    return es, crosses, actual_rx


def run_lane(block_wx, block_wy, heading_bias, distance_mm=1000.0, step=100.0):
    """对照组：LaneFusion（旧）在同样场景下的表现。"""
    fusion = LaneFusion(f_px=f_px, cx0=cx0)
    ctrl = LaneController(head_gain=0.9, cross_gain=0.35)

    rx = ry = 0.0
    n = int(round(distance_mm / step))
    actual_rx = []

    for i in range(n):
        rx += step * math.sin(heading_bias * _DEG)
        ry += step * math.cos(heading_bias * _DEG)

        dx = block_wx - rx
        dy = block_wy - ry
        R = math.hypot(dx, dy)
        beta = math.atan2(dx, dy)
        cx = cx0 + f_px * math.tan(beta)

        # LaneFusion：predict + update（老逻辑）
        # 这里简化：不模拟控制器闭环，只看 cross 估计
        fusion.predict(0.0, ds_mm=step, lateral_mm=0.0)
        fusion.update_bearing(cx, R)

        actual_rx.append(rx)

    return fusion, actual_rx


def main():
    block_wx, block_wy = 200.0, 3000.0   # 方块放远处，机器人 1000mm 内不会越过它
    heading_bias = 3.0

    print('=' * 70)
    print('场景：方块在 (%.0f, %.0f)（路线外侧），机器人航向偏置 %+.1f°，前进 1000mm'
          % (block_wx, block_wy, heading_bias))
    print('=' * 70)

    es, crosses, actual_rx = run_reference(block_wx, block_wy, heading_bias)
    print('\n[ReferenceFusion]')
    for i in range(0, len(es), 2):
        print('  第 %2d 步: 实际横向 %+6.1f mm | 估 cross %+6.1f mm | 估 e %+5.2f°'
              % (i, actual_rx[i], crosses[i], es[i]))

    err_cross = abs(crosses[-1] - actual_rx[-1])
    err_e = abs(es[-1] - heading_bias)
    print('\n  终点: 实际横向 %+.1f mm, 估 cross %+.1f mm (误差 %.1f mm)'
          % (actual_rx[-1], crosses[-1], err_cross))
    print('  终点: 实际航向 %+.1f°, 估 e %+.2f° (误差 %.2f°)'
          % (heading_bias, es[-1], err_e))

    if err_cross < 10 and err_e < 1.0:
        print('  ✅ ReferenceFusion 能正确估出漂移，不被方块位置带偏')
    else:
        print('  ❌ 估计偏差过大，检查方程')


if __name__ == '__main__':
    main()
