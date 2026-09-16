#!/usr/bin/python3
# coding=utf8
"""接线自检：用假硬件把 Auto-capture.py 里的融合函数真跑一遍。

`sim_heading.py` 验证的是**算法**，这个脚本验证的是**接线** —— 也就是
`move_straight_fusion` / `feed_turn_to_fusion` 里最容易悄悄搞错的东西：

- 左右符号约定（imu_state['yaw'] 是左正，本模块用右正）
- turn_left/turn_right 与 ±angle 的对应关系
- IMU 增量有没有被重复计入（转弯尾巴 vs 直线段第一次取值）
- 连续转弯后 e 会不会被撑爆

它用 stub 顶掉 cv2 / agcs_lib / board / ik，纯本地跑：

    python3 CompetitionUse/sim_fusion_wiring.py
"""

import importlib.util
import math
import os
import random
import sys
import types

_DEG = math.pi / 180.0
_RAD = 180.0 / math.pi
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------- 假硬件 ----------------
class FakeIK(object):
    """真值机器人：th 右正（度），x 右正（mm）。"""

    def __init__(self, rng, turn_gain=1.0, turn_noise=0.5,
                 gait_bias=0.12, gait_noise=0.5):
        self.initial_pos = None
        self.rng = rng
        self.turn_gain = turn_gain
        self.turn_noise = turn_noise
        self.gait_bias = gait_bias
        self.gait_noise = gait_noise
        self.x = 0.0
        self.th = 0.0
        self.n_turn = 0
        self.n_lat = 0

    def _rotate(self, deg):
        self.th += deg

    def turn_left(self, pos, t, deg, speed, n):
        self.th -= deg * self.turn_gain + self.rng.gauss(0, self.turn_noise)
        self.n_turn += 1

    def turn_right(self, pos, t, deg, speed, n):
        self.th += deg * self.turn_gain + self.rng.gauss(0, self.turn_noise)
        self.n_turn += 1

    def left_move(self, pos, t, mm, speed, n):
        self.x -= mm * math.cos(self.th * _DEG)
        self.n_lat += 1

    def right_move(self, pos, t, mm, speed, n):
        self.x += mm * math.cos(self.th * _DEG)
        self.n_lat += 1

    def go_forward(self, pos, t, mm, speed, n):
        self.th += self.rng.gauss(self.gait_bias, self.gait_noise)
        self.x += mm * math.sin(self.th * _DEG)

    def back(self, pos, t, mm, speed, n):
        self.th += self.rng.gauss(self.gait_bias, self.gait_noise)
        self.x -= mm * math.sin(self.th * _DEG)


class FakeTracker(object):
    """只实现 since_last：返回「自上次调用以来」的平均角速度与时长（左正）。"""

    def __init__(self, imu_state):
        self.state = imu_state
        self.mark = imu_state['yaw']
        self.t = 0.0

    def since_last(self):
        dyaw = self.state['yaw'] - self.mark
        self.mark = self.state['yaw']
        self.t += 1.0
        return dyaw, 1.0


# ---------------- 装载被测模块 ----------------
def load_autocapture():
    sys.modules.setdefault('cv2', types.ModuleType('cv2'))

    fake = types.ModuleType('agcs_lib')
    for name in ('make_board', 'make_ik', 'ImuTracker', 'load_params',
                 'load_lab_data', 'load_undistort_maps', 'detect_color',
                 'correct_camera', 'open_camera', 'capture'):
        setattr(fake, name, lambda *a, **k: None)
    sys.modules['agcs_lib'] = fake

    spec = importlib.util.spec_from_file_location(
        'agcs_lib.heading_fusion',
        os.path.join(_ROOT, 'agcs_lib', 'heading_fusion.py'))
    hf = importlib.util.module_from_spec(spec)
    sys.modules['agcs_lib.heading_fusion'] = hf
    spec.loader.exec_module(hf)

    spec = importlib.util.spec_from_file_location(
        'autocapture_under_test',
        os.path.join(_ROOT, 'CompetitionUse', 'Auto-capture.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, hf


def main():
    ac, hf = load_autocapture()
    print('已装载 Auto-capture.py 与 heading_fusion.py（假硬件）')

    f_px, cx0, f_full = 277.0, 160.0, 554.0
    marker_size = 35.0        # 色块真实半径 mm，用于按视半径估距离
    p_drop, px_noise = 0.15, 1.5

    results = []
    for trial in range(30):
        rng = random.Random(500 + trial)
        ik = FakeIK(rng)
        # imu_state['yaw'] 是左正 = -th（右正）
        imu_state = {'yaw': 0.0}
        tracker = FakeTracker(imu_state)
        fusion = hf.LaneFusion(f_px=f_px, cx0=cx0)
        ctrl = hf.LaneController()

        r_true = 1500.0
        marker_range = 1000.0

        def detector():
            nonlocal r_true
            if rng.random() < p_drop:
                return None
            beta = math.atan2(-ik.x, r_true) * _RAD - ik.th
            cx = cx0 + f_px * math.tan(beta * _DEG) + rng.gauss(0, px_noise)
            cx = max(0.0, min(319.0, cx))
            # radius 是原始分辨率下的视半径（320 坐标系的 2 倍）
            radius_px = f_full * marker_size / max(80.0, r_true)
            return {'bbox_center_x': cx, 'center': (cx * 2, 120.0), 'radius': radius_px}

        def hook(step_mm):
            """每前进 100mm 同步一次真值与 IMU。"""
            r = r_true - step_mm
            if r < 250.0:
                r = 1500.0
            return r

        # 用一个包装把「真值推进」塞进 go_forward
        orig_fwd = ik.go_forward
        orig_back = ik.back

        def go_forward(pos, t, mm, speed, n, _o=orig_fwd):
            th0 = ik.th
            _o(pos, t, mm, speed, n)
            imu_state['yaw'] += -(ik.th - th0)     # 左正
        ik.go_forward = go_forward
        ik.back = orig_back

        for seg in range(6):
            if seg > 0:
                # 模拟一次 turn_left 15°：命令 -15（右正），实际带残留
                yaw_before = imu_state['yaw']
                cmd_right = -15.0
                th0 = ik.th
                ik.turn_left(None, 2, 15, 30, 1)
                ik.th += 1.5                        # imu_turn 只修 1° 的残留
                imu_state['yaw'] += -(ik.th - th0)
                ac.feed_turn_to_fusion(imu_state, tracker, fusion,
                                       yaw_before, cmd_right)
                tracker.since_last()

            marker_range = ac.move_straight_fusion(
                ik, None, detector, imu_state, tracker, 1000,
                fusion, ctrl, marker_range, marker_size)
            r_true = hook(1000.0)

        results.append((abs(ik.x), abs(fusion.heading_error)))

    n = float(len(results))
    end_x = sum(r[0] for r in results) / n
    end_e = sum(r[1] for r in results) / n
    worst = max(r[0] for r in results)
    print()
    print('30 次试验 · 6 段 × 1000mm · 每段间接一次左转 15°（残留 1.5°）')
    print('  终点横向偏差  平均 %6.1fmm   最差 %6.1fmm' % (end_x, worst))
    print('  终点航向残差  平均 %6.2f°' % end_e)
    print()
    # 判据 20mm 是拿「反向对照」卡出来的：把转弯命令符号取反后均值会涨到 ~27mm，
    # 符号正确时 ~10mm。所以这个脚本同时也是转弯符号的回归守卫。
    if end_x < 20.0 and end_e < 3.0:
        print('✅ 接线自检通过：符号约定正确，连续转弯未撑爆估计，横向偏差收敛')
        return 0
    print('❌ 接线自检未通过（横向偏差 %.1fmm / 航向 %.2f°）。'
          '若偏差在 20~40mm 之间，优先怀疑 feed_turn_to_fusion 的 cmd_right_deg 符号：'
          'JSON 的 turn_left 应传 -angle、turn_right 应传 +angle。'
          % (end_x, end_e))
    return 1


if __name__ == '__main__':
    sys.exit(main())
