#!/usr/bin/python3
# coding=utf8
"""1.py —— NO7 + 融合导航（夹取/放下/路线仍读 JSON）。

与 NO6/NO7 的差别只在一处：直线段的「IMU 航向 + 颜色左右微调」双回路
换成「融合导航」（LaneFusion + LaneController：相机写状态、IMU 管执行、
先航向后横向）。夹取/放下/路线（fixed_route.json 的 pick/place 脉宽）都
原样走 NO7，不做 YOLO、不做深度。

YOLO 夹取那套已经挪到 2.py，这里不再碰。

为什么用 importlib 而不是 import：NO7 文件名带连字符（Auto-capture-1.py），
是非法模块名，普通 import 做不到。

跑法：
    cd /home/pi/spiderpi/CompetitionUse
    sudo systemctl stop spiderpi
    python3 1.py --color red            # 其余参数原样透传给 NO7
    python3 1.py --color red --pull-up 450
    python3 1.py --marker-range 800     # 融合导航调参
"""

import argparse
import importlib.util
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def _load_no7():
    """把 Auto-capture-1.py 当模块加载（文件名带 '-'，普通 import 做不到）。"""
    path = os.path.join(_HERE, 'Auto-capture-1.py')
    if not os.path.exists(path):
        raise SystemExit('找不到 %s —— 1.py 必须和 Auto-capture-1.py 放在同一目录' % path)
    spec = importlib.util.spec_from_file_location('no7', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['no7'] = mod
    spec.loader.exec_module(mod)
    return mod


no7 = _load_no7()


# ---------- 融合导航（替换 NO7 直线段的 IMU+颜色 双回路） ----------

from agcs_lib.heading_fusion import LaneFusion, LaneController, range_from_radius_px

# 融合导航参数。下面只是「能跑」的占位默认，真值用命令行调（见 main() 的 --f-px /
# --marker-range / --fusion-* 等）。与 NO6 `Auto-capture.py` 的 --fusion 一套对齐。
FUSION_CFG = {
    'f_px': 277.0,          # 320 坐标系焦距（detect_color 内部 resize 到 320×240）
    'f_px_full': 554.0,     # 原始 640 分辨率焦距，用于色块视半径反推距离
    'cx0': 160.0,           # 320 坐标系光心横坐标
    'head_gain': 0.9,       # 航向 P 增益（转「误差 × 该比例」度）
    'head_max': 6.0,        # 单次转向上限（度）
    'cross_gain': 0.35,     # 横向 P 增益（横移「偏差 × 该比例」mm）
    'cross_deadzone': 8.0,  # 横向死区（mm），偏差小于它就不横移（少发横移=少注噪声）
    'cross_max': 25.0,      # 单次横移上限（mm）
    'marker_range': 1000.0, # 到色块的前向距离（mm），--marker-size-mm=0 时用定值
    'marker_size': 0.0,     # 色块真实半径（mm），>0 时用视半径反推距离（需现场量）
}

_FUSION = {'fusion': None, 'ctrl': None}


def _ensure_fusion():
    if _FUSION['fusion'] is None:
        _FUSION['fusion'] = LaneFusion(f_px=FUSION_CFG['f_px'], cx0=FUSION_CFG['cx0'])
        _FUSION['ctrl'] = LaneController(head_gain=FUSION_CFG['head_gain'],
                                         head_max=FUSION_CFG['head_max'],
                                         cross_gain=FUSION_CFG['cross_gain'],
                                         cross_deadzone=FUSION_CFG['cross_deadzone'],
                                         cross_max=FUSION_CFG['cross_max'])


def fusion_move_straight(ik, board, detector, imu_state, target_yaw,
                         distance_mm, tilt, color_enabled, color_state):
    """融合版直线段（替换 NO7 的 move_straight_imu_color）。

    - 相机只「写状态」（update_bearing 更新 e/cross），不再平移去消像素偏移；
    - 单一控制器「先航向后横向」，同一小块只发一种指令；
    - fusion 状态全程不归零。
    """
    _ensure_fusion()
    fusion = _FUSION['fusion']
    ctrl = _FUSION['ctrl']
    tracker = imu_state.get('tracker')
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    last_ds = 0.0
    while remaining > 0:
        det = detector()
        cx = None
        rng = FUSION_CFG['marker_range']
        if det is not None:
            cx = det.get('bbox_center_x', det['center'][0])
            # radius 是映射回原始 640 分辨率的像素，所以用 f_px_full 反推距离
            if FUSION_CFG['marker_size'] > 0 and det.get('radius'):
                r = range_from_radius_px(det['radius'], FUSION_CFG['marker_size'],
                                         FUSION_CFG['f_px_full'])
                if r is not None:
                    rng = FUSION_CFG['marker_range'] = max(80.0, r)
        if cx is not None:
            fusion.update_bearing(cx, rng)

        turn, lateral = ctrl.decide(fusion.heading_error, fusion.cross_error)
        if turn != 0.0:
            if turn < 0:
                ik.turn_left(ik.initial_pos, 2, abs(turn), no7.TURN_SPEED, 1)
            else:
                ik.turn_right(ik.initial_pos, 2, abs(turn), no7.TURN_SPEED, 1)
            print('融合 e=%+.2f° cross=%+.0fmm -> 转%+.1f°'
                  % (fusion.heading_error, fusion.cross_error, turn), flush=True)
        elif lateral != 0.0:
            if lateral < 0:
                ik.left_move(ik.initial_pos, 2, abs(lateral), no7.MOVE_SPEED, 1)
            else:
                ik.right_move(ik.initial_pos, 2, abs(lateral), no7.MOVE_SPEED, 1)
            print('融合 e=%+.2f° cross=%+.0fmm -> 横移%+.0fmm'
                  % (fusion.heading_error, fusion.cross_error, lateral), flush=True)

        if tracker is not None:
            rate, dt = tracker.since_last()
            d_theta_right = -rate * dt
        else:
            d_theta_right = 0.0
        fusion.predict(d_theta_right, ds_mm=last_ds, lateral_mm=lateral)

        move = min(100, remaining)
        no7.move_one_chunk(ik, move, forward)
        last_ds = move if forward else -move
        remaining -= move
        time.sleep(0.05)


def fusion_reset_imu(board, imu_state):
    """融合模式：只重标零漂，**不清零航向**（融合状态不归零，reset 是把误差搬到机器人身上）。"""
    tracker = imu_state.get('tracker')
    if tracker is None:
        return
    tracker.calibrate(0.5, settle=no7.IMU_SETTLE_S)


# ---------- 入口 ----------

def main():
    ap = argparse.ArgumentParser(add_help=False)
    # 融合导航调参（默认值只是占位，真值在机器人上试出来直接写回 FUSION_CFG 或用命令行）
    ap.add_argument('--f-px', type=float, default=FUSION_CFG['f_px'],
                    help='融合 320 坐标系焦距（默认 %.0f）' % FUSION_CFG['f_px'])
    ap.add_argument('--cx0', type=float, default=FUSION_CFG['cx0'],
                    help='融合 320 坐标系光心横坐标（默认 %.0f）' % FUSION_CFG['cx0'])
    ap.add_argument('--marker-range', type=float, default=FUSION_CFG['marker_range'],
                    help='到色块的前向距离 mm（--marker-size-mm=0 时用定值，默认 %.0f）'
                         % FUSION_CFG['marker_range'])
    ap.add_argument('--marker-size-mm', type=float, default=FUSION_CFG['marker_size'],
                    help='色块真实半径 mm，>0 用视半径反推距离（默认 %.0f）'
                         % FUSION_CFG['marker_size'])
    ap.add_argument('--fusion-head-gain', type=float, default=FUSION_CFG['head_gain'])
    ap.add_argument('--fusion-head-max', type=float, default=FUSION_CFG['head_max'])
    ap.add_argument('--fusion-cross-gain', type=float, default=FUSION_CFG['cross_gain'])
    ap.add_argument('--fusion-cross-deadzone', type=float, default=FUSION_CFG['cross_deadzone'])
    ap.add_argument('--fusion-cross-max', type=float, default=FUSION_CFG['cross_max'])
    ap.add_argument('-h', '--help', action='store_true', help='显示本文件参数 + NO7 参数')
    args, rest = ap.parse_known_args()

    # 把命令行里的融合参数写回 FUSION_CFG（_ensure_fusion / fusion_move_straight 在运行时读）
    FUSION_CFG['f_px'] = args.f_px
    FUSION_CFG['cx0'] = args.cx0
    FUSION_CFG['marker_range'] = args.marker_range
    FUSION_CFG['marker_size'] = args.marker_size_mm
    FUSION_CFG['head_gain'] = args.fusion_head_gain
    FUSION_CFG['head_max'] = args.fusion_head_max
    FUSION_CFG['cross_gain'] = args.fusion_cross_gain
    FUSION_CFG['cross_deadzone'] = args.fusion_cross_deadzone
    FUSION_CFG['cross_max'] = args.fusion_cross_max

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

    # 融合导航（替换 NO7 直线段的 IMU+颜色 双回路）；夹取/放下/路线仍按 JSON 原样走 NO7
    no7.move_straight_imu_color = fusion_move_straight
    no7.reset_imu = fusion_reset_imu

    print('=' * 64, flush=True)
    print('1.py：融合导航 + JSON 路线（夹取/放下读 fixed_route.json 脉宽，不走 YOLO）', flush=True)
    print('融合导航参数：f_px=%.0f cx0=%.0f range=%.0fmm size=%.0fmm '
          'head_gain=%.2f cross_gain=%.2f' %
          (FUSION_CFG['f_px'], FUSION_CFG['cx0'], FUSION_CFG['marker_range'],
           FUSION_CFG['marker_size'], FUSION_CFG['head_gain'], FUSION_CFG['cross_gain']),
          flush=True)
    print('=' * 64, flush=True)

    no7.main()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
