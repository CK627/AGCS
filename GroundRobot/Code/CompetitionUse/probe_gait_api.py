#!/usr/bin/python3
# coding=utf8
"""在真机上探测 kinematics 的连续步态接口（只探测，不改任何业务流程）。

必须先在机器人上跑一遍，否则 gait_hold.GaitHold 的 rotation /
movement_direction 全是猜的，接进主流程一定会翻车。

跑法（在机器人上，不是在你 Mac 上）：

    cd ~/AGCS/GroundRobot/Code
    python3 CompetitionUse/probe_gait_api.py                 # 只打印 API 表面（安全）
    python3 CompetitionUse/probe_gait_api.py --spin          # 标定 rotation（会原地转）
    python3 CompetitionUse/probe_gait_api.py --dir           # 标定 movement_direction（会走）

--spin / --dir 会动，确认周围空旷、机器人垫高或放在地上有人看着再跑。
"""

import argparse
import inspect
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

# setStepMode 真实签名（inspect.signature 实测 12 参数，docstring 只写了 9 个）：
#   pos, mode, step_velocity, step_amplitude, step_height,
#   movement_direction, rotation_angle, rotation, p, o, speed, times
#   （rotation_angle=绝对转角 / p=方向角 / o=旋转角，走直线/纯自转时都传 0）
DEF = dict(mode=2, step_velocity=40.0, amplitude=30.0, height=25.0,
           direction=0.0, rotation=0.0, servo_speed=60, times=1)


def _sig(fn):
    try:
        return str(inspect.signature(fn))
    except (TypeError, ValueError):
        return '(?)'


def _safe(fn, *a, **kw):
    """调用持续步态接口；签名不匹配就把异常摊开给人看，不静默吞掉。"""
    return fn(*a, **kw)


def probe_api(ik):
    print('=' * 70)
    print('1) kinematics.IK 可见方法')
    print('=' * 70)
    for name in sorted(dir(ik)):
        if name.startswith('_'):
            continue
        obj = getattr(ik, name, None)
        if callable(obj):
            print('  def %-30s %s' % (name, _sig(obj)))
    print()
    print('=' * 70)
    print('2) 关键步态方法 / 姿态方法的 docstring')
    print('=' * 70)
    for n in ('setStepMode', 'setStepMode_whitout_delay', 'go_forward',
              'turn_left', 'moveBody', 'stopMove', 'stop_move', 'stand'):
        fn = getattr(ik, n, None)
        if fn is None:
            print('  x  没有 %s' % n)
            continue
        print('  --- %s%s ---' % (n, _sig(fn)))
        doc = inspect.getdoc(fn) or ''
        if doc:
            for line in doc.splitlines():
                if line.strip():
                    print('      ' + line.strip())
        else:
            print('      (无 docstring)')
    print()
    print('=' * 70)
    print('3) 姿态 / 校准相关属性')
    print('=' * 70)
    for n in ('initial_pos', 'initial_pos_high', 'initial_pos_quadruped',
              'current_pos', 'last_pos', 'CALIBRATION_DEVIATION',
              'DEFLECTION_ANGLE', 'LEG_MOVEMENT_INDEX', 'LINK1', 'LINK2',
              'LINK3', 'roll', 'pitch', 'yaw'):
        if hasattr(ik, n):
            print('  %-24s = %s' % (n, str(getattr(ik, n))[:150].replace('\n', ' ')))
        else:
            print('  %-24s (不存在)' % n)


def _drive(ik, pos, seconds, **kw):
    """用 move 持续走 seconds 秒（move 阻塞，一次 times=1 约 0.6s）。

    原实现用 setStepMode_whitout_delay 25Hz 下发，实测「转两下就停」不稳定，
    已改用 move（与 probe_spin 一致）。move 签名：(pos, mode, amplitude,
    movement_direction, rotation, speed, times)。
    """
    p = dict(DEF)
    p.update(kw)
    fn = getattr(ik, 'move', None)
    if fn is None:
        raise SystemExit('没有 move，本探测无法进行')
    t_end = time.time() + seconds
    while time.time() < t_end:
        _safe(fn, pos, p['mode'], p['amplitude'], p['direction'],
              p['rotation'], p['servo_speed'], 1)


def probe_spin(ik, board, seconds=3.0, rots=(0.25, 0.5, 1.0)):
    """标定 rotation=-1~1 与偏航角速度的对应关系。"""
    from agcs_lib import ImuTracker

    state = {'yaw': 0.0}
    tr = ImuTracker(board, state)
    tr.start()
    time.sleep(0.5)
    print('IMU 线程已启动（采样数会持续增长）')

    pos = ik.initial_pos
    results = []
    try:
        tr.calibrate(0.6, settle=0.4)
        for r in rots:
            tr.reset()
            time.sleep(0.2)
            # 用 move 标定（setStepMode 实测「转两下就停」，不稳定，弃用）
            t0 = time.time()
            ik.move(pos, 2, 30.0, 0.0, r, 60, 3)
            dt = time.time() - t0
            time.sleep(0.5)
            dyaw = state['yaw']
            w = dyaw / dt if dt > 0 else 0.0
            results.append((r, dyaw, w))
            print('rotation=%+.2f  %5.2fs 内偏航 %+7.2f°  ->  %+7.2f °/s'
                  % (r, dt, dyaw, w), flush=True)
            ik.stand(ik.initial_pos, t=400)
            time.sleep(0.3)
    finally:
        tr.stop()
        _stop(ik, pos)
        ik.stand(ik.initial_pos, t=500)

    print()
    if results:
        r1 = [x for x in results if abs(x[0] - 1.0) < 1e-6]
        if r1:
            print('>> rot_max_dps = %.1f  （rotation=1.0 时的偏航角速度）' % abs(r1[0][2]))
            print('   把这个值传给 GaitHold(rot_max_dps=...) 或 --hold-rot-max-dps')
        print('>> 线性度检查（应近似成正比）：')
        for r, dyaw, w in results:
            print('     rotation=%.2f -> %+7.2f °/s   单位 rotation: %+7.2f °/s'
                  % (r, w, w / r if r else float('nan')))


def probe_dir(ik, seconds=3.0, dirs=(0.0, 15.0, 30.0, 90.0)):
    """演示 movement_direction：每个方向走 seconds 秒，用尺子量实际位移方向。"""
    pos = ik.initial_pos
    print('每个方向走 %.1fs，请用量角器/尺子量机身相对初始朝向的位移方向角。' % seconds)
    print('（正前方 = 0°，顺时针为正 or 逆时针为正 —— 这个符号必须量出来）')
    try:
        for d in dirs:
            input('  >> 方向 %.0f：回车开始（先确认周围空旷）' % d)
            _drive(ik, pos, seconds, direction=d)
            _stop(ik, pos)
            time.sleep(0.6)
            ik.stand(ik.initial_pos, t=400)
            print('     方向 %.0f 走完，请量位移方向并记下' % d)
    finally:
        _stop(ik, pos)
        ik.stand(ik.initial_pos, t=500)


def _stop(ik, pos):
    fn = getattr(ik, 'stopMove', None) or getattr(ik, 'stop_move', None)
    if fn is not None:
        try:
            fn()
        except Exception as e:
            print('  stopMove 失败：%r' % (e,))


def main():
    ap = argparse.ArgumentParser(description='探测 SpiderPi Pro 连续步态接口')
    ap.add_argument('--spin', action='store_true', help='标定 rotation（会原地转）')
    ap.add_argument('--dir', action='store_true', help='标定 movement_direction（会走）')
    ap.add_argument('--seconds', type=float, default=3.0)
    args = ap.parse_args()

    from agcs_lib import make_board, make_ik
    board = make_board()
    ik = make_ik(board)
    ik.stand(ik.initial_pos, t=500)

    probe_api(ik)

    if args.spin:
        print()
        print('=' * 70)
        print('4) rotation 标定（机器人会原地转，确认周围空旷！）')
        print('=' * 70)
        probe_spin(ik, board, seconds=args.seconds)
    if args.dir:
        print()
        print('=' * 70)
        print('5) movement_direction 标定（机器人会走）')
        print('=' * 70)
        probe_dir(ik, seconds=args.seconds)

    if not (args.spin or args.dir):
        print()
        print('只跑了 API 探测。要标定 rotation 请加 --spin，要标定方向请加 --dir。')


if __name__ == '__main__':
    main()
