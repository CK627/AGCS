#!/usr/bin/python3
# coding=utf8
"""2.py —— 单独测 YOLO 检测 + 夹取（不走路线的两步状态机）。

跑法（在机器人上）：
    sudo systemctl stop spiderpi
    cd ~/spiderpi/CompetitionUse
    python3 2.py                 # 检测 + 夹取
    python3 2.py --conf 0.4      # 检测不到先降置信度门槛试

两步状态机（靠 `.2_grabbed` 标志文件记住「上次有没有夹到」）：

    第 1 次运行（还没夹到）：
        不恢复原位 → 直接 YOLO 检测（检测不到就重试 --tries 次）
        → 检测到就夹取（和 1.py 的夹取动作一样：闭合夹爪 25 → 拔起 22）
        → 写「已夹到」标志 → 退出（机械臂停在夹取后的姿态，方便现场看夹没夹到）

    第 2 次运行（上次夹到了）：
        恢复初始位置（21-24 回 OFFICIAL_ARM、夹爪张开）→ 清标志 → 退出（不动）

夹取刻意不恢复原位：恢复动作放到下一次运行，这样这次跑完能看清夹到没有。
想强制重新走「检测 + 夹取」，删掉标志文件即可：
    rm ~/spiderpi/CompetitionUse/.2_grabbed
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

from agcs_lib import make_board

# 标志文件：存在 = 上次已经夹到了
STATE_FILE = os.path.join(_HERE, '.2_grabbed')


def _load_no7():
    """把 Auto-capture-1.py 当模块加载，复用 open_vision / ModelDetector / restore_travel / 常量。"""
    path = os.path.join(_HERE, 'Auto-capture-1.py')
    if not os.path.exists(path):
        raise SystemExit('找不到 %s —— 2.py 必须和 Auto-capture-1.py 放同一目录' % path)
    spec = importlib.util.spec_from_file_location('no7_for_2py', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['no7_for_2py'] = mod
    spec.loader.exec_module(mod)
    return mod


no7 = _load_no7()


def grab(board, pull_up_pulse):
    """夹取（和 1.py 的夹取动作一样）：闭合夹爪 25 → 拔起 22。不恢复原位（恢复放到下次运行）。"""
    board.bus_servo_set_position(2.0, [[25, no7.GRIPPER_CLOSE]])
    time.sleep(2.0)
    time.sleep(0.5)
    pulse = max(0, min(1000, int(pull_up_pulse)))
    board.bus_servo_set_position(1.0, [[22, pulse]])
    time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=no7.DEFAULT_MODEL, help='YOLO 模型路径')
    ap.add_argument('--conf', type=float, default=no7.MODEL_CONF,
                    help='模型置信度阈值（检测不到先降到 0.4 试）')
    ap.add_argument('--classes', default='', help='目标类别，逗号分隔；留空=接受所有类别')
    ap.add_argument('--color', default='red', help='推流 LAB 显示用色（只影响显示）')
    ap.add_argument('--pull-up', type=int, default=no7.PULL_UP_22,
                    help='夹取后 22 号肩拔起脉宽（默认 %(default)d）')
    ap.add_argument('--tries', type=int, default=20,
                    help='检测不到时重试次数（默认 %(default)d）')
    ap.add_argument('--interval', type=float, default=0.5,
                    help='每次重试间隔秒（默认 %(default)s）')
    args = ap.parse_args()

    board = make_board()

    # 上次夹到了 → 恢复初始位置，然后不动
    if os.path.exists(STATE_FILE):
        no7.restore_travel(board, no7.GRIPPER_OPEN)
        os.remove(STATE_FILE)
        print('上次已夹到：本次恢复初始位置（夹爪张开），清标志后退出', flush=True)
        return

    # 不恢复原位，直接检测
    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    cam, read_frame, _color_detector, publish = no7.open_vision(args.color, 1)
    model_det = no7.ModelDetector(model_path, args.conf, classes, read_frame, publish)

    if no7.task_server is not None:
        no7.task_server.start_server()

    print('=' * 64, flush=True)
    print('2.py：不恢复原位，直接 YOLO 检测，检测到就夹取', flush=True)
    print('模型=%s conf=%.2f 重试 %d 次 / %.1fs'
          % (args.model, args.conf, args.tries, args.interval), flush=True)
    print('=' * 64, flush=True)

    for i in range(1, args.tries + 1):
        det = model_det.detect()
        if det is not None:
            print('✅ 检测到虫子 conf=%.2f bbox=%dx%d@(%d,%d)，执行夹取'
                  % (det['conf'], det['w'], det['h'], det['x'], det['y']), flush=True)
            grab(board, args.pull_up)
            with open(STATE_FILE, 'w') as f:
                f.write('grabbed\n')
            print('夹取完成，退出（下次运行会恢复初始位置）', flush=True)
            cam.camera_close()
            return
        print('  未检测到（%d/%d），重试…' % (i, args.tries), flush=True)
        time.sleep(args.interval)

    print('❌ 检测不到虫子，退出（未写标志，下次运行仍走检测）', flush=True)
    cam.camera_close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
