#!/usr/bin/python3
# coding=utf8
"""CS-1-Restore：随机安全姿态打乱 → 等5s → 恢复初始状态+摄像头启动 → 回车结束。

流程：
    1) 随机打乱一组预校验的安全姿态/动作，逐个"先算后动"执行
       （每个机械臂姿态先用 setPitchRange 校验有解才发指令，无解直接跳过，
         保证不会发出逆运动学无解的动作）；
    2) 等 5 秒（观察随机动作效果）；
    3) 调用 search.py 的 restore_initial_state() + start_camera()；
    4) 等待人工检查，按回车结束（结束时关闭摄像头）。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/Search/CS-1-Restore.py
"""
import os
import random
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import make_board, make_ik, make_arm_ik, load_params, stand
from agcs_lib.logs import setup_logger
from agcs_lib.search import Searcher

# 预校验的安全机械臂姿态（x,y,z cm）
SAFE_ARM_POSES = [
    (0, 15, 5), (12, 24, 5), (0, 10, 15), (5, 18, 8),
    (-8, 20, 6), (0, 15, 12), (8, 14, 4), (-5, 12, 10),
    (0, 18, 8), (10, 20, 10), (-10, 18, 6), (0, 22, 12),
]

# 安全的机身动作（体态升降 dz mm，正=升负=降）
SAFE_BODY_ACTIONS = [10, -5, 15, 0]


def main():
    logger = setup_logger('CS-1-Restore')
    params = load_params()
    arm = params['arm']
    pitch = arm.get('pitch', -90)
    alpha1 = arm.get('alpha1', -90)
    alpha2 = arm.get('alpha2', 100)

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(params)

    stand(ik, t=500)
    time.sleep(0.5)
    logger.info('[0] 立正完成，开始随机姿态打乱')

    # ---------------- 1) 随机打乱安全姿态/动作，逐个"先算后动" ----------------
    actions = [('arm', x, y, z) for x, y, z in SAFE_ARM_POSES] + \
              [('body', dz) for dz in SAFE_BODY_ACTIONS]
    random.shuffle(actions)
    moved = 0
    skipped = 0
    for act in actions:
        if act[0] == 'arm':
            _, x, y, z = act
            # 先校验是否有解（setPitchRange 只计算，不动作）
            if ak.setPitchRange((x, y, z), alpha1, alpha2) is False:
                logger.info('[1] 跳过无解姿态 (%.0f, %.0f, %.0f)', x, y, z)
                skipped += 1
                continue
            ak.setPitchRangeMoving((x, y, z), pitch, alpha1, alpha2, 1)
            logger.info('[1] 动作：机械臂 -> (%.0f, %.0f, %.0f)', x, y, z)
        else:
            dz = act[1]
            ik.moveBody(ik.initial_pos, [0, 0, dz], [0, 0, 0], 300)
            logger.info('[1] 动作：机身升降 %+d mm', dz)
        moved += 1
        time.sleep(1.0)
    logger.info('[1] 随机姿态打乱完成：执行 %d 个，跳过无解 %d 个', moved, skipped)

    # ---------------- 2) 等 5 秒 ----------------
    logger.info('[2] 等待 5 秒…')
    time.sleep(5)

    # ---------------- 3) 调用 search.py 恢复初始状态 + 摄像头启动 ----------------
    searcher = Searcher(board, ik, ak, params, detect=lambda: None)
    try:
        searcher.restore_initial_state()
        logger.info('[3] PASS 恢复初始状态')
    except Exception as e:
        logger.error('[3] FAIL 恢复初始状态: %s', e)
        return 1
    try:
        cam, mapx, mapy, rotate = searcher.start_camera()
        logger.info('[3] PASS 摄像头启动（取帧正常，畸变校正映射已加载）')
    except Exception as e:
        logger.error('[3] FAIL 摄像头启动: %s', e)
        return 1

    # ---------------- 4) 等人工检查，回车结束 ----------------
    try:
        input('检查完成，按回车结束…')
    except EOFError:
        time.sleep(2)
    searcher.close_camera()
    logger.info('[4] 结束：摄像头已关闭')
    return 0


if __name__ == '__main__':
    sys.exit(main())
