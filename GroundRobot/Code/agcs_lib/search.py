#!/usr/bin/python3
# coding=utf8
"""搜索/逼近模块（按流程图分步重建中，一步一检）。

已实现模块：
    ScanNumberTwentyFour()   24号舵机上下扫描目标检测
                             （序列 500-400-300-200-600-700-800，每档检测，有目标就停下；
                               找不到恢复 24号=260）
    ScanNumberTwentyOne()    21号舵机水平扫描
                             （序列 500-300-200-700-800，每档调用 24号扫描；
                               找不到恢复 21号=500、24号=260）
待实现：
    run() 完整搜索/逼近（当前占位，返回未找到）
    （恢复初始状态/摄像头启动已拆到 agcs_lib/restore.py 的 Restore 类）

对外接口契约（tasks/auto_fetch.py 依赖）：
    Searcher(board, ik, ak, params, detect, ultrasonic=None, display=None, tof=None)
    run() -> (center, cy) | (None, None)
    reset_pose() / stop()
"""
import threading
import time

from agcs_lib.logs import get_logger


class Searcher:
    """搜索/逼近（逐步重建中）。"""

    def __init__(self, board, ik, ak, params, detect,
                 ultrasonic=None, display=None, tof=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.detect = detect
        self.ultrasonic = ultrasonic
        self.display = display
        self.tof = tof
        self.log = get_logger('search')
        self._stop_event = threading.Event()

    # ---------------- 模块：24号舵机上下扫描 ----------------
    def ScanNumberTwentyFour(self, pulses=(500, 400, 300, 200, 600, 700, 800), wait=0.5):
        """24号按脉宽序列上下扫描，每档检测，有目标就停下；找不到恢复 24号=260。

        返回 Detection（dict）或 None。
        """
        for p in pulses:
            self.board.bus_servo_set_position(wait, [[24, p]])
            time.sleep(wait)
            r = self.detect()
            if r is not None:
                self.log.info('[scan24] 24号=%d 检测到目标 center=%s area=%d',
                              p, r['center'], r.get('area', 0))
                return r
            self.log.info('[scan24] 24号=%d 无目标', p)
        # 找不到：24号 恢复回官方初始位置 260（硬编码，官方 color_track 相机初始位）
        self.board.bus_servo_set_position(wait, [[24, 260]])
        time.sleep(wait)
        self.log.info('[scan24] 未找到目标，24号恢复回官方初始位置 260')
        return None

    # ---------------- 模块：21号舵机水平扫描 ----------------
    def ScanNumberTwentyOne(self, pan_pulses=(500, 300, 200, 700, 800), wait=0.5):
        """21号按脉宽序列扫描，每档调用 24号上下扫描；发现目标即停。

        找不到则 21号回 500、24号回 260。返回 Detection（dict）或 None。
        """
        for p in pan_pulses:
            self.board.bus_servo_set_position(wait, [[21, p]])
            time.sleep(wait)
            r = self.ScanNumberTwentyFour()
            if r is not None:
                self.log.info('[scan21] 21号=%d 找到目标 center=%s area=%d',
                              p, r['center'], r.get('area', 0))
                return r
            self.log.info('[scan21] 21号=%d 未找到', p)
        # 找不到：21号回 500，24号回 260
        self.board.bus_servo_set_position(wait, [[21, 500], [24, 260]])
        time.sleep(wait)
        self.log.info('[scan21] 未找到目标，21号恢复 500，24号恢复 260')
        return None

    # ---------------- 占位：完整搜索/逼近（待按流程图实现） ----------------
    def run(self):
        """占位：直接返回未找到（不动作），待重新设计。"""
        self._stop_event.clear()
        return None, None

    def reset_pose(self):
        """兼容旧接口：云台回中。"""
        self.board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        time.sleep(0.5)

    def stop(self):
        """占位：线程清理待重新设计。"""
        self._stop_event.set()
