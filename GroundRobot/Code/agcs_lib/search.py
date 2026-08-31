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
        # 当前云台位置（21 水平 / 24 俯仰），扫描与追踪共用起点
        self.x_dis = 500
        self.y_dis = 260

    # ---------------- 模块：24号舵机上下扫描 ----------------
    def ScanNumberTwentyFour(self, pulses=(500, 400, 300, 200, 600, 700, 800), wait=0.5):
        """24号按脉宽序列上下扫描，每档检测，有目标就停下；找不到恢复 24号=260。

        返回 Detection（dict）或 None。
        """
        for p in pulses:
            self.y_dis = p
            self.board.bus_servo_set_position(wait, [[24, p]])
            time.sleep(wait)
            r = self.detect()
            if r is not None:
                self.log.info('[scan24] 24号=%d 检测到目标 center=%s area=%d',
                              p, r['center'], r.get('area', 0))
                return r
            self.log.info('[scan24] 24号=%d 无目标', p)
        # 找不到：24号 恢复回官方初始位置 260（硬编码，官方 color_track 相机初始位）
        self.y_dis = 260
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
            self.x_dis = p
            self.board.bus_servo_set_position(wait, [[21, p]])
            time.sleep(wait)
            r = self.ScanNumberTwentyFour()
            if r is not None:
                self.log.info('[scan21] 21号=%d 找到目标 center=%s area=%d',
                              p, r['center'], r.get('area', 0))
                return r
            self.log.info('[scan21] 21号=%d 未找到', p)
        # 找不到：21号回 500，24号回 260
        self.x_dis = 500
        self.y_dis = 260
        self.board.bus_servo_set_position(wait, [[21, 500], [24, 260]])
        time.sleep(wait)
        self.log.info('[scan21] 未找到目标，21号恢复 500，24号恢复 260')
        return None

    # ---------------- 模块：颜色追踪（目标锁定 + 画面居中，不走动） ----------------
    def TrackColor(self):
        """颜色追踪：锁定目标，持续调整 21/24 让目标保持在画面居中（不走动）。

        借鉴官方 color_track 的 PID 云台跟踪，后台线程持续检测并修正；
        从当前云台位置开始（扫描已记录 x_dis/y_dis）。调用 stop() 停止。
        """
        from agcs_lib.tracker import ColorTracker
        gf = self.params.get('gimbal_fetch', {})
        self.tracker = ColorTracker(
            self.board, self.detect,
            dead_x=int(gf.get('track_dead_cx', 40)),
            dead_y=int(gf.get('track_dead_cy', 60)),
            start_x=self.x_dis, start_y=self.y_dis,
            tilt_sign=int(gf.get('tilt_sign', -1)),
            pan_sign=int(gf.get('pan_sign', 1)),
            p_gain=float(gf.get('track_p_gain', 0.2)),
            settle=float(gf.get('track_settle_ms', 80)) / 1000.0,
        )
        self.tracker.start()
        self.log.info('[track] 颜色追踪已启动：目标锁定并保持画面居中（21=%d 24=%d，不走动）',
                      self.x_dis, self.y_dis)

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
        """停止追踪线程。"""
        self._stop_event.set()
        if getattr(self, 'tracker', None) is not None:
            self.tracker.stop()
