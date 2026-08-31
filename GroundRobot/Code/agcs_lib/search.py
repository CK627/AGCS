#!/usr/bin/python3
# coding=utf8
"""搜索/逼近模块（实现已清空，待重新设计）。

原实现过于杂乱，已按 docs/SEARCH_FLOWCHART.md 的流程图清除。
此处只保留对外接口占位，保证 tasks/auto_fetch.py 等调用方不报错。
重新设计后请保持第 5 节接口契约：

    class Searcher:
        def __init__(self, board, ik, ak, params, detect,
                     ultrasonic=None, display=None, tof=None)
        def run(self) -> (center, cy) | (None, None)
        def reset_pose(self)      # 云台回中 21=500/24=260
        def stop(self)            # 停止内部线程
"""
import threading


class Searcher:
    """占位：搜索/逼近逻辑已清空，等待按流程图重新设计。"""

    def __init__(self, board, ik, ak, params, detect,
                 ultrasonic=None, display=None, tof=None):
        self._stop_event = threading.Event()
        self._status = 0

    def run(self):
        """占位：直接返回未找到（不动作），待重新设计。"""
        self._status = 1
        return None, None

    def reset_pose(self):
        """占位：云台回中待重新设计。"""
        self._status = 0

    def stop(self):
        """占位：线程清理待重新设计。"""
        self._stop_event.set()
