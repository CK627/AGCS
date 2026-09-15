#!/usr/bin/python3
# coding=utf8
"""IMU 航向（yaw）积分封装。

Board 只给角速度（`get_imu()` 返回 ax, ay, az, gx, gy, gz，取 gz 为偏航角速度），
没有磁力计绝对航向，所以 yaw 是相对开机时刻的积分值：先静止标定零漂，再按 dt 积分。
长时间会缓慢漂移，适合「每次转角闭环」这种短时用途，不适合当全局绝对朝向。
"""
import time


class ImuYaw:
    def __init__(self, board, gain=1.18, calib_samples=200):
        self.board = board
        self.gain = gain          # 积分增益（实测标定，补偿角速度标度误差）
        self.calib_samples = calib_samples
        self.bias = 0.0           # gz 零漂
        self.yaw = 0.0            # 相对开机时刻的航向角（度，左正右负）
        self._last_t = time.monotonic()

    def read_gz(self):
        """读一帧 gz 角速度；无数据返回 None。"""
        try:
            data = self.board.get_imu()
        except Exception:
            return None
        if data is None:
            return None
        return float(data[5])

    def calibrate(self, timeout=3.0):
        """静止标定 gz 零漂并清零航向（机器人必须静止）。返回采集到的样本数。"""
        self.board.enable_reception()
        vals = []
        deadline = time.monotonic() + timeout
        while len(vals) < self.calib_samples and time.monotonic() < deadline:
            gz = self.read_gz()
            if gz is not None:
                vals.append(gz)
            time.sleep(0.005)
        self.bias = sum(vals) / len(vals) if vals else 0.0
        self.reset()
        return len(vals)

    def update(self):
        """取最新一帧角速度积分进 yaw，返回当前 yaw。需按固定节奏（如 50Hz）持续调用。"""
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        gz = self.read_gz()
        if gz is not None:
            self.yaw += (gz - self.bias) * dt * self.gain
        return self.yaw

    def reset(self):
        self.yaw = 0.0
        self._last_t = time.monotonic()
