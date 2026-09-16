#!/usr/bin/python3
# coding=utf8
"""IMU 航向积分：后台线程密集采样。

## 为什么必须是后台线程密集采样

官方 SDK（`spiderpi_sdk/common_sdk/common/ros_robot_controller_sdk.py`）里：

    self.imu_queue = queue.Queue(maxsize=1)          # 只留一个样本
    def packet_report_imu(self, data):
        try:    self.imu_queue.put_nowait(data)
        except queue.Full: pass                      # 没人取就直接丢

`get_imu()` 拿到的是「上一次取走之后到达的第一个样本」，**两次调用之间的样本全被丢掉**。

比赛脚本原来每走完一段 100mm（1.5~1.9 秒）才调一次 `get_imu()`，拿一个瞬时样本乘上整段
`dt`。那个瞬时样本落在六足步态周期的哪个相位纯属偶然，而这一段的真实平均角速度可能是 0，
乘出来就是一个随机方向的几度~十几度假转角。

现场后果（2026-09-15 实测）：直线阶段 `yaw` 每段被灌进约 −13° 的假转角 → 控制器误以为
一直偏左 → 每一段都发「左转」→ 机器人真的朝左转飞，误差从 +13° 一路涨到 +17° 从不收敛，
最后色块转出画面。日志里 `rate` 稳定在 −7°/s、而机器人实际走得基本是直的。

## 实测依据（`CompetitionUse/imu_probe.py`）

| 测的东西 | 结果 |
|---|---|
| IMU 出数速率 | 约 **105 Hz**，紧循环每 9.5ms 取到一个新样本 |
| 静止 2 秒按真实样本间隔积分 | **+0.020°**（密集采样下零漂基本不累积） |
| 同一个 `ik.turn_left(20°)` 调用 | 密集积分 **+13.26°**；对照旧做法（单样本 × 整段 dt）根本积不到 |

## 为什么不跟舵机指令抢串口

`get_imu()` 只是从队列取数（队列由 SDK 自己的 `recv_task` 线程填充），**不写串口**；
舵机指令走 `buf_write()` 的串口写路径。两者不共享锁，所以本线程不会干扰指令下发。
（`bus_servo_read_*` 用的是另一把 `servo_read_lock`，本线程也不碰。）
"""

import threading
import time


class ImuTracker(threading.Thread):
    """后台持续读 IMU 队列并积分 yaw，结果写进调用方给的 state 字典。

    state 用到的键与比赛脚本原来的 `imu_state` 一致：`bias` / `yaw` /
    `last_rate` / `last_dt`。字典的单项读写是原子的，主线程直接读 `state['yaw']`
    不需要加锁。
    """

    def __init__(self, board, state, scale_left=1.177, scale_right=1.199):
        super().__init__(daemon=True)
        self.board = board
        self.state = state
        self.scale_left = scale_left
        self.scale_right = scale_right
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._calibrating = False
        self._bias_sum = 0.0
        self._bias_n = 0
        self.samples = 0        # 累计取到的样本数，用于判断线程是否真的在跑
        self._last_t = None
        self._mark_t = None
        self._mark_yaw = 0.0

        state.setdefault('bias', 0.0)
        state.setdefault('yaw', 0.0)
        state.setdefault('last_rate', 0.0)
        state.setdefault('last_dt', 0.0)

    # ---------- 线程主体 ----------
    def run(self):
        try:
            self.board.enable_reception()   # make_board() 默认没开接收，这里自包含地开启
        except Exception:
            pass
        while not self._stop_evt.is_set():
            data = self.board.get_imu()
            if data is None:
                time.sleep(0.001)   # 队列空。IMU 约 105Hz 出数，等 1ms 再来
                continue
            now = time.monotonic()
            gz = float(data[5])
            with self._lock:
                if self._last_t is not None:
                    # 用「距上次取到样本」的真实间隔积分。单样本间隔只有几毫秒，
                    # 坏样本乘出来的误差可以忽略，不再需要 dt 钳位之类的补丁。
                    dt = now - self._last_t
                    rate = gz - self.state['bias']
                    scale = self.scale_left if rate >= 0 else self.scale_right
                    self.state['yaw'] += rate * dt * scale
                self._last_t = now
                self.samples += 1
                if self._calibrating:
                    self._bias_sum += gz
                    self._bias_n += 1

    # ---------- 对外接口 ----------
    def calibrate(self, seconds=0.5, settle=0.25):
        """重标 gz 零漂：先等 settle 秒让机身晃动静下来，再采 seconds 秒取均值。

        为什么要有 settle：这个方法基本都在刚转完弯之后调用，而六足转身时整个机身
        还在晃，这时候采到的不是真实零漂。零漂标错多少，后面整段直行就照着错多少积分。

        返回 (零漂, 采样数)。
        """
        if settle > 0:
            time.sleep(settle)
        with self._lock:
            self._bias_sum = 0.0
            self._bias_n = 0
            self._calibrating = True
        time.sleep(seconds)
        with self._lock:
            self._calibrating = False
            if self._bias_n:
                self.state['bias'] = self._bias_sum / self._bias_n
            return self.state['bias'], self._bias_n

    def reset(self):
        """航向归零，并把「自上次统计」的基准一起挪到 0。

        注意不清 `_last_t`：积分是连续的，断了反而会在下一次补上一个错误的大 dt。
        """
        with self._lock:
            self.state['yaw'] = 0.0
            self._mark_yaw = 0.0
            self._mark_t = time.monotonic()

    def since_last(self):
        """取「自上次调用以来」的平均角速度与时长，供现场日志对照。

        机器人站在地上不动时它应该接近 0；若常在 ±0.5°/s 以上，说明零漂没标定准。
        """
        with self._lock:
            now = time.monotonic()
            if self._mark_t is None:
                self._mark_t = now
                self._mark_yaw = self.state['yaw']
                return 0.0, 0.0
            dt = now - self._mark_t
            dyaw = self.state['yaw'] - self._mark_yaw
            self._mark_t = now
            self._mark_yaw = self.state['yaw']
        return (dyaw / dt if dt > 0 else 0.0), dt

    def stop(self):
        self._stop_evt.set()
