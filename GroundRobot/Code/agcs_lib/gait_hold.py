"""连续步态航向保持（gait-level heading hold）—— move + bang-bang 版。

背景
----
实测（2026-09-17）结论：
  - 边走边转的稳定接口是 `move(pos, mode, amplitude, movement_direction,
    rotation, speed, times)`，**不是** `setStepMode`（后者「转两下就停」不稳定）。
  - `rotation`（-1~1）近似**恒定角速度** ~1.7°/s（rotation=1.0），不是比例量；
    且**符号反**：正 rotation → 负 yaw。所以用 bang-bang（误差超死区就发固定
    rotation），不做比例换算。
  - `move` 是**阻塞**调用（一个 times=1 周期约 0.6s），闭环节奏由 move 自己
    驱动，不再 25Hz 流式下发。

用法（闭环里循环调 step，每次内部阻塞一个周期）：
    hold = GaitHold(ik, pos, mode=2, amplitude=40, servo_speed=60,
                    rot_magnitude=0.5, rot_max_dps=1.7)
    hold.start()
    while not done:
        yaw = imu_read()
        hold.step(target_yaw, yaw, lateral_mm=0.0)
    hold.stop()

重要前提
--------
`rot_max_dps`（rotation=1.0 时的偏航角速度）与 `rotation_sign`（符号）都要先
用 `CompetitionUse/probe_gait_api.py --spin`（走 move）在真机上标定。标出来之前
本模块默认关闭，主流程行为不变。
"""

import math
import time


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(deg):
    """把角度归一化到 (-180, 180]。"""
    d = (deg + 180.0) % 360.0 - 180.0
    return d if d > -180.0 else 180.0


class GaitHold:
    """边走边修正的 bang-bang 控制器（move 版）。

    每个 step() 内部调一次阻塞的 move(times)，所以调用方**不要**再额外 sleep 定频，
    step 本身的阻塞就是节奏。
    """

    def __init__(self, ik, pos,
                 mode=2,
                 amplitude=40.0,      # 步幅 mm（小步 = 少打滑）
                 servo_speed=60,      # 舵机速度 ms
                 movement_dir=0.0,    # 基准行走方向 0-360（0 = 正前）
                 rot_magnitude=0.5,   # bang-bang 固定 rotation 幅值（0~1）
                 rot_max_dps=1.7,     # rotation=1.0 的偏航角速度（度/秒）★标定
                 rotation_sign=-1.0,  # 符号修正：实测 +rotation → 负 yaw
                 deadband_deg=0.5,    # 死区（度）
                 lat_gain=0.02,       # 横向偏差 -> 方向角偏移（度/mm）
                 lat_max_deg=12.0,
                 times=1,             # 每次 move 的执行周期数
                 verbose=False):
        self.ik = ik
        self.pos = pos
        self.mode = mode
        self.amplitude = amplitude
        self.servo_speed = servo_speed
        self.base_dir = movement_dir
        self.rot_magnitude = rot_magnitude
        self.rot_max_dps = rot_max_dps
        self.rotation_sign = rotation_sign
        self.deadband_deg = deadband_deg
        self.lat_gain = lat_gain
        self.lat_max_deg = lat_max_deg
        self.times = times
        self.verbose = verbose

        self._has_move = hasattr(ik, 'move')
        self.running = False

    # ---------- 生命周期 ----------

    def start(self):
        if not self._has_move:
            raise RuntimeError(
                '当前 kinematics 没有 move，无法使用连续航向保持；请退回离散模式')
        self.running = True

    def stop(self):
        """收尾：先发一个直行周期清掉 rotation，再停步态。"""
        if self.running:
            self._send(0.0, self.base_dir)
            self.running = False
        fn = getattr(self.ik, 'stopMove', None) or getattr(self.ik, 'stop_move', None)
        if fn is not None:
            try:
                fn()
            except Exception:
                pass

    # ---------- 每 tick ----------

    def step(self, target_yaw, yaw, lateral_mm=0.0):
        """根据 IMU 航向算一次 bang-bang 指令并下发（内部阻塞一个周期）。

        target_yaw / yaw : 度，右正（与 ImuTracker 一致）
        lateral_mm       : 横向偏差，右正（来自 LaneFusion 的 cross）
        """
        if not self.running:
            return 0.0

        err = wrap180(target_yaw - yaw)

        # bang-bang：误差超死区就发固定幅值 rotation；符号按标定修正
        if abs(err) < self.deadband_deg:
            rotation = 0.0
        else:
            rotation = self.rotation_sign * self.rot_magnitude * (1.0 if err > 0 else -1.0)

        # 横向偏差 -> 行走方向角（蟹行），与航向控制解耦
        dir_offset = clamp(self.lat_gain * lateral_mm,
                           -self.lat_max_deg, self.lat_max_deg)
        movement_dir = (self.base_dir + dir_offset) % 360.0

        self._send(rotation, movement_dir)
        if self.verbose:
            print('hold e=%+6.2f°  rot=%+.3f  dir=%.1f'
                  % (err, rotation, movement_dir), flush=True)
        return rotation

    # ---------- 下发 ----------

    def _send(self, rotation, movement_dir):
        # move 签名：(pos, mode, amplitude, movement_direction, rotation, speed, times)
        self.ik.move(
            self.pos,
            self.mode,
            self.amplitude,
            movement_dir,
            rotation,
            self.servo_speed,
            self.times,
        )

    # ---------- 估算 ----------

    def speed_mm_s(self):
        """估算行走速度 mm/s（约 amplitude/(servo_speed*10)，一个周期 10 个动作）。"""
        return self.amplitude / max(1e-6, self.servo_speed * 10.0) * 1000.0


def run_segment_hold(ik, pos, hold, imu_state, target_yaw,
                     distance_mm, lateral_fn=None, max_s=120.0):
    """走完一段：连续前进 + bang-bang 航向保持。

    move 是阻塞的，step() 内部走一个周期，所以这里不用再 sleep；到达判定用
    「速度 × 时间」软估，需要精确位移时仍靠外部标记/相机。
    """
    t_end = time.time() + max_s
    t_stop = time.time() + (distance_mm / max(1e-6, hold.speed_mm_s()))
    hold.start()
    try:
        while time.time() < min(t_end, t_stop):
            yaw = imu_state.get('yaw', 0.0)
            lat = lateral_fn() if lateral_fn else 0.0
            hold.step(target_yaw, yaw, lat)
    finally:
        hold.stop()
