"""连续步态航向保持（gait-level heading hold）。

背景
----
Auto-capture.py 现在的做法是「开环走一段 → 停下 → 读 IMU → 转 1° → 再走」，
属于 **离散 bang-bang**：

    go_forward(100mm) ── 这 100mm 内 SDK 完全不接受反馈 ──┐
    stop                                                  │ 误差自由增长
    turn_left/right(1°)  ← 粒度 1°、死区 3°  → 极限环     │
    go_forward(100mm) ────────────────────────────────────┘

停下-重新启动本身就是最大的偏航扰动源（机身惯量 + 足端重新咬合）。

SpiderPi Pro 的 kinematics 库（加密的 kinematics.so）其实提供了连续接口：

    setStepMode(pos, mode, step_velocity, step_amplitude, step_height,
                movement_direction, rotation, speed, times)
    setStepMode_whitout_delay(同上，无延时版本)
    stopMove()

其中：
    mode           1=Ripple Gait, 2=Tripod Gait, 3/4=四足
    step_amplitude 步幅 /mm
    step_height    抬腿高度 /mm
    movement_direction 行走方向 /0-360      ← 横向纠偏（蟹行）
    rotation       叠加的旋转 /-1~1         ← 航向纠偏
    speed          舵机速度 /ms，一个周期 10 个动作，
                   移动速度 = step_amplitude/(speed*10)

**这就是「边走边转向」接口**：在 20~50Hz 循环里持续刷新 rotation，
就能做真正的 IMU 航向保持，不需要停下来。

它还有一个额外好处：movement_direction 与 rotation 是**两个独立执行器**，
正好可以分别承接 (横向偏差 cross, 航向误差 e) 两个状态 ——
不像现在这样两个状态抢同一个 turn/left_move 通道。

重要前提
--------
`rotation` 的 -1~1 到底是「角速度」还是「每周期转角」，无法离线确认。
必须先跑 `CompetitionUse/probe_gait_api.py` 在真机上量出
「rotation=1 时的偏航角速度 ω_max（度/秒）」，再填 `--rot-max-dps`。
在量出来之前，本模块**默认关闭**，主流程行为完全不变。
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
    """连续的「边走边修正」控制器。

    用法（闭环里每 tick 调一次 step）：

        hold = GaitHold(ik, pos, mode=2, amplitude=40, height=30,
                        servo_speed=60, rot_max_dps=90.0)
        hold.start()
        while not done:
            yaw, yaw_rate = imu_read()          # 度 / 度每秒
            hold.step(target_yaw, yaw, yaw_rate, lateral_mm=0.0)
            time.sleep(1.0 / hold.hz)
        hold.stop()
    """

    def __init__(self, ik, pos,
                 mode=2,
                 amplitude=40.0,      # 步幅 mm（小步 = 少打滑，见文档 L1）
                 height=25.0,         # 抬腿高度 mm
                 step_velocity=50.0,  # 行走速度 mm/s
                 servo_speed=60,      # 舵机速度 ms
                 movement_dir=0.0,    # 基准行走方向 0-360（0 = 正前）
                 hz=25.0,             # 闭环刷新率
                 kp=0.6,              # 航向 P：rotation = kp*e + kd*edot
                 kd=0.05,
                 rot_max_dps=90.0,    # rotation=1.0 对应的偏航角速度（度/秒）★需标定
                 lat_gain=0.02,       # 横向偏差 -> 方向角偏移（度/mm）
                 lat_max_deg=12.0,
                 deadband_deg=0.3,
                 times=1,
                 verbose=False):
        self.ik = ik
        self.pos = pos
        self.mode = mode
        self.amplitude = amplitude
        self.height = height
        self.step_velocity = step_velocity
        self.servo_speed = servo_speed
        self.base_dir = movement_dir
        self.hz = hz
        self.kp = kp
        self.kd = kd
        self.rot_max_dps = rot_max_dps
        self.lat_gain = lat_gain
        self.lat_max_deg = lat_max_deg
        self.deadband_deg = deadband_deg
        self.times = times
        self.verbose = verbose

        self._has_cont = hasattr(ik, 'setStepMode_whitout_delay')
        self._prev_err = None
        self._last_t = None
        self._integral = 0.0
        self.running = False

    # ---------- 生命周期 ----------

    def start(self):
        self.running = True
        self._prev_err = None
        self._last_t = None
        self._integral = 0.0
        if not self._has_cont:
            raise RuntimeError(
                '当前 kinematics 没有 setStepMode_whitout_delay，'
                '无法使用连续航向保持；请退回离散模式')

    def stop(self):
        """收尾：先让步态停住，再回站姿。"""
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

    def step(self, target_yaw, yaw, yaw_rate=None, lateral_mm=0.0):
        """根据当前 IMU 航向算一次指令并下发。

        target_yaw / yaw : 度，右正（与 ImuTracker 一致）
        yaw_rate         : 度/秒；None 时由差分估计
        lateral_mm       : 横向偏差，右正（来自 LaneFusion 的 cross）
        """
        if not self.running:
            return 0.0

        err = wrap180(target_yaw - yaw)          # 右正：需要向右转
        now = time.time()

        if yaw_rate is None:
            if self._last_t is not None and self._prev_err is not None:
                dt = max(1e-3, now - self._last_t)
                yaw_rate = wrap180(err - self._prev_err) / dt
            else:
                yaw_rate = 0.0
        self._last_t = now
        self._prev_err = err

        # 死区：避免围绕 0 抖
        if abs(err) < self.deadband_deg:
            out_dps = 0.0
        else:
            out_dps = self.kp * err + self.kd * (-yaw_rate)   # D 用阻尼项

        rotation = clamp(out_dps / max(1e-6, self.rot_max_dps), -1.0, 1.0)

        # 横向偏差 -> 行走方向角（蟹行），与航向控制解耦
        dir_offset = clamp(self.lat_gain * lateral_mm,
                           -self.lat_max_deg, self.lat_max_deg)
        movement_dir = (self.base_dir + dir_offset) % 360.0

        self._send(rotation, movement_dir)
        if self.verbose:
            print('hold e=%+6.2f°  yaw_rate=%+6.1f°/s  rot=%+.3f  dir=%.1f'
                  % (err, yaw_rate, rotation, movement_dir), flush=True)
        return rotation

    # ---------- 下发 ----------

    def _send(self, rotation, movement_dir):
        # 签名按 kinematics.so 内 docstring 的参数顺序
        # (pos, mode, step_velocity, step_amplitude, step_height,
        #  movement_direction, rotation, speed, times)
        self.ik.setStepMode_whitout_delay(
            self.pos,
            self.mode,
            self.step_velocity,
            self.amplitude,
            self.height,
            movement_dir,
            rotation,
            self.servo_speed,
            self.times,
        )


def run_segment_hold(ik, pos, hold, imu_state, target_yaw,
                     distance_mm, lateral_fn=None, max_s=120.0):
    """走完一段：连续前进 + 连续航向保持。

    distance_mm 用「速度 × 时间」估算（步态是速度控制的，不是位移控制的），
    所以到达判定是软的；需要精确位移时仍然要靠外部标记/相机。
    """
    speed_mm_s = hold.step_velocity
    t_end = time.time() + max_s
    t_stop = time.time() + (distance_mm / max(1e-6, speed_mm_s))
    hold.start()
    try:
        while time.time() < min(t_end, t_stop):
            yaw = imu_state.get('yaw', 0.0)
            rate = imu_state.get('rate_dps')
            lat = lateral_fn() if lateral_fn else 0.0
            hold.step(target_yaw, yaw, rate, lat)
            time.sleep(1.0 / hold.hz)
    finally:
        hold.stop()
