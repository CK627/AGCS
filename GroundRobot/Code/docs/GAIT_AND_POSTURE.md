# 六足「走直、走稳」——步态设计 / 姿态控制 / 姿态自稳

> 调研 + 落地方案，针对 SpiderPi Pro（Hiwonder）`common.kinematics.IK`。
> 结论里的 SDK 参数是从加密的 `spiderpi_sdk/common_sdk/common/kinematics.so`
> 里直接扒出来的（含 `not stripped` 的 debug 段与 docstring），不是猜的。

---

## 0. 一句话结论

1. **你的判断成立**：漂移的根因在步态/机械层，IMU + 颜色只是事后补救。
   但补的顺序要反过来 —— **先标定（前馈），再闭环（反馈）**。
2. **你的 SDK 里已经有正解接口**，只是你没用：
   `setStepMode_whitout_delay(pos, mode, step_velocity, step_amplitude,
   step_height, movement_direction, rotation, speed, times)`
   它带 `movement_direction`（行走方向 0-360）和 `rotation`（叠加旋转 -1~1），
   **支持一边走一边转向**。现在「走 100mm → 停 → 转 1° → 再走」这种离散 bang-bang
   从结构上就收敛不了。
3. 它顺带解决了上次那个问题：`rotation` 承接航向误差 `e`、
   `movement_direction` 承接横向偏差 `cross` —— **两个状态终于有了两个独立执行器**，
   不再像现在这样两个状态抢同一个 `turn` / `left_move` 通道。

---

## 1. 现状：为什么「走 100mm → 停 → 转 1°」结构上就收敛不了

```
go_forward(100mm)  ── 这 100mm 内 SDK 完全不接受反馈 ──┐
stop                                                    │ 误差自由增长
读 IMU → |e|>3° ? turn(round(0.6·e)°)                   │
start  ─────────────────────────────────────────────────┘
```

四个结构性缺陷：

| 缺陷 | 说明 |
|---|---|
| **开环窗口** | 每个 100mm 里 SDK 按「一个周期 10 个动作」走完，中途不接受任何反馈。50mm/s 下这是 2 秒的自由漂移。 |
| **死区比偏差大** | 实测级系统性偏航约 0.5~1.5 °/s，一个 2s 窗口只积 1~3°，恰好在 `TURN_TOL_DEG=3.0` 以下 → **全程看不见、从不修正**，一路累积。这就是「先直后偏」。 |
| **修正粒度取整** | `step = round(0.6 * |e|)` 取整到 1°，|e|=3.1 算出 2°，而六足单次转弯最小步进可能就大于 2° → 左右来回抖（极限环）。 |
| **停-启动注入扰动** | 停下再启动要重新咬合地面，机身惯量 + 足端打滑会额外啃出一次随机偏航。你**每 100mm 就制造一次**。 |

### 离线对照（`CompetitionUse/sim_gait_hold.py`，6 m 行程 × 200 次）

| 变体 | 终点 \|e\|° | 终点横偏 mm | RMS \|e\|° | 峰值 \|e\|° |
|---|---|---|---|---|
| A 现状（走 100mm→停→转 round(0.6e)°，死区 3°） | 2.34 | **249.8** | 3.98 | 12.18 |
| B 连续航向保持 25 Hz（未标定） | 1.08 | **69.5** | 1.40 | 4.09 |
| C 标定后 + 连续航向保持 | 0.98 | **16.0** | 1.23 | 3.72 |

注意 A 的终点误差只有 2.34° 却横偏 250mm —— 因为它**全程顶着 2° 走**，
不是偶尔歪一下。B 的 RMS 只有 1.4° 且是零均值抖，所以横偏只有 70mm。

压力测试下排序稳定：

* 关掉停-启动扰动（`--no-stop-kick`）：A 只从 249.8 → 246mm。**说明主因不是停-启动，是开环窗口 + 死区。**
* 系统偏差放大到 4°/s（`--bias 4.0`，这时离散本该"看得见"）：A 横偏放大到 ~3.6 m，B 346mm，C 仍是 16mm。

---

## 2. GitHub / 论文上的同类做法

### 可直接抄的开源实现

| 仓库 | 关键做法 | 对你的价值 |
|---|---|---|
| [Janga786/hexapod-cpg](https://github.com/Janga786/hexapod-cpg) | Arduino Mega 固件里做 **closed-loop IMU heading hold**；Python 侧用 6 个 Kuramoto 耦合振荡器让 wave/ripple/tripod 从单个参数 ω 涌现，而不是硬编码相位表 | **和你要做的东西几乎一模一样**：连续步态 + IMU 航向闭环。相位从耦合涌现 ⇒  gait 切换不会"跳" |
| [Siedmiu/Hexapod-ROS2-System](https://github.com/Siedmiu/Hexapod-ROS2-System) | wave/bipod/tripod/ripple 四种 gait + **足端接触传感器** + `move_body_based_on_imu.py` | 接触传感器是"检测打滑"的低成本方案；`move_body_based_on_imu` 是你要的 L2 姿态控制 |
| [KevinOchs/hexapod_ros](https://github.com/KevinOchs/hexapod_ros)（及一堆 fork） | 正弦 tripod；订阅 `imu/data` → 输出 `body_scalar` 做 **auto body leveling** | 姿态控制的经典接线：IMU → 机体姿态偏移，脚不动 |
| [NguyenTrongPhuc552003/reinforcement-autonomous-hexapod](https://github.com/NguyenTrongPhuc552003/reinforcement-autonomous-hexapod) | **Balance Mode**：IMU 反馈 + `response factor` + `deadzone` 两个可调量；MPU6050 内核驱动 | 自稳环的参数就这两个，别搞复杂；它还做了逐舵机 `calibration.cpp` |
| [rohitguta2432/hexapod-simulator](https://github.com/rohitguta2432/hexapod-simulator) | 解析 IK；**支撑相足端在机体坐标系里严格以机体速度后移 ⇒ 零打滑** | 这句话是"走直"的第一性原理：打滑 = 足端相对地面有速度 |
| [rafael-andre1/Hexapod-RLvsIRL](https://github.com/rafael-andre1/Hexapod-RLvsIRL) | Webots + PPO，奖励里显式惩罚 **body oscillation / IMU 方差** | 如果你后面想上 RL，这是最省事的起点（但比赛前不建议） |

### 论文（有实测数字，可直接当设计依据）

* **MDPI Appl. Sci. 2021, 11, 3714 — *Straight Gait Research of a Small Electric Hexapod Robot***
  * 「**增大占空比**」策略（腿要进入摆动相时不立刻摆，先多支撑一会儿）：
    roll / pitch / yaw 的 RMSE 分别降 **35% / 25% / 12%**。
  * 基于运动学的 **yaw PD 校正**（Kp=0.01, Kd=0.005）：yaw RMSE **4.141° → 1.327°**，漂移限制在 **±3°**。
  * 不做占空比策略时：8 个 tripod 周期理论 480mm，**实走只到 410mm**（做策略是 460mm）——**打滑直接吃掉 14% 行程**。
  * tripod vs 四足 vs 五足：腿落得越多，姿态波动和 yaw 越小，但打滑和行程误差越大。
  * ⚠️ 对你最有用的一条：**它的 yaw 校正是在步态里改支撑相足端的 Y 轨迹**，
    不需要"停下来转"。这正是 `setStepMode` 的 `rotation` 在你 SDK 里的位置。
* **Frontiers Neurorobot. 2017 — Tegotae-based hexapedal coordination**：
  **左右腿占空比不对称 = 转弯**。所以你看到"机器人自己往一边拐"，
  第一嫌疑人就是左右腿零位/摩擦不对称，不是 IMU 坏了。
* **Corin hexapod state estimator（EKF）灵敏度分析**：
  占空比、步长与打滑概率的关系**不是单调的** —— 步长大了单脚可滑距离长，
  但步数少了撞击事件也少。所以**步长必须现场扫，不能拍脑袋**。

---

## 3. 你的 SDK 到底有什么（从 `kinematics.so` 扒出来的）

`kinematics.IK` 可见方法：

```
stand(pos, t)          go_forward(pos, mode, step_amplitude, step_velocity, times)
back / turn_left / turn_right / left_move / right_move
moveBody(pos, [dx,dy,dz], [rx,ry,rz], speed)      # 6-DOF 机体姿态，脚不动
setStepMode(...)                                   # ★ 连续步态
setStepMode_whitout_delay(...)                     # ★ 连续步态（无延时版）
setPosition / setAngle / moveToPosition / leg
stopMove / stop_move
```

`setStepMode` 的 docstring（原样）：

```
:param pos:                 初始位置/mm
:param mode:                步态模式，1:Ripple Gait，2:Tripod Gait, 3:四足，4：四足
:param step_velocity:       速度
:param step_amplitude:      幅度/mm
:param step_height:         高度/mm
:param movement_direction:  方向/0-360
:param rotation:            旋转角/-1~1
:param speed:               舵机速度/ms, 一个周期10个动作，前进幅度为step_amplitude，
                            所以移动速度为 step_amplitude/(speed*10)
:param times:               执行次数
```

其它可用的东西：

| 名称 | 用途 |
|---|---|
| `initial_pos` / `initial_pos_high` / `initial_pos_quadruped` | 三套站姿；`_high` 可以抬高/降低重心 |
| `CALIBRATION_DEVIATION` / `DEFLECTION_ANGLE` | **舵机零位偏差常量 —— 标定的入口** |
| `current_pos` / `last_pos` / `last_status` | 读回实际足端位置（标定要用） |
| `roll` `pitch` `yaw` / `roll_x/y/z` `pitch_x/z` `yaw_x/y/z` | 机体姿态内部量 |
| `LEG_MOVEMENT_INDEX` / `LINK1..3` / `FACTOR` | 腿序与杆长 |

**你现在的 `Auto-capture.py` 只用了 `go_forward` / `back` / `turn_*` / `left_move` /
`right_move` / `moveBody`，完全没碰 `setStepMode`。** 这是最大的浪费。

---

## 4. 四层落地方案

### L0 · 标定（**前馈，最该先做，收益最大**）

六足走歪的头号机械原因：**6 条腿零位不对称**。左右 coxa 各差 1°，
每个 gait 周期就啃出一点 yaw，走 6 m 就是几百毫米。这是**开环能修的部分**，
修完闭环的压力直接小一个数量级（仿真里 C vs B：70mm → 16mm）。

做法（从易到难）：

1. 站姿下逐腿量足端坐标，和 `initial_pos` 对比，看左右是否镜像对称。
2. 让机器人直走 6 m，量终点横偏 `Δx` 和偏航 `Δθ`，反推等效零位偏差，
   写进 `CALIBRATION_DEVIATION` 一类的常量（需要先 `probe_gait_api.py` 确认它可写）。
3. 上 `calibration.cpp` 那种逐舵机 offset 表（参考 reinforcement-autonomous-hexapod）。

### L1 · 步态设计（**改参数，不改代码**）

对应论文的「增大占空比」策略，在你 SDK 里就是 **小步幅 + 中低速**：

| 参数 | 现在 | 建议先试 | 理由 |
|---|---|---|---|
| `step_amplitude` | —（`go_forward` 第 3 参，现用 100mm 的段长） | **40~60mm** | 步幅小 → 单脚可滑距离短 → 打滑少（论文：不做策略时行程被吃掉 14%） |
| `step_velocity` | 50 mm/s | **30~40 mm/s** | 慢 → 支撑相占比自然上升（等价增大占空比） |
| `step_height` | 未设 | **20~30mm** | 太高 → 落腿冲击大 → 咬合瞬间啃 yaw；太低 → 拖脚 |
| `mode` | 2 (Tripod) | 平地比赛保持 **2**；需要稳时试 **1 (Ripple)** | Ripple 同时支撑腿更多，姿态更稳但更慢 |

⚠️ 步长和打滑**不是单调关系**（Corin 的灵敏度分析），所以这三行必须
**现场扫一遍**（比如 40/60/80mm × 30/50 mm/s 扫 9 组，每组走 2 m 量横偏）。

### L2 · 姿态控制（`moveBody`，脚不动）

机身不水平 → 重心偏出支撑多边形中心 → 各腿载荷不均 → 打滑方向性 → yaw 漂移。

```
roll_err  = imu_roll  - 0
pitch_err = imu_pitch - 0
ik.moveBody(ik.initial_pos, [0, 0, dz], [-k*roll_err, -k*pitch_err, 0], speed)
```

* 只在**段间**（停下时）做，别在步态中间插 `moveBody`，会和 `setStepMode` 打架。
* 降低重心用 `ik.initial_pos_high` 的反方向（重心低 → 稳定裕度大，但注意别蹭地）。
* 参考 `Siedmiu/Hexapod-ROS2-System` 的 `move_body_based_on_imu.py`。

### L3 · 姿态自稳 / 连续航向保持（`setStepMode_whitout_delay`）

这是核心改动。**不要再 stop → turn → go**：

```python
hold = GaitHold(ik, ik.initial_pos, mode=2,
                amplitude=50, height=25, step_velocity=40,
                rot_max_dps=<标定值>, kp=1.2, kd=0.08, hz=25)
hold.start()
while 还没到:
    hold.step(target_yaw, imu_state['yaw'], lateral_mm=fusion.cross)
    time.sleep(1/25)
hold.stop()
```

控制律：

```
e        = wrap180(target_yaw - yaw)          # 右正
out_dps  = kp*e + kd*(-yaw_rate)              # PD
rotation = clamp(out_dps / rot_max_dps, -1, 1)
dir      = base_dir + clamp(lat_gain*cross, ±lat_max)   # 蟹行纠横向
```

**必须先标定 `rot_max_dps`**（`rotation=1.0` 时机器人实际的偏航角速度），
用 `CompetitionUse/probe_gait_api.py --spin` 在真机上量。没量之前这个模块默认关闭。

---

## 5. 和上次做的 `LaneFusion` 是什么关系

上次我把 IMU 和颜色融合成两个状态 `(e 航向误差, cross 横向偏差)`，
但**执行器只有一个**（`turn_*` / `left_move`），所以只能「先航向后横向，
同一小块只发一种指令」—— 那还是耦合的，只是耦合得干净了一点。

有了 `setStepMode` 之后：

```
LaneFusion 状态          执行器                        SDK 参数
─────────────────────────────────────────────────────────────
e      航向误差    →   机身自转              →   rotation  (-1~1)
cross  横向偏差    →   蟹行（不转向）        →   movement_direction (0-360)
```

**这才是真正的解耦**：横向纠偏不用再付"顺带把头扭过去"的代价，
航向纠偏也不用再付"顺带平移"的代价。

---

## 6. 建议的实验顺序（每步都能独立验证，做完一步再看下一步）

| # | 做什么 | 怎么判成败 |
|---|---|---|
| 1 | 跑 `probe_gait_api.py`（机器人上） | 拿到 `setStepMode` 真实签名；确认有无 `setStepMode_whitout_delay` |
| 2 | 跑 `probe_gait_api.py --spin` | 拿到 `rot_max_dps`；检查 rotation 是否线性 |
| 3 | **标定**：站姿量 6 腿对称性 + 直走 6 m 反推 | 直走 6 m 横偏 < 100mm（现在是 250mm） |
| 4 | **步长扫描**：40/60/80mm × 30/50 mm/s，各走 2 m | 挑出横偏最小的一组 |
| 5 | 接 `GaitHold`（先只走直线、关掉颜色） | 6 m 横偏 < 50mm、RMS \|e\| < 1.5° |
| 6 | 再把 `LaneFusion.cross` 接到 `movement_direction` | 全程横偏 < 30mm |
| 7 | 最后才回到颜色/夹取 | 原来那套 `ref_cx` / tilt 搜索 大概率可以直接删掉 |

---

## 7. 文件清单

| 文件 | 状态 | 说明 |
|---|---|---|
| `docs/GAIT_AND_POSTURE.md` | 新增 | 本文档 |
| `agcs_lib/gait_hold.py` | 新增 | `GaitHold` 连续航向保持控制器，默认不接主流程 |
| `CompetitionUse/probe_gait_api.py` | 新增 | 在机器人上探测/标定连续步态接口 |
| `CompetitionUse/sim_gait_hold.py` | 新增 | 离散 vs 连续离线对照（本文第 1 节的数字来源） |
| `CompetitionUse/Auto-capture.py` | **未改动** | 等标定数据回来再接 |
