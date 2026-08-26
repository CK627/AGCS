# Python 与无人机互通：库函数详解（无真机也能学）

> 定位：真机到货前，把"Python 怎么和无人机说话"彻底搞懂。
> 看完本文，任何一条 pymavlink 代码，你都能说出它对应无人机上的什么。
> 配套练习：[Code/sim_drone.py](../Code/sim_drone.py)（假无人机模拟器）。

## 1. 一句话原理

飞控上跑着 PX4，它**不断广播"遥测消息"**（姿态/位置/GPS/电量…），也
**接收"指令消息"**（切换模式/解锁/开始任务…）。这些消息的格式叫
**MAVLink 协议**。Python 用 **pymavlink** 库建一条通道，收消息、发指令。

```text
[飞控 PX4] ──MAVLink──▶ [数传/USB] ──UDP/串口──▶ [电脑 pymavlink] ──▶ [打印/文件/绘图]
```

## 2. 用了哪些库（分工）

| 库 | 负责什么 | 对应无人机/电脑的哪部分 |
|----|----------|--------------------------|
| **pymavlink** | MAVLink 消息编解码 + 通道 | "翻译官 + 电话线"，**唯一和无人机通信的库** |
| pyserial | 串口底层（pymavlink 内部调用） | USB 直连时的那根数据线 |
| matplotlib | 画曲线 | 把数据变成图表 |

核心只有 pymavlink。**网页（前端）由队友编写，你们只做后端**：把数据读进来、
整理好、输出成约定格式（JSON 文件），网页去消费这些文件。

## 3. pymavlink 的核心：mavutil

所有脚本开头都是：

```python
from pymavlink import mavutil
```

`mavutil`（MAVLink Utilities）提供：

- 连接：`mavutil.mavlink_connection(地址)`
- 收消息：`master.recv_match(...)`、`master.wait_heartbeat(...)`
- 最近状态缓存：`master.messages`
- 发指令：`master.mav.xxx_send(...)`

`master`（也叫 `conn`）就是一个**双向通道对象**，所有收发都通过它。

## 4. 函数逐讲解：这个函数对应无人机的什么

### 4.1 `mavutil.mavlink_connection(地址)` —— 建立物理链路

对应无人机：**把数传基站/USB 线接好**。

| 地址 | 对应连接 |
|------|----------|
| `udpin:0.0.0.0:14550` | 数传：电脑监听 14550，收飞控发来的数据 |
| `udpout:127.0.0.1:14550` | 模拟器/SITL：主动向对方发数据 |
| `serial:COM3,57600` | USB 直连（Windows） |

返回的 `master` 就是通道本身。

### 4.2 `master.wait_heartbeat(timeout)` —— 等飞控"报到"

对应无人机：**飞控上电后持续广播心跳**（约 1Hz），内容包含"我是谁（四旋翼/
PX4）、当前模式、是否解锁"。

```python
master.wait_heartbeat(timeout=10)
```

等不到 = 链路不通或飞控没开机。**所有脚本的第一步都是这个**。

### 4.3 `master.recv_match(type=..., blocking=...)` —— 收遥测消息

对应无人机：**飞控按频率广播遥测**（姿态几十 Hz、GPS 5Hz、电池 1Hz…）。

```python
msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
```

- `blocking=True`：阻塞等到一条；`False`：有就返回，没有返回 None
- `type`：只收指定消息（不写就是什么都要）
- 收到的消息会存进 `master.messages`（**最近一帧缓存**）

### 4.4 `master.messages` —— 飞控"仪表盘快照"

对应无人机：**一帧帧遥测的最新值**。

```python
master.messages.get('ATTITUDE')   # 最近一帧姿态，可能为 None
```

### 4.5 消息字段 → 无人机传感器对照表

| 消息 | 对应无人机部件/数据 | 字段 |
|------|---------------------|------|
| `ATTITUDE` | IMU 姿态 | roll/pitch/yaw（弧度） |
| `GLOBAL_POSITION_INT` | GPS+气压融合位置 | lat/lon（1e7）、relative_alt（mm） |
| `LOCAL_POSITION_NED` | 局部位置（NED 系） | x/y/z（m），offboard 用 |
| `GPS_RAW_INT` | GPS 模块 | fix_type、satellites_visible、eph |
| `SYS_STATUS` / `BATTERY_STATUS` | 电池 | voltage_battery（mV）、battery_remaining |
| `VFR_HUD` | HUD 仪表 | heading、groundspeed、throttle、alt、climb |
| `HEARTBEAT` | 飞控状态 | base_mode（含解锁位）、custom_mode |

### 4.6 `master.flightmode` —— 当前飞行模式

对应无人机：**遥控器 8 通道拨的档位**（定点/降落/任务…）。

### 4.7 发送指令：`master.mav.xxx_send(...)` —— 从电脑控制无人机

| 函数 | 对应无人机的什么 |
|------|------------------|
| `set_mode_send` | 相当于拨遥控器模式开关 |
| `command_long_send` | 相当于按下一个功能键（解锁/起飞/返航/开始任务） |
| `mission_count/item_int/...` | 相当于在 QGC 里规划并上传航线 |
| `request_data_stream_send` | 要求飞控"多说点数据"（调遥测频率） |

### 4.8 `set_mode_send` 详解（PX4 模式编码）

```python
master.mav.set_mode_send(
    target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   # base_mode
    custom_mode)                                          # 模式编码
```

PX4 的模式编码：`custom_mode = (主模式 << 16) | (子模式 << 24)`。
项目里的 `connect.set_px4_mode(master, main, sub)` 就是封装它。

| 模式 | main | sub | 对应遥控器/行为 |
|------|------|-----|------------------|
| AUTO + LOITER | 4 | 3 | 定点模式（8 通道上档） |
| AUTO + MISSION | 4 | 4 | 任务模式（8 通道下档） |
| AUTO + LAND | 4 | 6 | 降落（8 通道中档） |
| OFFBOARD | 6 | - | 机载电脑直接控制（进阶） |

### 4.9 `command_long_send` 详解（通用命令）

```python
master.mav.command_long_send(
    target_system, target_component, command, confirmation,
    param1, param2, param3, param4, param5, param6, param7)
```

| 命令常量 | 对应无人机的什么 |
|----------|------------------|
| `MAV_CMD_COMPONENT_ARM_DISARM` | 解锁/上锁（对应遥控器 5 通道） |
| `MAV_CMD_NAV_TAKEOFF` | 起飞到指定高度 |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 返航（对应遥控器 7 通道） |
| `MAV_CMD_MISSION_START` | 开始执行上传的任务 |

> 本项目的安全原则：解锁仍用遥控器，代码不自动解锁。

### 4.10 任务上传函数（mission_*）—— QGC 航线规划的程序版

对应无人机：**QGC 里点航点 → 上传 → 执行**。

| 函数/消息 | 协议里的角色 |
|-----------|--------------|
| `mission_count_send` | 告诉飞控"一共 N 个任务项" |
| `MISSION_REQUEST_INT` | 飞控反问"把第 seq 项给我" |
| `mission_item_int_send` | 我们发第 seq 项（命令+坐标+高度） |
| `MISSION_ACK` | 飞控回执（成功/拒绝） |
| `mission_clear_all_send` | 清空旧任务 |

协议是一问一答（见 mission.md 的流程图），不能一口气全发。

### 4.11 参数读写（进阶，先了解）

对应无人机：**QGC 参数页**（如 `SER_TEL1_BAUD`、`MAV_1_RATE`）。

```python
master.mav.param_request_read_send(target_system, target_component, b'参数名', -1)
```

本项目暂时不封装参数读写，用到时再展开。

## 5. 一张总对照表

| Python 代码 | 对应无人机的什么 |
|-------------|------------------|
| `mavlink_connection('udpin:...')` | 数传链路接通 |
| `wait_heartbeat()` | 飞控上电"报到" |
| `recv_match()` / `messages` | 遥测数据流 |
| `messages['ATTITUDE']` | IMU 姿态 |
| `messages['GPS_RAW_INT']` | GPS 模块 |
| `messages['SYS_STATUS']` | 电池 |
| `flightmode` | 当前飞行模式 |
| `set_mode_send(...)` | 拨遥控器模式开关 |
| `command_long_send(ARM_DISARM)` | 解锁/上锁 |
| `command_long_send(NAV_TAKEOFF)` | 起飞 |
| `command_long_send(MISSION_START)` | 开始任务 |
| `mission_*` 系列 | QGC 航线规划与上传 |
| `param_request_read_send` | QGC 参数页 |

## 6. 没有真机怎么练

```bash
# 终端1：启动假无人机（模拟飞控广播数据）
python3 Code/sim_drone.py

# 终端2：任选一个接收端
python3 Code/read_drone.py
python3 Code/view_data.py
```

练习清单：

1. 先跑 `read_drone.py`，对照第 4.5 节说出每条数据来自哪个传感器
2. 跑 `backend_exercise.py`，用测试数据练习输出 JSON（见
   [backend_exercise.md](../Code/backend_exercise.md)）
3. 改 `sim_drone.py` 里的姿态幅度，观察接收端变化

## 7. 常见错误与排查

| 报错/现象 | 原因 | 对应哪一环 |
|-----------|------|-----------|
| `wait_heartbeat` 超时 | 没启动模拟器/端口不对 | 4.2 链路 |
| 端口冲突 | QGC 8080 / pymavlink 14550 混用 | 4.1 地址 |
| 字段全是 0 | 模拟器没发该消息 | 4.3 收消息 |
| 方向感觉反了 | NED/ENU 没换算 | 4.5 LOCAL_POSITION_NED |
| 数值大得离谱 | 单位没换算（mm/mV/弧度） | 4.5 字段单位 |
| 发送无反应 | target_system 不对 | 4.7 发指令 |

## 8. 动手练习

1. 用一句话向别人解释：pymavlink 的 `wait_heartbeat` 对应无人机的什么
2. 在 `view_data.py` 里加一个字段，并说出它来自哪个传感器
3. 画一张从"飞控 → 数传 → 电脑 → 后端输出"的数据流图，标出每段用什么
