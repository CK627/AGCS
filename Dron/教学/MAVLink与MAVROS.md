# MAVLink 与 pymavlink 教学文档（Windows 地面站版）

> 旧版本文档讲 MAVROS（ROS 专属）；地面站改为 Windows 后，用官方 Python 库
> **pymavlink** 直接读写 MAVLink，功能等价且不依赖 ROS。MAVLink 协议本身不变。

## 1. MAVLink：无人机世界的"通用语言"

MAVLink 是飞控与地面站/机载电脑之间的**消息协议**：姿态、位置、电池、
遥控指令等都封装成一条条"消息"在通信链路上传输。

```text
飞控 ──MAVLink──▶ QGC / pymavlink / 自写程序
```

本机 QGC 通过 UDP 8080 收 MAVLink 消息；用 USB 直连时是串口 MAVLink
（波特率 57600）。

## 2. pymavlink：让 Python 听懂 MAVLink

pymavlink 是 MAVLink 官方的 Python 库，**跨平台**（Windows/Linux/macOS 通用）。

| 对比 | MAVROS（旧方案） | pymavlink（现方案） |
|------|------------------|----------------------|
| 依赖 | ROS Noetic（仅 Ubuntu） | Python 3 + pip，Windows 直接装 |
| 数据来源 | ROS Topic（/mavros/...） | MAVLink 消息直接读 |
| 安装 | apt 装 ros-noetic-mavros | `python -m pip install pymavlink` |
| 启动 | roscore + roslaunch mavros | 一行代码建连接 |

安装：

```bat
python -m pip install pymavlink
```

## 3. 本机连接方式

| 连接方式 | pymavlink 连接串 | 说明 |
|----------|------------------|------|
| 数传 UDP（推荐） | `udpin:0.0.0.0:14550` | 地面站监听 14550 收 MAVLink（QGC 占用 8080） |
| SITL 仿真 | `udpout:127.0.0.1:14550` | 本地仿真调试用 |
| USB 串口 | `serial:/dev/ttyUSB0,57600` | Windows 下是 `serial:COM3,57600`（校准用） |

> QGC 已经监听 UDP 8080，pymavlink 就用 **14550**（MAVLink 默认地面站端口）。
> 数传端需要把地面站 IP + 14550 加为额外发送目标，或直接改数传目标端口。

最小读取脚本 `read_drone.py`：

```python
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print('等待飞控心跳...')
conn.wait_heartbeat(timeout=15)
print('已连接，目标系统 %d' % conn.target_system)

while True:
    msg = conn.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    mtype = msg.get_type()
    if mtype == 'LOCAL_POSITION_NED':
        # NED（北-东-下）→ ENU（东-北-上）
        print('position ENU(m): x=%.2f y=%.2f z=%.2f' % (msg.y, msg.x, -msg.z))
    elif mtype == 'GLOBAL_POSITION_INT':
        print('GPS: lat=%.7f lon=%.7f alt=%.2fm' %
              (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0))
    elif mtype == 'ATTITUDE':
        print('yaw=%.1fdeg' % (msg.yaw * 57.2958))
    elif mtype == 'HEARTBEAT':
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print('mode=%s armed=%s' % (mavutil.mode_string_v10(msg), armed))
    elif mtype == 'SYS_STATUS':
        print('battery: %.2fV %d%%' % (msg.voltage_battery / 1000.0,
                                       msg.battery_remaining))
```

## 4. 常用 MAVLink 消息与旧 ROS Topic 对照

| 数据 | MAVLink 消息 | 旧 ROS Topic（/mavros/...） | pymavlink 字段 |
|------|--------------|-----------------------------|----------------|
| 局部位置 | `LOCAL_POSITION_NED` | `local_position/pose` | `x/y/z`（NED，单位 m） |
| GPS | `GLOBAL_POSITION_INT` | `global_position/global` | `lat/lon/alt`（1e7 / 1e3 缩放） |
| 姿态 | `ATTITUDE` | `imu/data` | `roll/pitch/yaw`（弧度） |
| 模式/解锁 | `HEARTBEAT` | `state` | `base_mode` / `custom_mode` |
| 电量 | `SYS_STATUS` | `battery` | `voltage_battery`(mV) / `battery_remaining`(%) |

**坐标系提醒**：飞控输出的是 **NED**（北-东-下），旧 ROS 习惯用 **ENU**
（东-北-上）。换算：`ENU.x = NED.y`、`ENU.y = NED.x`、`ENU.z = -NED.z`。
坐标计算（YOLO 文档 7.4）里用的就是 ENU。

## 5. 发送指令（进阶）

pymavlink 也能下发指令，替代 MAVROS 的 service：

```python
# 解锁/上锁（MAV_CMD_COMPONENT_ARM_DISARM：1=解锁，0=上锁）
conn.mav.command_long_send(conn.target_system, conn.target_component,
                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           0, 1, 0, 0, 0, 0, 0, 0)

# 切换飞行模式（PX4 模式名 → 模式码）
mode_id = mavutil.mode_mapping_apm['GUIDED']   # 以实际固件映射为准
conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
```

> 正式版由杨梦享统一封装成"指令服务"：地面站程序只发
> `LEFT / RIGHT / FORWARD / PICK` 这类高层指令，具体 MAVLink 命令在服务里实现
> （见 YOLO 文档 7.10）。给机器人下发坐标仍走 HTTP（7.3），不经过 MAVLink。

## 6. 常见问题排查

| 现象 | 可能原因 | 排查/解决 |
|------|----------|-----------|
| `wait_heartbeat` 超时 | 端口没开/数传目标没加 | 先确认 QGC 能连上；防火墙放行 UDP 14550；数传端添加地面站 IP+14550 |
| 与 QGC 抢端口 | 两个程序监听同一 UDP | QGC 用 8080，pymavlink 用 14550，不要重复 |
| USB 串口连不上 | 波特率/驱动 | Windows 设备管理器查 COM 口，波特率 57600 |
| 只有心跳、没有位置 | 消息频率没开 | 检查飞控参数 `MAV_1_RATE`；数传带宽不够时降频率 |
| 数据方向感觉反了 | NED vs ENU | 统一按第 4 节换算，先在地面画坐标验证 |

## 7. 动手练习

1. 运行 `read_drone.py`，确认能打印位置/GPS/电量；
2. 和 QGC 画面上的数值对照，验证 pymavlink 读到的字段一致；
3. 把 LOCAL_POSITION_NED 换算成 ENU，在地面站坐标系里标出无人机当前位置。
