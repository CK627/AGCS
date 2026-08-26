# connect.py 教学文档（pymavlink 库学习）

## 1. 文件作用

MAVLink 连接封装：任何脚本要连无人机，先 `from connect import connect`，
拿到的 `master` 对象就是"和飞控对话的通道"。本文件也是学习
**pymavlink 核心函数**的入口。

## 2. 本文件用到的库：pymavlink（完整学习）

### 2.1 mavutil：pymavlink 的工具模块

```python
from pymavlink import mavutil
```

`mavutil`（MAVLink Utilities）提供连接、收消息、发指令的全部工具。

### 2.2 `mavutil.mavlink_connection(地址)` —— 建立通道

| 地址 | 对应连接 |
|------|----------|
| `udpin:0.0.0.0:14550` | 数传：电脑监听 14550 收数据 |
| `udpout:127.0.0.1:14550` | 模拟器/仿真：主动发数据 |
| `serial:COM3,57600` | USB 直连（Windows） |
| `路径.tlog` | 读取日志文件（**离线练习用**） |

### 2.3 收消息三件套

```python
master.wait_heartbeat(timeout=10)          # 等飞控心跳（握手）
msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)  # 收指定消息
master.messages.get('ATTITUDE')            # 最近一帧缓存（可能 None）
```

### 2.4 常用常量

```python
mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED   # 128：解锁位
mavutil.mavlink.MAV_TYPE_QUADROTOR            # 2：四旋翼
mavutil.mavlink.MAV_AUTOPILOT_PX4             # 12：PX4 固件
mavutil.PX4_CUSTOM_MAIN_MODE_AUTO             # 4：自动主模式
mavutil.PX4_CUSTOM_SUB_MODE_AUTO_MISSION      # 4：任务子模式
```

### 2.5 发指令（进阶）

```python
master.mav.set_mode_send(target_system, base_mode, custom_mode)  # 切模式
master.mav.command_long_send(...)                                # 通用命令
```

## 3. 逐段讲解

### 3.1 connect()：建通道 + 等心跳

```python
master = mavutil.mavlink_connection(conn)
hb = master.wait_heartbeat(timeout=timeout)
```

- 先建"管道"（不保证通），再等心跳（真正的验证）
- 等不到就 `SystemExit` 报错退出

### 3.2 工具函数

- `is_armed(master)`：读心跳 `base_mode` 的第 8 位，判断解锁
- `px4_custom_mode(main, sub)`：PX4 模式编码 `(主模式<<16)|(子模式<<24)`
- `set_px4_mode(master, main, sub)`：封装 `set_mode_send`

## 4. 运行与运行示例

### 4.1 在线运行（配合模拟器/真机）

```bash
# 终端1
python3 sim_drone.py
# 终端2
python3 connect.py
```

预期输出：

```
[连接] 目标: udpin:0.0.0.0:14550
[连接] 等待飞控心跳（超时 10 秒）...
[连接] 成功：type=MAV_TYPE_QUADROTOR autopilot=MAV_AUTOPILOT_PX4 flightmode=LOITER
测试完成。飞行模式: LOITER，解锁状态: False
```

### 4.2 离线运行（数据从 test_data 读取）

`connect.py` 本身连接实时源；离线时直接用 pymavlink 读 tlog 日志练习同样的
库函数：

```bash
python3 - <<'EOF'
from pymavlink import mavutil
master = mavutil.mavlink_connection('test_data/flight_sample.tlog')
master.wait_heartbeat()
print('flightmode =', master.flightmode)
print('armed =', bool(master.messages['HEARTBEAT'].base_mode & 128))
EOF
```

预期输出：

```
flightmode = LOITER
armed = False
```

## 5. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 没有心跳 | 通路错/端口没加发送目标 | 看 drone_config.md 排查 |
| flightmode 显示 UNKNOWN | 心跳 base_mode 缺模式标志 | 真机正常；模拟器用 `auto_mode_flags` |
| 发模式没反应 | target_system 不对 | 确认飞控 sysid（默认 1） |

## 6. 练习

1. 用离线示例读取 tlog，打印出 `master.flightmode`
2. 在 `connect.py` 加一个 `get_gps(master)` 返回卫星数和 fix_type
3. 解释 `is_armed` 为什么用 `& MAV_MODE_FLAG_SAFETY_ARMED`
