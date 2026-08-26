# read_drone.py 教学文档（pymavlink 库学习）

## 1. 文件作用

**最小** MAVLink 读取脚本：只打印五类数据（位置 ENU、GPS、偏航、模式/解锁、
电量）。这是无人机代码的"第一个程序"，先跑通它再碰其他脚本。

## 2. 本文件用到的库

| 库/模块 | 函数 | 作用 |
|---------|------|------|
| pymavlink | `mavlink_connection()` | 建通道（在线 UDP / 离线 tlog） |
| pymavlink | `recv_match(type, blocking, timeout)` | 收消息并按类型分发 |
| pymavlink | `mode_string_v10(msg)` | 从心跳解析飞行模式 |
| pymavlink | `MAV_MODE_FLAG_SAFETY_ARMED` | 判断解锁的位标志 |
| 标准库 | `argparse` | 命令行参数（`--tlog`） |

## 3. 逐段讲解

### 3.1 连接（在线/离线二选一）

```python
if args.tlog:
    conn = mavutil.mavlink_connection(args.tlog)   # 读日志
else:
    conn = mavutil.mavlink_connection(CONNECTION)  # 在线
    conn.wait_heartbeat(timeout=15)
```

同一个 `recv_match` 循环，在线收实时数据、离线回放日志，**代码几乎不用改**。

### 3.2 主循环：按消息类型分发

```python
msg = conn.recv_match(blocking=True, timeout=1.0)
mtype = msg.get_type()
if mtype == 'ATTITUDE': ...
elif mtype == 'HEARTBEAT': ...
```

### 3.3 单位换算（重点）

| 消息 | 字段 | 换算 |
|------|------|------|
| `LOCAL_POSITION_NED` | x/y/z | NED→ENU：`(y, x, -z)` |
| `GLOBAL_POSITION_INT` | lat/lon/alt | ÷1e7；alt÷1000→m |
| `ATTITUDE` | yaw | ×57.2958→度 |
| `SYS_STATUS` | voltage_battery | ÷1000→V |

## 4. 运行与运行示例

### 4.1 离线运行（数据从 test_data 读取，推荐先做）

```bash
python3 read_drone.py --tlog test_data/flight_sample.tlog
```

预期输出（节选）：

```
离线读取: test_data/flight_sample.tlog
yaw=47.3deg
GPS: lat=31.2304006 lon=121.4737099 alt=10.00m
position ENU(m): x=1.00 y=0.06 z=5.00
battery: 16.00V 99%
mode=LOITER armed=False
...
日志读取完毕
```

### 4.2 在线运行（配合模拟器/真机）

```bash
# 终端1
python3 sim_drone.py
# 终端2
python3 read_drone.py
```

## 5. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 离线读 tlog 报 BAD_DATA | 日志格式不对 | 用我们生成的 test_data 日志；格式=8 字节时间戳+消息 |
| 离线读到末尾程序不退出 | `recv_match(blocking=True)` 在日志 EOF 不会返回 None | 离线模式用 `blocking=False`（代码已处理） |
| 在线等心跳超时 | 14550 没通 | 数传端加地面站 IP:14550 |
| 位置方向反 | NED/ENU 没换算 | 用 `(y, x, -z)` |

## 6. 练习

1. 离线跑通后，对照 test_data 日志说出每条数据来自哪个传感器
2. 加一个 `VFR_HUD` 分支打印航向和地速
3. 改成每 0.5 秒只打印一次（提示：记录上次打印时间）
