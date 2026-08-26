# sim_drone.py 教学文档（假无人机模拟器）

## 1. 这个文件是做什么的

真机到货前，用代码"假装"一架无人机：持续向地面站监听端口 14550 发送
MAVLink 遥测数据（心跳/姿态/位置/GPS/电量/速度），让 `read_drone.py`、
`view_data.py` 等接收端在没有硬件的情况下也能跑通。

## 2. 本文件用到的库：pymavlink（发送方向）

| 函数 | 模拟的真实数据 |
|------|----------------|
| `mavlink_connection('udpout:...')` | 扮演数传，主动把数据发到地面站端口 |
| `heartbeat_send` | 飞控心跳：类型、固件、模式、解锁位 |
| `attitude_send` | IMU 姿态（roll/pitch/yaw，弧度） |
| `global_position_int_send` | GPS 融合位置（经纬度/高度） |
| `local_position_ned_send` | 局部位置（NED） |
| `gps_raw_int_send` | GPS 原始数据（fix/星数/精度） |
| `sys_status_send` | 电池电压/剩余电量 |
| `vfr_hud_send` | HUD 仪表：航向/地速/油门/高度 |

> 与接收端（read_drone/view_data）形成对照：接收端用 `recv_match` 收，
> 这里用 `xxx_send` 发——同一个库的两半。

## 3. 原理：udpout 扮演无人机

```text
sim_drone.py（扮演飞控）──udpout──▶ 地面站 udpin:0.0.0.0:14550
```

pymavlink 的 `udpout:IP:端口` 是"主动向对方发数据"的连接，正好模拟
数传把飞控数据发到电脑；地面站脚本用 `udpin:0.0.0.0:14550` 监听接收。

## 4. 消息 → 真实无人机对照

| 发送函数 | 模拟的真实数据 |
|----------|----------------|
| `heartbeat_send` | 飞控心跳：类型、固件、模式、解锁位 |
| `attitude_send` | IMU 姿态（roll/pitch/yaw，弧度） |
| `global_position_int_send` | GPS 融合位置（经纬度/高度） |
| `local_position_ned_send` | 局部位置（NED，offboard 用） |
| `gps_raw_int_send` | GPS 原始数据（fix/星数/精度） |
| `sys_status_send` | 电池电压/剩余电量 |
| `vfr_hud_send` | HUD 仪表：航向/地速/油门/高度 |

数据不是静态的：姿态在摆动、位置在漂移、电量在下降，这样你能在接收端
看到数据在变，验证链路真的通了。

## 5. 运行

```bash
# 终端1：启动假无人机
python3 sim_drone.py

# 终端2：读取数据（任选）
python3 read_drone.py
python3 view_data.py
```

## 6. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 地面站收不到 | 模拟器没启动 / 端口不一致 | 两个终端的端口都要是 14550 |
| 数据不动 | 只开了接收没开模拟器 | 先启动 sim_drone.py |
| 电量曲线下降太快 | 模拟参数 | 改 `t / 60` 的系数 |

## 7. 练习

1. 启动模拟器 + `view_data.py`，观察数据在动
2. 改 `sim_drone.py` 里的 yaw 初始值（0.8），接收端航向跟着变
3. 把 `satellites_visible` 改成 5，观察 GPS 显示

> 离线不想开模拟器时，直接用 `read_drone.py --tlog test_data/flight_sample.tlog`
> 等命令读日志，效果类似（数据是录好的）。
