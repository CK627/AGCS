# 测试数据说明

给后端写代码训练用的**测试文本数据**（模拟一次 60 秒定点飞行，120 行 @0.5s）。

| 文件 | 内容 | 用途 |
|------|------|------|
| `telemetry_sample.csv` | 一行一帧的原始数据 | 练习解析（训练用） |
| `history_sample.jsonl` | 每行一条 JSON | 后端输出的参考格式 |
| `telemetry_flight.csv` | 完整飞行过程（起飞→定点→任务→返航→降落，300 行 @1s） | 练习按模式分段、统计 |
| `mission_sample.json` | 航线任务（起飞+4 航点+返航） | 练习任务解析，对照 mission.py |
| `flight_sample.tlog` | MAVLink 二进制日志（240 秒悬停） | `plot_data.py --tlog` 回放练习 |

## 数据文件怎么用

```bash
# CSV / JSONL：用 backend_exercise.py 的思路解析
python3 backend_exercise.py

# tlog：直接回放（不需要连接无人机）
python3 plot_data.py --tlog test_data/flight_sample.tlog
```

`telemetry_flight.csv` 的 `mode` 字段包含 5 个阶段：

| 时间 | 模式 | 行为 |
|------|------|------|
| 0~30s | TAKEOFF | 起飞，高度 0→8m |
| 30~120s | LOITER | 定点悬停，高度约 8m |
| 120~240s | MISSION | 沿 4 个航点飞行，高度约 10m |
| 240~270s | RTL | 返航 |
| 270~300s | LAND | 降落，最后上锁 |

`mission_sample.json` 与 `mission.py` 的任务结构一致（起飞 → 航点 → 返航），
可以对照学习任务上传协议。

## 字段说明

| 字段 | 含义 | 单位/格式 |
|------|------|-----------|
| `time` | 飞行时间 | 秒 |
| `mode` | 飞行模式 | LOITER（定点） |
| `armed` | 是否解锁 | true/false |
| `roll_deg` / `pitch_deg` / `yaw_deg` | 姿态 | 度 |
| `lat` / `lon` | 经纬度 | 度（7 位小数） |
| `alt_m` | 相对起飞点高度 | 米 |
| `sats` | GPS 卫星数 | 颗 |
| `volt` | 电池电压 | 伏 |
| `battery_pct` | 剩余电量 | % |
| `heading_deg` | 航向 | 度 |
| `groundspeed` | 地速 | m/s |
| `climb` | 垂直速度 | m/s |

> 数据是模拟器风格生成的（姿态摆动、位置漂移、电量下降），
> 和真机/`sim_drone.py` 的数据结构一致，方便练完直接对接真数据。
