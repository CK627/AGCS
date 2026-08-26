# view_data.py 教学文档（消息读取与单位换算）

## 1. 文件作用

实时（或离线日志）打印无人机数据：模式、解锁、姿态、位置、GPS、电池、
航向/地速。是 `read_drone.py` 的完整版，只读安全。

## 2. 本文件用到的库

| 库/模块 | 函数/对象 | 作用 |
|---------|-----------|------|
| pymavlink | `master.messages` 字典 | 按类型取最近一帧 |
| pymavlink | `recv_match()` | 收一帧，更新缓存 |
| 标准库 | `math.degrees()` | 弧度→度 |
| 标准库 | `argparse` / `time` | 参数解析 / 节奏控制 |
| connect.py | `connect()` / `is_armed()` | 在线连接 / 解锁判断 |

## 3. 核心：master.messages 缓存

```python
master.recv_match(blocking=True, timeout=2)   # 收一帧（任意类型）
master.messages.get('ATTITUDE')               # 取最近一帧姿态（可能 None）
```

- 收到的消息按类型存进字典，覆盖旧帧 = 永远是最新值
- `.get()` 可能返回 None（还没收到过），必须判空

## 4. 单位换算表（最容易错）

| 字段 | 原始单位 | 显示换算 |
|------|----------|----------|
| roll/pitch/yaw | 弧度 | `math.degrees()` → 度 |
| lat/lon | 1e7 度 | ÷1e7 |
| relative_alt | 毫米 | ÷1000 → 米 |
| voltage_battery | 毫伏 | ÷1000 → 伏 |
| battery_remaining | 百分比 | 直接显示 |
| eph | 厘米 | 直接显示 |

> 数值"离谱"（如高度 100000）先查单位，不是飞控坏了。

## 5. 运行与运行示例

### 5.1 离线运行（数据从 test_data 读取，推荐先做）

```bash
python3 view_data.py --tlog test_data/flight_sample.tlog
```

预期输出（节选）：

```
离线读取: test_data/flight_sample.tlog
模式=LOITER | 解锁=否 | 姿态r/p/y=0.0/2.3/45.8° | 位置31.2304,121.4737 高度5.0m | GPS fix=3 星=15 eph=100cm | 电池16.00V 100% | 航向46° 地速1.2m/s 爬升0.1m/s
...
日志读取完毕
```

### 5.2 在线运行

```bash
# 终端1
python3 sim_drone.py
# 终端2
python3 view_data.py
```

## 6. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 一直打印"(等待数据...)" | 没收到消息 | 数传连通/日志文件存在 |
| 离线读完日志程序不结束 | `blocking=True` 在 EOF 不返回 None | 离线用 `blocking=False`（代码已处理） |
| 经纬度全是 0 | GPS 未定位 | 室外等 fix≥3 |
| 电量显示 - | 还没收到电量消息 | 等几秒；消息名拼写检查 |

## 7. 练习

1. 加上 `VFR_HUD` 的油门显示（`throttle`）
2. 把数据追加写进 CSV（`csv` 模块），为日志功能打基础
3. 离线跑 `telemetry_flight.csv` 的等价 tlog，观察模式变化（提示：换一个含多阶段的日志）
