# plot_data.py 教学文档（matplotlib + 日志回放）

## 1. 文件作用

把数据画成曲线：**实时采集**（`--live`）或**回放 tlog 日志**（`--tlog`）。
三张图：姿态（roll/pitch/yaw）、高度、电池电压。

## 2. 本文件用到的库

| 库/模块 | 函数/对象 | 作用 |
|---------|-----------|------|
| pymavlink | `mavlink_connection(tlog路径)` | 读日志（离线） |
| pymavlink | `recv_match(blocking=False)` | 非阻塞读完整个日志 |
| matplotlib | `plt.subplots` / `plot` / `show` | 画图 |
| collections | `deque(maxlen=N)` | 固定长度滚动缓存 |
| 标准库 | `argparse` / `math` / `time` | 参数 / 换算 / 计时 |

## 3. 两种模式

| 模式 | 数据来源 | 场景 |
|------|----------|------|
| `--live` | 实时连接飞控 | 悬停时观察姿态稳定性 |
| `--tlog` | 日志文件 | **离线练习/飞行后复盘** |

## 4. 关键概念：deque 滚动缓存

```python
ts, roll, pitch, yaw, alt, bat = [deque(maxlen=maxlen) for _ in range(6)]
```

- `deque(maxlen=N)`：满 N 自动丢最老的，只保留最近 N 个点
- `maxlen = 秒数 × 采样率`（如 60s × 2Hz = 120）

## 5. 运行与运行示例

### 5.1 离线运行（数据从 test_data 读取，推荐先做）

```bash
python3 plot_data.py --tlog test_data/flight_sample.tlog
```

预期：弹出窗口显示三张曲线——姿态（roll/pitch/yaw 三条线摆动）、
高度（约 5m 平线）、电池电压（缓慢下降）。无硬件、无需模拟器。

### 5.2 实时运行

```bash
# 终端1
python3 sim_drone.py
# 终端2
python3 plot_data.py --live --seconds 60
```

## 6. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 报 `TclError` / 无窗口 | 图形后端问题 | 换 `matplotlib.use('MacOSX')` 或 `'Agg'` |
| tlog 没有曲线 | 日志里没 ATTITUDE | 用我们生成的 test_data 日志 |
| 曲线是平的 | 只收到一种消息/数据没变 | 摇动无人机；检查消息过滤 |
| 数据点太少 | 采样率/时长不够 | 调 `--seconds` 或 `SAMPLE_RATE` |

## 7. 练习

1. 把电池曲线换成 GPS 星数（`GPS_RAW_INT.satellites_visible`）
2. 回放 tlog，指出姿态摆动幅度和真实悬停的区别
3. 改 `--tlog` 支持 CSV（提示：用 `csv.DictReader` 读 `telemetry_flight.csv` 再画）
