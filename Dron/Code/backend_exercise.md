# backend_exercise.py 教学文档（后端文件训练）

## 1. 分工：你负责后端，网页别人写

```text
无人机/模拟器 ──▶ 你写的后端（读数据→整理→输出 JSON 文件）
                                                    │
                                                    ▼
                               队友写的网页（前端）读这些文件展示
```

你只需要让"后端"把数据**读进来、整理好、输出成约定格式**，
不需要学网页（flask/HTML/JS 都不碰）。

## 2. 输出格式约定（接口契约）

后端输出两个文件：

1. **`telemetry.json`**：最新一帧数据，网页用来刷新"当前状态"
2. **`history.jsonl`**：历史数据，每行一条 JSON，网页用来画曲线

参考格式见 `test_data/history_sample.jsonl`：

```json
{"time": 0.0, "mode": "LOITER", "armed": false, "roll_deg": 0.0, ...}
{"time": 0.5, "mode": "LOITER", "armed": false, "roll_deg": 1.96, ...}
```

字段名要固定，网页按字段名取值，**不能今天叫 `roll` 明天叫 `roll_deg`**。

## 3. 测试数据

`test_data/telemetry_sample.csv`：120 行模拟飞行数据（60 秒 @0.5s），
字段说明见 [test_data/README.md](test_data/README.md)。

## 4. 训练任务（按 TODO 顺序做）

### TODO 1：读 CSV

用 `csv.DictReader` 把文件读成字典列表。注意：读出来全是字符串，
计算前要转数字。

自查：`print(rows[0])` 能看到第一行，`len(rows) == 120`。

### TODO 2：最新一帧

`rows[-1]` 就是最新一行，直接作为 telemetry.json 内容。

### TODO 3：JSON Lines

每行 `json.dumps(一行)`，行尾 `\n`。和 `history_sample.jsonl` 对比，
内容应一致（字段顺序无所谓）。

## 5. 进阶：把后端接到真数据

练完 CSV 后，把"读 CSV"换成"读无人机"：

```python
# 思路（参考 read_drone.py）：
from connect import connect
master = connect()
while True:
    master.recv_match(blocking=True, timeout=1)
    # 把 master.messages 里的 ATTITUDE / GLOBAL_POSITION_INT / SYS_STATUS
    # 整理成同样的字段，每 0.5 秒 append 一行 jsonl，并刷新 telemetry.json
```

真机没到先用 `sim_drone.py` 当数据源。

## 6. 运行与运行示例（数据从 test_data 读取）

```bash
# 默认：读 telemetry_sample.csv（120 行定点悬停）
python3 backend_exercise.py

# 换一份：读 telemetry_flight.csv（起飞→定点→任务→返航→降落）
python3 backend_exercise.py --csv test_data/telemetry_flight.csv
```

预期输出（补完 TODO 后）：

```
读取到 120 行
已生成 telemetry.json 和 history.jsonl
检查：打开 test_data/history_sample.jsonl，与你的输出对比
```

用 `--csv test_data/telemetry_flight.csv` 时，history.jsonl 会包含 5 种模式
（TAKEOFF/LOITER/MISSION/RTL/LAND），可以练习按 mode 分组统计。

## 7. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `FileNotFoundError` | 路径不对 | 脚本和 `test_data/` 必须在同一目录 |
| 数值是字符串 | 没转类型 | `float(row['volt'])` |
| jsonl 全挤在一行 | 没加 `\n` | 每行末尾加换行 |
| 中文乱码 | 编码问题 | 读写都加 `encoding='utf-8'` |
| 与参考文件对不上 | 字段名/顺序不同 | 对照 `history_sample.jsonl` 检查 |

## 8. 练习

1. 完成三个 TODO，让输出和 `history_sample.jsonl` 一致
2. 加一个 `max_alt()` 函数：返回历史最高高度（`alt_m` 最大值）
3. 进阶：把数据源换成 `sim_drone.py`，写一个不停更新 `telemetry.json` 的版本
4. 进阶：用 `test_data/telemetry_flight.csv`（多阶段飞行数据），统计每个
   `mode` 各持续多少秒、任务阶段飞了多远（提示：按 mode 分组遍历）
5. 进阶：用 `test_data/flight_sample.tlog` 练习 `plot_data.py --tlog` 回放，
   观察姿态/高度曲线
