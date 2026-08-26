# mission.py 教学文档（任务上传协议 + 离线预览）

## 1. 文件作用

航线任务的**上传与启动**：把"起飞 → 航点 → 返航"打包成 MAVLink 任务传给飞控。
默认只上传不飞；`--preview` 可离线只打印任务；`--start` 且人工确认后才执行。

这是本套代码里**唯一会飞**的脚本，必须逐字看懂再动。

## 2. 本文件用到的库

| 库/模块 | 函数 | 作用 |
|---------|------|------|
| pymavlink | `mission_count_send` | 报任务数量 |
| pymavlink | `recv_match(type='MISSION_REQUEST_INT')` | 等飞控索要任务项 |
| pymavlink | `mission_item_int_send` | 发单个任务项（命令+坐标+高度） |
| pymavlink | `recv_match(type='MISSION_ACK')` | 收上传回执 |
| pymavlink | `mission_clear_all_send` | 清空旧任务 |
| pymavlink | `set_mode_send` / `command_long_send` | 切任务模式 / 发 MISSION_START |
| connect.py | `connect()` / `is_armed()` / `set_px4_mode()` | 连接 / 预检 / 切模式 |

## 3. 任务上传协议（一问一答）

```text
我们 mission_count(数量) ──────────────▶ 飞控
飞控 MISSION_REQUEST_INT(seq=0) ──────▶ 我们
我们 mission_item_int(第0项) ──────────▶ 飞控
...（循环到最后一个任务项）
飞控 MISSION_ACK(结果) ────────────────▶ 我们
```

为什么不能一口气全发？飞控要求一问一答，确保每个任务项按序号接收。

## 4. 任务项字段

| 字段 | 含义 |
|------|------|
| `command` | 动作：NAV_TAKEOFF(22) / NAV_WAYPOINT(16) / NAV_RETURN_TO_LAUNCH(20) |
| `x / y / z` | 经纬度（1e7）与高度（m） |
| `p1` | 航点悬停秒数 |
| `current` | 第一项=1，其余=0 |
| `autocontinue` | 1=继续下一个 |

## 5. 运行与运行示例

### 5.1 离线预览（数据从 test_data 读取，推荐先做）

```bash
python3 mission.py --waypoints "31.2305,121.4738,10;31.2306,121.4739,10" --preview
```

预期输出：

```
[任务] 预览（起飞前请人工核对）：
  #0 起飞: 原地起飞到 8.0m
  #1 航点: 31.2305000, 121.4738000 @ 10.0m
  #2 航点: 31.2306000, 121.4739000 @ 10.0m
  #3 返航: 返回起飞点
[任务] 预览模式结束（未连接飞控）
```

对照任务结构：`test_data/mission_sample.json`（起飞+4 航点+返航），
字段与 `mission.py` 的 `build_mission` 一致。

### 5.2 在线上传（配合模拟器/真机）

```bash
python3 mission.py --waypoints "31.2305,121.4738,10;31.2306,121.4739,10"
```

### 5.3 启动飞行（高危）

```bash
python3 mission.py --waypoints "..." --start
```

## 6. 安全设计

1. **不自动解锁**：解锁用遥控器 5 通道
2. **强制确认**：`--start` 后还要输入 `YES`
3. **只监控不干预**：飞行中只打印状态，出事用遥控器接管（6 急停 / 7 返航）

## 7. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| MISSION_REQUEST 超时 | 清空任务 ACK 被误读 | 上传前先读掉清空 ACK（代码已处理） |
| MISSION_ACK 非 ACCEPTED | 航点非法/数量不对 | 看结果名；检查坐标格式 |
| 上传成功但不起飞 | 没切任务模式/没解锁 | 确认 `--start`；遥控器解锁 |
| 飞错方向 | 航点是占位坐标 | 实飞必须 `--waypoints` 改实际场地 |

## 8. 练习

1. **不装桨**跑 `--preview`，核对任务预览
2. 对照 `mission_sample.json`，说出 6 个任务项分别是什么
3. 读代码找出 `current=1` 的作用（任务第一项标志）
