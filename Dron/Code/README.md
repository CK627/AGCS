# Dron Code —— 无人机 Python 学习（新手闯关版）

本目录是"无人机 Python 互通"的新手教材：**先读比喻，再按关卡闯关**。

入口顺序：

1. [黑话对照表.md](黑话对照表.md) —— 把专业词换成生活比喻
2. [闯关任务.md](闯关任务.md) —— 从简单到完整的 10 关任务
3. [无人机操控与自主飞行教学手册](../无人机操控与自主飞行教学手册.md) —— 需要理论时再看

所有示例都能离线跑，数据来自 [test_data/](test_data/README.md)。

## 1. 文件规划

| 文件 | 作用 | 安全级别 | 教学文档 |
|------|------|----------|----------|
| `drone_config.py` | 连接参数配置（改这里切连接方式） | - | [drone_config.md](drone_config.md) |
| `connect.py` | MAVLink 连接 + 工具函数 | 只读 | [connect.md](connect.md) |
| `read_drone.py` | 最小数据读取演示（对应教学文档示例） | 只读 | [read_drone.md](read_drone.md) |
| `all_data.py` | **总和代码：一次读全所有参数** | 只读 | [all_data.md](all_data.md) |
| `view_data.py` | 电脑上实时打印姿态/GPS/电量 | 只读 | [view_data.md](view_data.md) |
| `plot_data.py` | 实时曲线 / tlog 日志回放 | 只读 | [plot_data.md](plot_data.md) |
| `mission.py` | 航线任务上传 + 启动自主飞行 | **高危** | [mission.md](mission.md) |
| `sim_drone.py` | 假无人机模拟器（真机没到先用它练） | 模拟 | [sim_drone.md](sim_drone.md) |
| `backend_exercise.py` | 后端文件训练（测试数据 → JSON 输出） | 练习 | [backend_exercise.md](backend_exercise.md) |
| `test_data/` | 测试文本数据（写代码练习用） | - | [test_data/README.md](test_data/README.md) |

> 分工：**网页（前端）由队友编写**，无人机侧只负责后端——把数据读进来、
> 整理好、输出成约定格式（见 `backend_exercise.md` 的接口约定）。

## 1.1 黑话速查（新手先看）

MAVLink=快递单格式，pymavlink=翻译官，消息=小纸条，心跳=报平安，
recv_match=从信箱取信，messages=公告栏。完整对照见 [黑话对照表.md](黑话对照表.md)。

## 2. 安装依赖

```bash
python -m pip install -r requirements.txt
```

（pymavlink 负责 MAVLink 通信；matplotlib 负责绘图；pyserial 负责串口。）

## 2.1 连接方式速查

QGC 与 pymavlink 用**不同端口**，互不冲突：

| 工具 | UDP 端口 | 说明 |
|------|----------|------|
| QGC | 8080 | 官方地面站 |
| pymavlink（本项目） | 14550 | 需在数传端把"地面站 IP + 14550"加为发送目标 |

## 3. 运行顺序

**离线优先**：所有读取示例的数据都来自 `test_data/`，不需要无人机和模拟器。

```bash
python3 backend_exercise.py                                       # 0. 读 test_data CSV → JSON（训练）
python3 read_drone.py --tlog test_data/flight_sample.tlog         # 1. 读 tlog（离线）
python3 all_data.py --tlog test_data/flight_sample.tlog           # 2. 总和代码：读全参数（离线）
python3 view_data.py --tlog test_data/flight_sample.tlog          # 3. 完整数据打印（离线）
python3 plot_data.py --tlog test_data/flight_sample.tlog          # 4. 回放曲线（离线）
python3 mission.py --waypoints "31.2305,121.4738,10" --preview    # 5. 任务预览（离线）

# 在线（模拟器/真机）：
python3 sim_drone.py                        # 终端1：启动假无人机
python3 all_data.py / read_drone.py / view_data.py / plot_data.py --live   # 终端2：实时
python3 mission.py --waypoints "..."        # 只上传任务（不飞）
python3 mission.py --waypoints "..." --start # 预检通过后起飞（高危！）
```

> 真机到货后：关掉 `sim_drone.py`，把 `drone_config.py` 的连接目标改成
> 数传/串口即可，接收端代码一行不用改。

## 4. 安全红线（mission.py 相关）

1. 默认**只上传任务不启动**；加 `--start` 且有 `YES` 确认才执行
2. 代码**不自动解锁**，解锁用遥控器 5 通道
3. 起飞前必须：GPS fix≥3 且星数≥10、电量充足、场地开阔、遥控器在手
4. `mission.py` 里的示例航点是**占位坐标**，实飞前必须改成实际场地

## 5. 与 README 目标对应

- 阶段 1/2（互通、数据查看）→ 对应 README「MAVROS 通信」的前置练习
- 阶段 3（任务自主飞行）→ 对应 README「无人机自主飞行：按预设航线完成巡检」
