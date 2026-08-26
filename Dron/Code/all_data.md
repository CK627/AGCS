# all_data.py 教学文档（总和代码：一次读全所有参数）

## 1. 这个文件是干嘛的

把无人机能给的"小纸条"一次性读全，一行打印所有参数。它是前面几个小脚本的
"大合集"，也是你以后写后端的基础。

对比：

- `read_drone.py`：只读 5 类，最简
- `all_data.py`：读全所有参数（状态、姿态、位置、GPS、电池、飞行、局部位置）

## 2. 用比喻看代码结构

整个文件就是三步：

```text
取信箱（连接/读日志） → 每收一张姿态纸条就查公告栏 → 把公告栏翻译成一行字
```

### 2.1 公告栏（master.messages）

无人机不断往"公告栏"贴纸条，同类纸条只保留最新一张：

```python
m = master.messages
att = m.get('ATTITUDE')   # 从公告栏拿"姿态"纸条，可能还没有
```

`.get()` 可能返回 `None`（还没有这类纸条），所以每个都要判空。

### 2.2 单位换算 = 汇率

| 纸条上的原始值 | 人想看的 | 换算 |
|----------------|----------|------|
| roll（弧度） | 度 | `math.degrees()` |
| lat（1e7 度） | 度 | `÷ 1e7` |
| relative_alt（毫米） | 米 | `÷ 1000` |
| voltage_battery（毫伏） | 伏 | `÷ 1000` |

### 2.3 位置纸条的两个坐标方向

`LOCAL_POSITION_NED` 是"北-东-下"；我们习惯"东-北-上"，所以要翻转：

```python
东 = lpn.y
北 = lpn.x
上 = -lpn.z   # 因为它是"下"
```

## 3. 运行与运行示例（数据从 test_data 读取）

```bash
python3 all_data.py --tlog test_data/flight_sample.tlog
```

预期输出（节选）：

```text
离线读取: test_data/flight_sample.tlog
状态=模式:LOITER 解锁:False | 姿态=roll 0.0° pitch 2.3° yaw 45.8° | 位置=lat 31.2304000 lon 121.4737100 高度 5.00m | GPS=fix 3 星数 15 精度 100cm | 电池=16.00V 剩余 100% | 飞行=航向 45° 地速 1.2m/s 爬升 0.1m/s 油门 35% | 局部位置=东 1.00m 北 0.00m 上 5.00m
日志读完
```

在线运行：

```bash
# 终端1
python3 sim_drone.py
# 终端2
python3 all_data.py
```

## 4. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 一直显示"(还没有收到纸条)" | 日志里没有 ATTITUDE | 换 test_data 里的日志 |
| 数值很大/很小 | 单位没换算 | 对照 2.2 换算表 |
| 方向感觉反了 | NED/ENU 没翻转 | 用 `(y, x, -z)` |
| 离线读完卡住 | blocking=True 在 EOF 不返回 | 离线用 blocking=False（已处理） |

## 5. 练习

1. 离线跑通后，指出哪一段对应"姿态"、哪一段对应"电池"
2. 给输出加一个时间戳（提示：用 `time.time()`）
3. 把 `snapshot()` 改成返回字典，为下一步输出 JSON 做准备
