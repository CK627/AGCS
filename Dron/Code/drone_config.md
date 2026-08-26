# drone_config.py 教学文档

## 1. 这个文件是做什么的

所有无人机脚本共用的**连接配置**。改这个文件，所有脚本的连接方式一起变，
不用每个文件都改一遍——这就是"配置与代码分离"。

## 2. 关键概念：连接字符串

pymavlink 用一个字符串描述"连到哪、怎么连"：

| 连接字符串 | 含义 |
|------------|------|
| `udpin:0.0.0.0:14550` | 监听本机 14550 端口，接收数传发来的 UDP 数据 |
| `serial:COM3,57600` | Windows 串口 + 波特率 |
| `serial:/dev/ttyUSB0,57600` | Linux 串口 + 波特率 |
| `udpout:127.0.0.1:14550` | 主动向某地址发送（SITL 仿真用） |

`udpin:0.0.0.0:14550` 里的 `0.0.0.0` 表示"监听所有网卡"，这样数传基站发到
这台电脑 14550 端口的数据都能收到。

## 3. 为什么 QGC 用 8080、pymavlink 用 14550

**同一端口同一时间只能被一个程序占用**。QGC 已经监听 8080，所以 pymavlink
用 MAVLink 默认地面站端口 **14550**，互不冲突。

前提：数传基站端要把**地面站 IP + 14550** 加为额外发送目标（或直接改数传
目标端口），否则数据不会发到 14550。

## 4. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 连接超时 | 数传没开机 / 无人机没通电 / 14550 没加为发送目标 | 先 QGC 确认通路，再在数传端加 14550 |
| 与 QGC 抢端口 | 两个程序监听同一 UDP | QGC 用 8080、pymavlink 用 14550 |
| 串口打不开 | 权限不足 / COM 号不对 | Windows 查设备管理器；Linux 加 `dialout` 组 |
| SITL 连不上 | 仿真没启动 | 先启动 SITL 再跑脚本 |

## 5. 运行与运行示例

本文件不直接运行，它被其他脚本 import：

```python
from drone_config import CONNECTION, TIMEOUT, TARGET_SYSTEM, TARGET_COMPONENT
```

快速查看当前配置：

```bash
python3 -c "from drone_config import CONNECTION, TIMEOUT; print(CONNECTION, TIMEOUT)"
```

预期输出：

```
udpin:0.0.0.0:14550 10
```

离线练习时不需要改它：`read_drone.py --tlog` / `view_data.py --tlog` /
`plot_data.py --tlog` 直接读 `test_data/` 里的日志，连接配置不参与。

## 6. 练习

1. 分别改成 UDP、USB 两种方式，跑 `read_drone.py` 对比
2. 解释为什么 `TARGET_SYSTEM=1` 和 `TARGET_COMPONENT=1` 默认不用改
