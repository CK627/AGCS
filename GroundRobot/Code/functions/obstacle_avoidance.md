# obstacle_avoidance.py 教学文档

## 1. 这个文件是做什么的

让六足机器人**边前进边避障**：超声波传感器测前方距离，太近就后退、转向，
否则继续前进。这是"自主行走"的基础模块，也是整合程序
`autonomous_pick.py` 里走路部分的地基。

## 2. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `time` | Python 标准库 | 循环节奏、延时 |
| `common.ros_robot_controller_sdk.Board` | 官方 SDK | 与扩展板串口通信（`/dev/ttyAMA0`） |
| `common.kinematics` | 官方 SDK（编译库 `.so`） | 六足运动学：`go_forward` / `turn_left` / `back` |
| `sensor.ultrasonic_sensor.Ultrasonic` | 官方 SDK | I2C 超声波（地址 `0x77`），`getDistance()` 返回毫米 |
| `functions.robot_config` | 本项目 | 读取避障参数 |

## 3. 关键概念：超声波怎么工作

超声波模块发出声波，测回波时间，算出前方障碍距离。`getDistance()` 返回**毫米**，
代码里统一转成厘米（`/ 10.0`），因为 `robot_params.yaml` 里的阈值用的也是厘米。

超声波有一个特性：**单次读数不可靠**（可能打到斜面、被环境反射干扰）。
所以代码做了滑动窗口滤波：

```python
def distance_cm(self):
    d = self.ultrasonic.getDistance() / 10.0
    self._dist_window.append(d)
    n = self.obs['filter_window']          # 默认 5
    if len(self._dist_window) > n:
        self._dist_window.pop(0)           # 挤掉最老的一个
    return sum(self._dist_window) / float(len(self._dist_window))
```

保留最近 5 次读数的平均值，比单次读数稳定得多。**如果测距忽大忽小，先检查
这一段的窗口大小和传感器朝向。**

## 4. 关键概念：六足运动学 API

`kinematics.IK(board)` 是官方加密的运动学库，用法：

```python
ik.stand(ik.initial_pos, t=500)                 # 立正，t 单位毫秒
ik.go_forward(ik.initial_pos, 2, 60, 50, 1)     # 前进 60mm
ik.turn_left(ik.initial_pos, 2, 15, 50, 1)      # 左转 15 度
ik.back(ik.initial_pos, 2, 80, 50, 1)           # 后退 80mm
```

参数对照：

| 参数 | 含义 |
|------|------|
| `ik.initial_pos` | 立正姿态（6 条腿末端坐标数组），由 SDK 提供 |
| `2` | 六足模式（`4` 是四足模式） |
| 第三个参数 | 直行是步幅（mm），**转向是角度（度）** |
| 第四个参数 | 速度（mm/s 或 deg/s） |
| 第五个参数 | 执行次数，`0` 表示无限循环 |

这些调用是**阻塞的**：执行完一步才返回。所以 `walk_forward` 是"走一步、停一下"
的节奏，而不是连续运动。

## 5. 决策逻辑讲解

```python
def walk_forward(self, stride=None, speed=None):
    if self.blocked():                 # 前方太近
        self.ik.back(..., 80, 50, 1)   # 先后退
        self.turn(left=True, angle=45) # 再左转 45 度
        return False
    self.ik.go_forward(..., stride, speed, 1)
    return True
```

`blocked()` 判断：`0 < 距离 < threshold`。`0 <` 是防止超声波读数为 0（未检测到
回波）时误判为"有障碍"。

`walk_forward` 返回 `True/False` 是**给上层用的信号**：调用方可以根据是否真的
前进了，决定下一步逻辑（比如目标丢失时是否重新扫描）。

> 新手提示：`self.walk['turn_angle']`、`self.obs['threshold']` 都是嵌套字典取值
> （`self.walk` 就是 `params['walk']`），不懂先读
> [robot_config.md 第 2 节「新手必读」](robot_config.md)。

## 6. 常见问题排查

| 现象 | 问题出在哪 | 排查方法 |
|------|-----------|----------|
| 机器人完全不动 | `Board()` 串口没打开 / 舵机没上电 | 先跑官方 `kinematics_control_demo.py` 验证运动学可用 |
| 一直后退转圈 | 超声波读数异常，`blocked()` 恒为真 | 打印 `distance_cm()` 原始值；检查传感器是否被遮挡/接反 |
| 距离显示忽大忽小 | 滤波窗口太小或传感器朝向偏 | 增大 `filter_window`；固定传感器 |
| 前进方向偏斜 | 六足步态本身或地面不平 | 这是机械特性，先接受，后续靠视觉纠偏 |
| 走得太快刹不住 | `stride`/`speed` 太大 | 调小 `walk.stride`、`walk.speed` |

## 7. 动手练习

1. 在障碍物前 20cm/40cm 各放一个目标，观察 `blocked()` 阈值切换是否正确
2. 把滤波窗口改成 10，比较距离读数稳定性
3. 给 `ObstacleAvoidance` 加一个 `go_back(steps)` 方法，用于整合程序后退
