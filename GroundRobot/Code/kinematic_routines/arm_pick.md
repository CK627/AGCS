# arm_pick.py 教学文档

## 1. 这个文件是做什么的

机械臂抓取封装。把"移动到目标点 → 下降 → 夹取 → 抬起 → 松开"这一整套动作
打包成几个方法，`autonomous_pick.py` 只需要调用：

```python
ok = picker.pick_at(wx, wy)   # wx, wy 是机械臂坐标系下的厘米坐标
```

它是官方 `block_fetch.py` 抓取逻辑的**重构版**：功能相同，但拆成可复用的方法，
并允许把参数（抓取高度、夹爪开合脉宽）放在配置文件里。

## 2. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `time` | Python 标准库 | 动作间隔等待 |
| `functions.robot_config` | 本项目 | 读取 `arm.*` 参数 |
| `arm_ik.arm_move_ik.ArmIK` | 官方 SDK | 机械臂逆运动学 |
| `common.ros_robot_controller_sdk.Board` | 官方 SDK | 控制总线舵机（含夹爪 25 号） |

## 3. 关键概念：正运动学 vs 逆运动学

- **正运动学**：已知 5 个关节角度，求末端（夹爪）在哪
- **逆运动学（本代码用的）**：已知夹爪要到的坐标 `(x, y, z)`，反推出每个
  关节该转多少度

```python
def move_to(self, x, y, z, movetime=1.0):
    return self.ak.setPitchRangeMoving(
        (x, y, z), self.p['pitch'], self.p['alpha1'], self.p['alpha2'], movetime)
```

`setPitchRangeMoving` 参数含义：

| 参数 | 含义 | 本项目取值 |
|------|------|-----------|
| `(x, y, z)` | 夹爪目标坐标，**单位 cm** | 来自视觉换算 |
| `pitch` | 期望俯仰角（度） | `-90`（夹爪朝下） |
| `alpha1, alpha2` | 俯仰角搜索范围，自动找最近解 | `-90 ~ 100` |
| `movetime` | 舵机运动时间，**单位秒** | `1.0` 左右 |

**返回值**：找到解返回 `(servos, alpha, movetime)`，找不到返回 `False`。所以
`pick_at` 里必须先判断：

```python
if self.move_to(x, y, self.p['pick_z']) is False:
    print('pick_at: 逆运动学无解 x=%.1f y=%.1f' % (x, y))
    return False
```

"无解"= 目标点超出了机械臂的工作空间（太远/太近/太高）。这是抓取失败最常见
的原因，所以要把坐标打出来，方便判断是视觉给错了坐标还是目标真不可达。

## 4. 关键概念：夹爪就是 25 号总线舵机

```python
def open_gripper(self, movetime=0.5):
    self.board.bus_servo_set_position(movetime, [[25, self.p['gripper_open']]])
def close_gripper(self, movetime=0.5):
    self.board.bus_servo_set_position(movetime, [[25, self.p['gripper_close']]])
```

- `bus_servo_set_position(时间秒, [[舵机号, 脉宽]])`
- 25 号舵机是夹爪：脉宽 `120` = 张开，`550` = 闭合（来自官方 block_fetch）
- `movetime` 传的是**秒**，SDK 内部会转成毫秒——这是官方 SDK 的约定，
  和 `arm_ik` 的 movetime 单位一致

如果夹爪夹不紧或夹不住，先单独调这两个脉宽值，而不是改抓取逻辑。

> 新手提示：`self.p['gripper_open']` 里的 `self.p` 就是 `params['arm']`，
> 嵌套字典取值方法见 [robot_config.md 第 2 节「新手必读」](robot_config.md)。

## 5. 抓取动作时序

```
reset_pose: 张开夹爪 → 回到检测姿态 (0,15,5)
pick_at:
  1. move_to(x, y, pick_z=5)    夹爪移到目标正上方/低处
  2. close_gripper              夹取
  3. move_to(raise_pose)        抬起，防止拖着走
release: 移到 release_pose(-5) 松开
```

每步之间的 `time.sleep` 是为了等舵机动作完成，值太小会出现"还没到位就夹"。

## 6. 常见问题排查

| 现象 | 可能原因 | 排查/解决 |
|------|----------|-----------|
| 打印"逆运动学无解" | 坐标超出工作空间 | 打印 (x,y) 看数值；机械臂可及范围约 x±8cm、y≤24cm |
| 夹爪不动 | 25 号舵机脉宽超范围 / 舵机没上电 | 单独测试 `bus_servo_set_position` |
| 夹不住目标 | 开合脉宽不适合目标尺寸 | 微调 `gripper_open/close` |
| 抓取点总是偏高/偏低 | `pick_z` 不合适 | 小步调整 `arm.pick_z` |
| 机械臂动作很抖 | `movetime` 太短、舵机负载大 | 增大 movetime；确认电池电压 ≥10V |
| 舵机堵转发热 | 目标不可达还硬夹 | 加坐标范围判断，超出就报错 |

## 7. 动手练习

1. 单独写测试：依次调用 `move_to(0,10,10)`、`move_to(5,20,5)`，观察机械臂
   是否按坐标移动
2. 打印 `setPitchRangeMoving` 的返回值，理解"解"和"无解"的区别
3. 把 `raise_pose` 改成 `(0, 15, 10)`，观察抬起轨迹变化，体会工作空间边界
