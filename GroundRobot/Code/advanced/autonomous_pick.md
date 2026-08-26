# autonomous_pick.py 教学文档

## 1. 这个文件是做什么的

**整合程序**，把三个模块串成完整闭环：超声波避障行走、颜色/YOLO 视觉识别、
机械臂抓取。对应 README 的"识别→移动→抓取"完整流程，是第二阶段地面机器人的
核心目标（验收标准：成功率 ≥70%）。

运行：

```bash
python3 ~/spiderpi/advanced/autonomous_pick.py --color red
python3 ~/spiderpi/advanced/autonomous_pick.py --detector yolo --model /home/pi/best.pt --classes damaged_pod
```

启动后主程序会自动开启 HTTP 服务（`communication/task_server.py`，5000 端口）
并进入 NAV 状态等待地面站下发任务——**不用单独再启动一个 py 文件**。

## 2. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `argparse` | 标准库 | 命令行参数 |
| `cv2` | OpenCV | 显示画面、键盘退出 |
| `common.kinematics` + `Board` | 官方 SDK | 六足运动、扩展板通信 |
| `sensor.ultrasonic_sensor` | 官方 SDK | 测距避障 |
| `calibration.camera` | 官方 SDK | 摄像头 |
| `arm_ik.arm_move_ik` | 官方 SDK | 机械臂逆运动学 |
| `functions.*`（本项目） | 机器人功能模块 | 参数/视觉/避障 |
| `kinematic_routines.arm_pick`（本项目） | 机械臂封装 | 抓取动作 |
| `communication.task_server`（本项目） | HTTP 通信 | 收任务（POST /task）、上报状态（GET /status） |

**依赖关系图**：

```
autonomous_pick.py
  ├── communication.task_server ──── 任务接收/状态上报（自动启动）
  ├── functions.obstacle_avoidance ── 行走/避障
  ├── functions.vision_utils ──────── 视觉 + 坐标变换
  ├── functions.yolo_detect / color_detect ── 检测器（可切换）
  └── kinematic_routines.arm_pick ── 抓取
```

> 新手提示：代码里 `self.params['walk']['turn_angle']` 是嵌套字典取值，先读
> [robot_config.md 第 2 节「新手必读」](robot_config.md)。

## 3. 核心设计：状态机

机器人行为不是"线性脚本"，而是需要**根据当前情况反复决策**，所以用状态机：

```
        ┌──────────────────────────────────────────────────┐
        ▼                                                  │
     NAV ──收到任务──▶ SEARCH ──发现目标──▶ APPROACH ──够近──▶ PICK
        ▲                  ▲                │ 丢失/超范围  │ 完成
        └──(等下一个)──────┴────────────────┴──────────────┘
```

| 状态 | 进入条件 | 行为 | 退出条件 |
|------|----------|------|----------|
| `NAV` | 启动（默认） | 等待地面站 `POST /task`；收到后转向目标、避障前进 | 进入粗定位半径 → SEARCH；超时 → 报 failed |
| `SEARCH` | 启动/目标丢失 | 原地左转 15° 扫描；一圈无果则前进一段 | 检测到目标 |
| `APPROACH` | 发现目标 | 偏了就转向对准；对准了就前进（带避障） | 目标面积 ≥ 阈值 → PICK；连续丢失 → SEARCH |
| `PICK` | 目标够近 | 算坐标 → 抓取 → 抬起 → 松开 → 后退 | 回到 SEARCH |

> 不想等任务、想启动直接扫描，加 `--no-wait-task` 参数即可恢复旧行为。

代码里每个状态一个方法，方法返回**下一个状态**：

```python
if self.state == STATE_SEARCH:
    self.state = self._step_search(frame)
elif self.state == STATE_APPROACH:
    self.state = self._step_approach(frame)
```

这种写法的好处：

- 每个状态独立，好读好改
- 出问题时看画面上的 `State:` 显示，马上知道卡在哪一步
- 加新状态（比如"绕障"）只需新增一个方法

## 4. 逐状态讲解

### 4.1 SEARCH：怎么找目标

```python
self.walker.turn(left=True, angle=w['turn_angle'])   # 每帧左转 15°
self.turn_count += 1
if self.turn_count * w['turn_angle'] >= 360:          # 转满一圈
    self.walker.walk_forward()                        # 前进一段再扫
```

注意：`ik.turn_left` 是阻塞调用，执行完才回循环，所以每帧只转一次，循环节拍
由"舵机动作耗时"自然控制，不需要额外 sleep。

### 4.2 APPROACH：怎么对准

```python
offset = det['center'][0] - img_cx      # 目标中心 - 画面中心
if abs(offset) > w['align_tolerance']:  # 偏得比较多
    self.walker.turn(left=offset < 0, angle=w['turn_angle'])
    return STATE_APPROACH
```

- `offset > 0`：目标在画面右侧 → 机器人右转（`left=False`）
- `offset < 0`：目标在左侧 → 左转
- 偏差在容差内 → 前进

`align_tolerance` 和 `approach_area` 是关键参数：容差太小会左右抖，面积阈值
太小会在没到可抓距离就触发抓取。

### 4.3 PICK：怎么决定能不能抓

```python
wx, wy = pixel_to_arm_coord(self.K, self.R, self.T, det['center'])
if abs(wx) > w['reach_x'] or wy > w['reach_y']:
    return STATE_APPROACH      # 还在机械臂可及范围外
```

视觉给的坐标先过一次"可及范围"检查（x±8cm、y≤24cm），防止机械臂对着
够不到的位置硬伸。这是官方 `block_fetch.py` 里 `Position out of range` 判断
的通用化。

## 5. 数据流总览（理解"哪里出错"）

```
摄像头帧
  → 畸变校正 remap
  → detect_color / yolo  → {center, area}
  → APPROACH 用 center 决定转向
  → PICK 用 center + K/R/T 换算出 (wx, wy) cm
  → ArmPicker.pick_at(wx, wy) → 舵机动作
```

排查问题时的**定位顺序**（从前往后）：

1. 画面里有没有圈出目标？→ 没有：视觉/阈值问题（看 `color_detect` 文档）
2. 圈出了但一直转圈？→ 对准逻辑/`align_tolerance` 问题
3. 走到目标附近但不抓？→ `approach_area` 阈值 / 可及范围判断
4. 抓了但位置偏？→ 相机标定 K/R/T 不准（重做位置校准）
5. 抓取动作本身失败？→ 看 `arm_pick` 文档

## 6. 常见问题排查

| 现象 | 问题环节 | 排查/解决 |
|------|----------|-----------|
| 一直 SEARCH 转圈 | 视觉没检测到 | 先单独跑 `color_detect.py`；检查 `lab_config.yaml` |
| SEARCH 偶尔发现又立刻丢 | `min_area` 太大或检测不稳定 | 调小 `min_area`；检查光照 |
| APPROACH 左右摇摆 | `align_tolerance` 太小 | 调大到 40~60 |
| 接近时撞到目标/障碍 | 超声波没生效或阈值太小 | 单独测 `obstacle_avoidance.py` |
| 走到很近却一直 APPROACH | `approach_area` 没触发 | 调小 `approach_area` 或调大 `reach_y` |
| 抓取点偏移固定量 | 相机标定参数旧 | 重新 `camera_cal_main.py` 校准 |
| 抓完后退时把目标碰倒 | 后退方向/距离不合适 | 改成先 `reset_pose` 再后退 |
| 程序卡死不动 | 某一步阻塞在舵机/串口 | 加打印日志；检查 `Board()` 是否正常 |

## 7. 调试方法论（重要）

1. **先单项后整合**：三个模块各自独立跑通，再跑 `autonomous_pick.py`
2. **参数一次只改一个**：改完记录效果，避免多个变量互相干扰
3. **善用打印**：状态切换、检测到的 `(wx, wy)`、距离值都要打印
4. **失败要留证据**：把画面截图、坐标数值记下来，调参才有依据
5. **成功率统计**：记录"扫描成功/对准成功/抓取成功"三个阶段各自的次数，
   看瓶颈在哪一个阶段

## 8. 动手练习

1. 给每个状态切换加一条带时间戳的日志，跑完看完整决策轨迹
2. 把 `reach_y` 从 24 改成 20，观察抓取时机变化
3. 增加一个 `AVOID` 状态：APPROACH 时遇到障碍就绕行而不是丢目标
   （提示：新增状态方法 + 在 `_step_approach` 里判断 `blocked()`）
