# 地面机器人代码重构方案（REFACTOR_PLAN）

> 状态：方案稿（待确认） · 适用范围：`GroundRobot/Code` · 目标平台：SpiderPi Pro / 树莓派
>
> 本文档不会被 `sync_to_robot.sh` 同步到机器人（脚本已排除 `*.md`），只保存在本地仓库。

## 0. 背景与问题

对代码全量审查后发现，`Code/` 里并存着两套互相竞争的机器人程序：

| 程序 | 入口 | 使用的库 | 状态 |
|---|---|---|---|
| 旧整合 | `advanced/autonomous_pick.py` | `functions/` + `kinematic_routines/arm_pick.py` + 官方 SDK 直连 | README 描述的是它，但已不再迭代 |
| 新整合 | `tasks/auto_fetch.py` | `agcs_lib/`（Searcher + Grabber） | 实际迭代的是它，README 完全没提 |
| 跟踪演示 | `YOLO/Code/main.py` | `agcs_lib` + ONNX | 独立演示程序 |

git 历史显示一个月内算法被推翻重写至少 6 次（bgas → search/grab → v2 → 删 v2 → 官方定点 → 红外占比），
每次都没有彻底删除旧文件，造成：

1. **两套阈值系统**：`walk.approach_area=3000` 与 `grab.approach_area_threshold=2800` 语义重叠但互不相通；
2. **面积坐标系混乱**：`detect_color` 在 320×240 空间算面积，参数注释却写"640×480 画面下"；
3. **死代码与残留**：`grab_official.py`、`orientation.py`、v2 的 `.pyc`、7 份 `.bak` 配置；
4. **实机危险项**：`grab` 段 `min_z` 键重复（3.0 被 0.0 覆盖）、前伸循环 `while True` 无硬上限；
5. **"边动边检测"失效**：扫描循环内 `sleep(间隔)` 导致每步只检测 1 帧；
6. **抽象只包了一半**：`functions/` 已是 `agcs_lib` 转发壳，但 `obstacle_avoidance.py`、`arm_pick.py`、`autonomous_pick.py` 仍直连官方 SDK。

## 1. 重构目标

1. **单一入口**：一个命令跑完整流程，杜绝"今天跑哪个看记忆"。
2. **单一配置源**：`robot_params.yaml` 只保留当前算法消费的键，启动时校验并打印生效值。
3. **单层封装**：业务代码只能依赖 `agcs_lib`，官方 SDK 只允许出现在 `agcs_lib` 内部。
4. **显式状态机**：搜索/夹取/导航用显式状态 + 转移表表达，日志自动记录每次转移及原因。
5. **硬性终止**：所有电机循环必须有"目标达成 / 最大步数 / 异常退出"三出口。
6. **可观测**：启动 `doctor` 自检 + 结构化日志 + 参数生效值 dump。

### 非目标

- 不改硬件接线、舵机编号（21/22/23/24/25）；
- 不改官方 SDK（`spiderpi_sdk/`）本身；
- 不重写已经标定好的视觉阈值、相机标定、逆运动学算法；
- 不新增玩法功能（如朝向对齐、高处夹取暂不启用，架构上留位即可）。

## 2. 目标目录结构

保持顶层目录与机器人 `~/spiderpi` 的映射关系不变（`sync_to_robot.sh` 依赖它），只做内部重组：

```
Code/
├── agcs_lib/                      # 唯一业务库（保留，内部重组）
│   ├── __init__.py                # 精简 re-export
│   ├── params.py                  # 加载 + 重复键校验 + 未用键警告 + 生效值打印
│   ├── hardware.py                # RobotContext：Board/IK/ArmIK/相机/超声/TOF/点阵统一生命周期
│   ├── camera.py                  # 取帧管线（统一分辨率常量）
│   ├── vision.py                  # Detection 标准结构 + color/yolo 统一入口
│   ├── geometry.py                # camera_to_world / pixel_to_arm_coord（唯一换算点）
│   ├── motion.py                  # 六足步态封装（保留，删无用方法）
│   ├── gimbal.py                  # 21/24 云台封装（脉宽范围/步长/死区集中管理）
│   ├── arm.py                     # 机械臂 IK + 夹爪封装（吸收 arm_pick.py）
│   ├── sensors.py                 # 超声/点阵（保留）
│   ├── tracker.py                 # 云台 PID 跟踪线程（保留，消费 Detection）
│   ├── fsm.py                     # 新增：极简状态机框架
│   ├── search.py                  # 重构：SCAN → LOCK → ALIGN → APPROACH 状态机
│   ├── grab.py                    # 重构：REACH → DESCEND → CLOSE → LIFT 状态机
│   ├── logs.py                    # 保留
│   ├── grab_official.py           # 删除（git 历史可恢复）
│   └── orientation.py             # 删除（git 历史可恢复）
├── tasks/
│   ├── main.py                    # 唯一入口：fetch / nav / demo / doctor
│   ├── auto_fetch.py              # 删除（逻辑并入 main.py fetch）
│   └── ceshi/                     # 保留为开发工具（数据采集/单测脚本）
├── config/
│   └── robot_params.yaml          # 清理后的单一参数源
├── communication/
│   └── task_server.py             # 保留（HTTP 任务/状态/推流）
├── functions/                     # 降级为兼容壳（见决策 D5），最终删除
├── advanced/autonomous_pick.py    # 删除（导航逻辑并入 agcs_lib/nav.py，见决策 D1）
├── kinematic_routines/arm_pick.py # 删除（并入 agcs_lib/arm.py）
├── tests/                         # 新增：无硬件纯逻辑测试（不同步到机器人）
└── docs/
    └── REFACTOR_PLAN.md           # 本方案
```

### 依赖规则

```
tasks/main.py
    └── agcs_lib.{fsm, search, grab, nav, params, logs}
            └── agcs_lib.{hardware, vision, geometry, motion, gimbal, arm, sensors, camera, tracker}
                    └── 官方 SDK（common / calibration / sensor / arm_ik）
```

- 业务脚本（tasks/、communication/ 之外）禁止直接 `import common.* / calibration.* / sensor.* / arm_ik.*`；
- 只允许上层调用下层，禁止反向依赖；`agcs_lib` 内部模块之间尽量只依赖 `hardware/vision/geometry/motion/gimbal/arm`。

## 3. 关键设计决策

### D1 导航（NAV 状态机）是否保留？

- **默认：保留**，但拆成独立模式 `main.py nav`，复用 `agcs_lib` 的步态/避障封装，不再依赖 `autonomous_pick.py`。
- 如果实际使用场景是"放好目标→本地搜索→夹取"，不需要地面站导航，可以整个砍掉，方案对应简化。

### D2 夹取算法保留哪套？

- **默认：保留 `grab.py`（前伸 + 画面占比 + 红外下降）**，删除 `grab_official.py`。
- 理由：`auto_fetch.py` 当前实际用的是它；官方定点夹取无人引用。
- 若之后想回到官方定点夹取，从 git 历史恢复即可，不需要现在保留两套。

### D3 YOLO 统一到哪套？

- **默认：统一到 ONNX 版**（`functions/yolo_detect_onnx.py`），因为它不依赖 torch/ultralytics，树莓派上可跑。
- ultralytics 版 `functions/yolo_detect.py` 删除或降级为"提示改用 ONNX"的壳。

### D4 面积基准统一为多少？

- **默认：320×240**。`detect_color` 现在就是缩放到 320×240 后算面积，所有阈值（`min_area`、`approach_area_threshold`、`grab_area_ratio`）都按这个口径，并把参数注释里的"640×480"全部改掉。
- `Detection` 结构固定带 `meta.frame_area`，夹取占比用 `area / frame_area` 计算，不再硬编码。

### D5 `functions/` 兼容层去留？

- **默认：保留一个发布周期**，作为 deprecated 壳（打印一次告警），保证 `SpiderPi.py` 的 RPC 接口（`init/start/stop/exit/run`）和旧脚本不中断；重构稳定后删除。
- `functions/color_detect.py` 的官方 RPC 接口是硬约束，重构时必须保留等价物。

## 4. 模块级改造说明

### 4.1 `params.py` — 配置加载与校验

```python
def load_params(path=None):
    """加载 + 校验。"""
    # 1. 文本级扫描：同段内重复键 → 直接抛错（杜绝 min_z 被静默覆盖）
    # 2. yaml.safe_load
    # 3. 校验每个模块必需键存在（模块启动时调用 require(section, keys)）
    # 4. 未知/未使用键 → 启动警告（warn_unused）
    # 5. 返回冻结后的配置（启动时打印关键生效值，如
    #    "grab.min_z=3.0  approach_area_threshold=2800(320x240)  obstacle.threshold=35cm"）
```

校验规则：

- 重复键 = 配置错误，必须失败，不允许静默取最后一个；
- 每个阶段（fetch/nav/demo）启动时打印该阶段消费的全部参数；
- 提供 `params.diff_used()` 供 `doctor` 列出"当前代码没读但 yaml 里有"的键。

### 4.2 `hardware.py` — 统一生命周期

```python
class RobotContext:
    def __init__(self, need_camera=True, need_tof=False):
        # board / ik / ak / camera / ultrasonic / tof / display
    def __enter__(self): ...
    def __exit__(self): ...   # camera_close + 复位 + 舵机释放
```

- 相机、TOF、超声、点阵只在这里初始化一次，全程序共享；
- TOF（I2C 0x29）初始化失败时记录日志并降级（`tof=None`），不允许整个程序退出；
- 所有入口用 `with RobotContext(...)`，`finally` 里统一清理，消灭散落在各脚本里的 `camera_close()`。

### 4.3 `vision.py` / `geometry.py` — Detection 标准化

```python
@dataclass
class Detection:
    center: tuple[int, int]      # 640×480 原始帧坐标（画框用）
    center_small: tuple[int, int] # 320×240 检测空间坐标（跟踪用）
    area: float                  # 320×240 空间面积（唯一面积口径）
    radius: float
    contour: object
    frame_area: int = 320 * 240
    meta: dict = field(default_factory=dict)  # color/conf/…
```

- `detect_color` / `YoloDetector.detect` 都返回 `Detection` 或 `None`，接口完全一致；
- `pixel_to_arm_coord` 移到 `geometry.py`，去掉 `int()` 截断（保留亚毫米精度）；
- `grab.py` 不再接收 `K, R, T`（占比夹取用不到），需要定点夹取时从 `geometry` 取。

### 4.4 `motion.py` / `gimbal.py`

- `motion.py` 保留 `stand / move_body / turn_left / turn_right / go_forward / go_back`，删掉无人使用的 `move_body_xyz`（或标注保留待用）；
- `gimbal.py` 集中管理 21/24 号：
  - 脉宽范围、步长、死区、`pan_band`、`pan_band_fine`、`track_dead_*` 全部进 yaml 的 `gimbal` 段；
  - 提供 `move_pan(step)` / `move_tilt(step)` / `home()` / `center()`；
- `search.py` 和 `tracker.py` 里的云台操作全部改为调用 `gimbal`，消除 `_set_cam` / `_cam` / `_smooth_*` 里重复的 clamp + 日志代码。

### 4.5 `arm.py`

- 吸收 `kinematic_routines/arm_pick.py` 的 `open_gripper / close_gripper / move_to / reset_pose / pick_at`；
- 保留 `flip_servos` / `flip_gripper` 方向校正逻辑；
- `raise_pose / detect_pose` 等当前未使用的配置在重构后统一处理：要么被新状态机使用，要么从配置删除。

### 4.6 `fsm.py` — 极简状态机框架（约 60 行，不引第三方）

```python
class FSM:
    def __init__(self, name, initial, transitions, log):
        # transitions: dict[state, dict[event, (next_state, reason)]]
    def handle(self, event, **ctx):
        # 转移前记录：[fsm] STATE_A --event--> STATE_B (原因)
        # 非法事件记录 ERROR 日志，不静默
```

约定：

- 每个状态一个 `update(ctx) -> event` 方法，方法内只做"检测 + 决策"，不隐式改状态；
- 转移表集中声明，`doctor` 可以打印整张表；
- 每个状态有 `on_enter / on_exit` 钩子（进状态时复位计数器、出状态时清理线程）。

### 4.7 `search.py` — 状态机化

状态与事件：

```
SCAN_TILT ──FOUND──▶ LOCK ──CONFIRMED──▶ ALIGN_BODY ──ALIGNED──▶ APPROACH ──ARRIVED──▶ DONE
   │                   │                     │                        │
   └──SWEEP_DONE──▶ SCAN_PAN              ──LOST──▶ SCAN_TILT     ──LOST×3──▶ SCAN_TILT
                                                                    └──BLOCKED──▶ STOPPED(避障)
```

必须修复的旧问题：

1. **边动边检测失效**：不再用"发一个移动指令 + sleep 整个间隔"的模式，改为：

   ```python
   # 每小步：先发舵机指令，然后在移动时间内用短轮询持续检测，
   # 检测间隔与移动时间解耦（detect 本身耗时也应计入）。
   self.gimbal.move_tilt(step)            # 移动指令（1-2ms 生效）
   deadline = time.time() + move_duration
   while time.time() < deadline:
       r = self.detect()
       if r: return self._lock_on(r)
       time.sleep(min(0.005, deadline - time.time()))
   ```

2. 扫描步长、档位、死区全部来自 `gimbal` 段配置，删除魔法数字；
3. `_approach` 返回值简化为 `Detection | None`，不再返回 `(center, cy)` 这种半截结构；
4. `max_approach` 保留并加"总超时"双上限，避免卡死；
5. `tof` / `_tof_distance_cm` / `edge_margin` / `px_per_deg` / `near_radius` 等死参数随配置清理一并删除。

### 4.8 `grab.py` — 状态机化 + 硬终止

```
REACH ──RATIO_OK──▶ DESCEND ──TOF_OK──▶ CLOSE ──CLOSE_OK──▶ LIFT ──▶ DONE
   │                   │                    │
   └──REACH_LIMIT──▶ CLOSE(兜底)         └──DESCEND_FAIL──▶ FAIL(记录原因)
```

必须修复的旧问题：

1. `_reach` 加硬上限：`y > reach_y_max` 或步数 > `max_reach_steps` 即停止，禁止只依赖 IK 无解；
2. `_descend` 的返回值**必须检查**：失败时进入 `FAIL` 并记日志，不允许静默继续闭夹；
3. `min_z` 只保留一个（建议 3.0，安全优先），重复键由 `params.py` 启动校验兜底；
4. `K, R, T`、`ultrasonic` 从构造函数移除（当前算法用不到）；`raise_pose / detect_pose` 要么用起来要么删；
5. 夹取动作序列（闭夹→抬升→松开→复位）抽成 `arm` 层方法，`grab` 只管决策。

### 4.9 `tracker.py`

- 保留 PID 跟踪线程，改为消费 `Detection`；
- 把 `SetPoint(320,240)`、死区、`pan_band` 等参数从构造参数改为读取 `gimbal` 配置；
- 与 `grab` 阶段的交互改为显式：`tracker.stop()` 与 `searcher.stop()` 合并为一次清理，避免重复调用。

### 4.10 `tasks/main.py` — 唯一入口

```bash
python3 tasks/main.py fetch  --color blue --detector color|yolo [--ratio 0.3]
python3 tasks/main.py nav     # 等地面站任务（整合 task_server）
python3 tasks/main.py demo    # 单项验证：color / yolo / obstacle / gimbal
python3 tasks/main.py doctor  # 自检：yaml 校验、相机、控制板、超声、TOF、日志路径
```

- `--ratio` 改为只影响本次运行的 `Grabber` 实例参数，不改共享 `params` 字典；
- `doctor` 不移动任何舵机，只做只读自检，供"出问题时第一步跑什么"。

### 4.11 `logs.py` / `task_server.py`

- `logs.py` 保留；FSM 日志统一格式 `[fsm:search] SCAN_TILT --FOUND--> LOCK (原因: 中心x=320px 面积=2800)`；
- `task_server.py` 保留，由 `main.py nav` 启动；`set_status` 增加 `fsm_state` 字段，地面站能看到当前状态机位置。

## 5. 参数迁移表

口径说明：**"保留"= 当前代码确实读取**；**"删除"= 当前代码不读**（含旧算法残留）；迁移后每个键只出现一次。

### 5.1 `vision`

| 键 | 处置 | 说明 |
|---|---|---|
| `target_color` | 保留 | |
| `min_area` | 保留 | 注释改为"320×240 检测空间最小面积" |
| `camera_rotate` | 保留 | |

### 5.2 `walk`（旧整合专用段，整体并入新段）

| 键 | 处置 | 说明 |
|---|---|---|
| `stride / speed / turn_angle / turn_speed` | 保留 → 迁到 `approach` 段 | nav/逼近步态用 |
| `align_tolerance` | 保留 → `approach` | |
| `approach_area` | **删除** | 被 `grab.approach_area_threshold` 取代，口径一致化 |
| `reach_x / reach_y` | **删除** | 仅 `grab_official`（已删）使用 |

### 5.3 `obstacle`

| 键 | 处置 |
|---|---|
| `threshold / target_radius_gate` | 保留（search 用） |
| `back_stride / filter_window` | 保留（nav/避障用，若 D1 保留导航） |

### 5.4 `nav`

| 键 | 处置 |
|---|---|
| `arrive_radius / heading_tolerance / max_walk_m` | 保留（若 D1 保留导航） |

### 5.5 `arm`

保留：`gripper_open / gripper_close / reset_pulses / flip_servos / flip_gripper / detect_pose`。

待决策：

- `pitch / alpha1 / alpha2 / pick_z / raise_pose / release_pose`：定点夹取相关，若 D2 保留占比夹取且不需要定点模式，可删除；
- `approach_z / grab_pitch / pick_distance / hand_raise_pose / hand_hold / hand_raise_pulses`：**删除**（旧整合残留，无人使用）。

### 5.6 `search`

保留：`pan_pulses / tilt_pulses / tilt_scan_step / body_turn_speed`。

删除：`tilt_step_ms / pan_step_ms / align_tol / align_step / edge_ratio / confirm_frames / min_area / body_turn_angle`（代码硬编码或改用 `gimbal` 段）。

### 5.7 `align`（整段删除）

朝向对齐功能当前未启用，`orientation.py` 一并删除；将来启用时用 git 历史恢复。

### 5.8 `gimbal_fetch` → 更名 `gimbal`

保留：`settle_ms / scan_move_ms / detect_interval_ms / pan_scan_step / pan_move_ms / pan_detect_interval_ms / turn_sign / max_approach / slow_radius / pan_band / pan_band_fine / pan_turn_deg / fast_walk_mm / fast_speed / walk_mm / walk_speed / track_dead_cx / track_dead_cy`。

删除（旧算法残留）：`pan_min / pan_max / pan_step / cam_start / cam_max / cam_step / min_area / ultrasonic_max_cm / area_k / near_cm / near_radius / base_pulse / radius_per_pulse / stop_pulse_min / stop_y / turn_deg / edge_margin / px_per_deg / cx_tol / cy_tol / fine_pan_step / fine_tilt_step / max_fine / max_retries / shoulder_step / pos_tol / cy_target / cy_sign / body_lift_step / body_lift_max / arm_tilt_step / height_max_iter / centering_max_iter / pan_gain / grab_attempts / ultra_min_bottom / fine_cm / scan_rounds / slow_cm`。

### 5.9 `grab`

保留：`approach_area_threshold / coarse_z / reach_y_start / reach_y_max（启用，作为硬上限）/ grab_area_ratio / reach_gain / reach_step_min / reach_step_max / lift_pulse / tof_grab_cm / descend_step_cm / min_z（只留 3.0）/ descend_max_steps / descend_move_ms / descend_settle_ms / attempts`。

删除（约 80 个旧键，均为前几代算法残留）：`align_x_cm / align_y_cm / y_ref / y_forward / grab_y_min / grab_y_max / fine_step_mm / move_tol_px / pos_ratio / align_iter / big_step_mm / big_radius_gap / tilt_live / verify_bottom_margin / tilt_body_lift / body_lift_* / height_max_iter / tilt_pose / tilt_pitch / tilt_cy_* / tilt_cx_* / tilt_turn_deg / tilt_rounds / fixed_grab* / servo_* / tof_trigger_cm / down_z_* / forward_y / servo_raise_z / servo_place_pulse / cam_offset_* / z_step_down / z_tries / grab_ref_* / grab_*_gain / approach_radius / grab_radius / high_pulse_threshold / cy_ref / height_gain / ultra_weight / ultra_max_cm / base_pulse / height_gain_pulse / area_ref / area_z_gain / cy_target / cy_tol / cy_sign / flat_h_cm / visible_margin_px / usable_margin_px / cam_min / cam_max / height_enabled / reacquire_enabled / ultra_samples / ultra_rounds / ultra_jump_cm / ultra_offset_cm / ultra_gate_cm / ultra_match_cm / ultra_min_h_cm / ultra_max_h_cm / tof_samples / grab_trigger_mm / fast_switch_mm / tof_abnormal_max / tof_abnormal_min / fast_step_cm / slow_step_cm`。

> 删除前建议用 git 打 tag（如 `refactor/before-cleanup`），需要时随时恢复。

## 6. 文件处置清单

| 文件 | 处置 | 说明 |
|---|---|---|
| `agcs_lib/params.py` | 重构 | 加校验/告警/生效值打印 |
| `agcs_lib/hardware.py` | 重构 | 升级为 RobotContext |
| `agcs_lib/camera.py` | 保留 | 并入 RobotContext |
| `agcs_lib/vision.py` | 重构 | Detection 标准化 + 统一入口 |
| `agcs_lib/geometry.py` | 新增 | 从 vision.py 拆出坐标换算 |
| `agcs_lib/motion.py` | 保留 | 删无用方法 |
| `agcs_lib/gimbal.py` | 新增 | 云台封装 |
| `agcs_lib/arm.py` | 重构 | 吸收 arm_pick.py |
| `agcs_lib/sensors.py` | 保留 | |
| `agcs_lib/tracker.py` | 重构 | 消费 Detection + 参数来自 gimbal 段 |
| `agcs_lib/fsm.py` | 新增 | 极简状态机 |
| `agcs_lib/search.py` | 重构 | FSM + 修边动边检测 + 删死参数 |
| `agcs_lib/grab.py` | 重构 | FSM + 硬终止 + 检查 _descend 结果 |
| `agcs_lib/logs.py` | 保留 | |
| `agcs_lib/grab_official.py` | **删除** | 死代码 |
| `agcs_lib/orientation.py` | **删除** | 死代码 |
| `tasks/auto_fetch.py` | **删除** | 并入 tasks/main.py |
| `tasks/main.py` | 新增 | 唯一入口 |
| `tasks/ceshi/*` | 保留 | 开发工具，加"已过时请用 main.py demo"头部提示 |
| `config/robot_params.yaml` | 重构 | 按 §5 迁移表瘦身 |
| `config/robot_params.yaml.bak_*` | **删除** | 7 份备份，git 历史可恢复 |
| `communication/task_server.py` | 保留 | main.py nav 启动它 |
| `functions/robot_config.py` | 降级壳 | 保留一个周期后删除 |
| `functions/vision_utils.py` | 降级壳 | 同上 |
| `functions/color_detect.py` | 保留壳 | RPC 接口是硬约束 |
| `functions/obstacle_avoidance.py` | **删除** | 逻辑并入 agcs_lib/motion + nav |
| `functions/yolo_detect.py` | **删除** | 统一 ONNX（D3） |
| `functions/yolo_detect_onnx.py` | 保留 | 移入 agcs_lib/vision 或保留位置 |
| `advanced/autonomous_pick.py` | **删除** | 导航并入 agcs_lib/nav.py（D1） |
| `kinematic_routines/arm_pick.py` | **删除** | 并入 agcs_lib/arm.py |
| `YOLO/Code/*` | 保留 | 演示程序，依赖 main.py 修复后同步 |
| `**/__pycache__/*.pyc` | **删除** | 尤其 bgas/search_v2/grab_v2/auto_fetch_v2 |
| `tests/` | 新增 | 本地纯逻辑测试 |

## 7. 分阶段实施计划

每阶段独立可验证、可回滚（git tag），不要一次性大爆炸重构。

### Phase 0 — 止血（半天，先做，实机危险项）

改动：

1. `params.py`：文本级扫描重复键，重复即抛错；
2. `grab.py`：`_reach` 加 `reach_y_max` + `max_reach_steps` 硬上限；`_descend` 返回值检查，失败进 FAIL 并记日志；
3. `search.py`：修复 `_smooth_tilt_to` / `_smooth_pan_to` 的边动边检测；
4. 启动时打印关键生效参数。

验证：

- 机器人上完整跑一遍 `fetch`，确认下降不再触底、前伸不会无限；
- 日志里能看到"边动边检测"每步多次取帧的记录。

产出：危险项消除，行为可复现。

### Phase 1 — 收敛入口（1–2 天）

改动：

1. 新增 `tasks/main.py`，`fetch` 子命令 = 旧 auto_fetch 行为；
2. 删除 `grab_official.py`、`orientation.py`、`__pycache__`、`.bak_*`；
3. YOLO 统一 ONNX（D3）；
4. git tag `refactor/phase1`。

验证：

- `python3 tasks/main.py fetch --color blue` 与旧 `auto_fetch.py` 行为一致；
- `rg -n "grab_official|orientation"` 无命中；
- 机器人端 `tasks/` 目录同步后无残留（sync 脚本 `--delete` 生效）。

### Phase 2 — 分层与状态机（2–3 天，核心）

改动：

1. `hardware.py` 升级 RobotContext，统一生命周期；
2. `vision.py` Detection 标准化 + `geometry.py` 拆分；
3. `fsm.py` + `search.py`、`grab.py` 状态机化（§4.6–4.8）；
4. `gimbal.py` 抽出云台操作；
5. `tracker.py` 消费 Detection。

验证：

- 本地 `tests/`：用假 detect/假 IK 跑 FSM 转移（无硬件）；
- 机器人：`fetch` 全流程 + `nav`（若保留）各跑一遍；
- 日志中每步状态转移有 `SCAN_TILT --FOUND--> LOCK (原因...)` 记录。

### Phase 3 — 配置瘦身（1 天）

改动：

1. 按 §5 迁移表删除未用键、去重、修正单位注释；
2. README 更新为单一流程；教学 md 与代码同步；
3. `doctor` 子命令完成。

验证：

- `doctor` 通过：无重复键、无未用键警告（或警告列表与预期一致）；
- 启动打印的生效参数与 yaml 一一对应；
- `fetch`/`nav` 全流程复跑无回归。

### Phase 4 — 收尾（1–2 天，可选）

改动：

1. `functions/` 兼容壳打告警（D5），最终删除；
2. `advanced/`、`kinematic_routines/` 删除；
3. `task_server` 状态增加 `fsm_state`，地面站可看到状态机位置；
4. 教学文档全面更新。

验证：

- `SpiderPi.py` 加载 `color_detect` 的 RPC 接口仍可用；
- 旧脚本提示"请改用 main.py"；
- git 历史可恢复任何被删文件。

## 8. 测试与验证策略

### 8.1 本地无硬件测试（tests/，不同步到机器人）

- `test_params.py`：重复键检测、必需键校验、未用键告警；
- `test_geometry.py`：`camera_to_world` / `pixel_to_arm_coord` 精度（去掉 int 截断后）；
- `test_fsm.py`：search/grab 状态机转移表 + 非法事件行为，用 FakeDetect / FakeIK 注入；
- `test_vision.py`：Detection 面积口径一致性（320×240）、占比计算；
- `test_config_used.py`：代码读取的键 ⊆ yaml 存在的键（防拼写错误）。

### 8.2 机器人联调清单（每阶段提交前）

1. `main.py doctor` 全绿；
2. `demo color`：目标颜色识别稳定；
3. `demo gimbal`：21/24 扫描、回中正常；
4. `fetch`：放目标在 3 个不同位置（左/中/右、近/远），各跑一遍，记录成功率；
5. 抓取后机械臂复位、夹爪张开、`camera_close` 正常；
6. 相机独占：确认没有第二个进程占用摄像头（`sudo systemctl stop spiderpi`）。

## 9. 风险与回滚

| 风险 | 缓解 |
|---|---|
| 机械臂撞地/撞目标 | Phase 0 先行：min_z 校验 + reach 硬上限 + descend 结果检查 |
| 删文件误删可用逻辑 | 每阶段前 git tag；被删文件均在历史中 |
| 同步脚本要求工作区干净 | 每阶段先 commit 再同步；`sync_to_robot.sh` 已有未提交检查 |
| 相机/舵机被多个进程抢占 | RobotContext 统一生命周期 + 联调清单第 6 条 |
| FSM 重构引入行为变化 | 每阶段保持"行为等价"验收；本地单测覆盖转移表 |
| 参数瘦身误删仍在用的键 | Phase 3 前先跑 `test_config_used.py` + `doctor` |

回滚策略：git tag 粒度（`refactor/phase0` … `refactor/phase4`），任一步出问题 `git checkout` 对应 tag；机器人端重新跑 `sync_to_robot.sh` 即恢复。

## 10. 开放问题（需要确认）

- [ ] **D1** 导航（NAV）模式保留还是砍掉？默认保留。
- [ ] **D2** 夹取算法保留"占比+红外"、删除官方定点？默认是。
- [ ] **D3** YOLO 统一 ONNX、删除 ultralytics 版？默认是。
- [ ] **D5** `functions/` 兼容层保留一个周期再删？默认是。
- [ ] 是否同意 Phase 0 先单独做（半天，独立于大重构）？

确认后即可按 Phase 0 → 4 依次实施。
