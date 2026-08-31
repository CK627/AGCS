# 函数全量审计（FUNCTION_AUDIT）

> 日期：2026-08-31 · 范围：`GroundRobot/Code`（不含官方 SDK `spiderpi_sdk/` 与独立演示 `YOLO/`）
> 结论标签：**无** = 正常；**冗余/死代码** = 无人调用或与其它实现重复；**Bug** = 逻辑问题或风险点。

## 0. 总览

代码实际上并存**三套体系**：

| 体系 | 入口 | 状态 |
|---|---|---|
| 新主流程 | `tasks/auto_fetch.py` + `agcs_lib/` | 实际在迭代，本次审计重点 |
| 旧整合 | `advanced/autonomous_pick.py` + `functions/` + `kinematic_routines/` | README 描述但已停更，待迁移/删除 |
| 工具脚本 | `tasks/ceshi/` | 数据采集/单测，部分与主流程重复 |

### 死代码汇总（可直接删，git 历史可恢复）

| 位置 | 内容 |
|---|---|
| `agcs_lib/orientation.py` 整文件 | 朝向角对齐，无调用（align.enabled=false） |
| `agcs_lib/grab_official.py` 整文件 | 官方定点夹取，无入口引用 |
| `agcs_lib/search.py:101` | `_tof_distance_cm()` 从不调用 |
| `agcs_lib/search.py:128` | `_set_cam()` 从不调用 |
| `agcs_lib/motion.py` | `move_body()` / `move_body_xyz()` / `go_back()` 无人调用 |
| `agcs_lib/search.py` __init__ | 11+ 个死参数（见 1.7） |
| `functions/robot_config.py` / `vision_utils.py` | 转发壳，仅旧代码兼容用 |
| `functions/yolo_detect.py` | ultralytics 版，与 ONNX 版并存（D3 决定删除） |
| `kinematic_routines/arm_pick.py` | 与 `agcs_lib/arm.py` 职责重叠 |
| `advanced/autonomous_pick.py` | 旧状态机，与 auto_fetch 平行 |
| `__pycache__/` | bgas / search_v2 / grab_v2 / auto_fetch_v2 残留字节码 |

### Bug 汇总（按风险排序）

| 位置 | 问题 | 风险 |
|---|---|---|
| `vision.py:121 pixel_to_arm_coord` | `int(-w[0])/10.0` 先截断毫米再转 cm，丢精度；`-w` 符号未现场核对 | 中 |
| `tracker.py` / `search.py _lock_on` | 水平（21号）修正方向无 `pan_sign`，未实测验证；若反了会把目标越追越远 | 高（待实测） |
| `vision.py:59 detect_color` | 面积在 320×240 空间算，注释却写 640×480；radius 只按宽度映射回原图 | 中（口径错） |
| `search.py:116 _home` | 写死 `(10,15,30)`，与 `arm.detect_pose=[0,15,5]` 矛盾 | 低 |
| `grab.py:25 __init__` | `K/R/T/ultrasonic` 参数传入但不用，误导 | 低 |
| `search.py:481 run` | 返回 `(center, cy)`，第二个元素无人用 | 低 |
| `auto_fetch.py` | `--ratio` 直接改共享 params 字典（副作用式配置） | 低 |
| `obstacle_avoidance.py distance_cm` | 超声波读取无异常保护，失败会抛 | 低 |

---

## 1. agcs_lib（新主流程库）

### 1.1 params.py — 参数加载

- `L26 _check_duplicate_keys(text, path)`：文本级扫描同段重复键，发现即抛错（防 PyYAML 静默覆盖）。**无**。
- `L57 load_params(path)`：加载 yaml + 重复键校验。**无**。
- `L68 summarize(params)`：生成启动生效参数摘要。**无**。

### 1.2 hardware.py

- `L6 make_board()`：创建官方 `Board`。**无**（Phase 2 将升级为统一生命周期 RobotContext）。

### 1.3 arm.py

- `L17 make_arm_ik(params)`：创建 ArmIK，并应用 `arm.flip_servos` 方向校正。**Bug 候选**：`flip` 若含非 21–24 舵机会 `KeyError`（当前配置 `[]` 未触发）。
- `L24 _CalibratedArmIK.servosMove`：覆写官方方法做脉宽镜像。同上。

### 1.4 motion.py — 六足封装

- `L6 make_ik(board)`：创建 kinematics.IK。**无**。
- `L11 stand(ik, t=500)`：立正复位。**无**。
- `L16 move_body(ik, dz, speed)`：体态升降。**冗余：死代码**（无人调用）。
- `L26 move_body_xyz(...)`：六足精确平移。**冗余：死代码**。
- `L35/39 turn_left / turn_right`：原地转向。**无**。
- `L43 go_forward`：前进。**无**。
- `L47 go_back`：后退。**冗余：死代码**（仅被 `__init__.py` re-export）。

### 1.5 sensors.py

- `L7 make_ultrasonic()`：创建超声波，失败返回 None。**无**。
- `L16 dist_cm(ultrasonic, samples=3)`：多次采样取**最近**有效值（避障用保守值）。**无**。
- `L34 make_display(clk=8, dio=7)`：创建点阵 TM1640，失败 None。**无**。
- `L45 show_status(display, v)`：点阵显示状态数字，失败静默。**无**。

### 1.6 camera.py

- `L7 open_camera()`：打开相机。**无**。
- `L15 capture(cam, tries=20)`：取帧，失败重试后返回 None。**无**。

### 1.7 vision.py — 视觉

- `L30 load_lab_data()`：读 lab_config.yaml 颜色阈值。**无**。
- `L35 load_block_params()`：读相机标定 K/R/T。**无**。
- `L44 get_area_max_contour(contours, min_area)`：返回最大且 ≥min_area 的轮廓。**冗余**：`contour_area_max`/`max_area` 双变量易读错，建议简化。逻辑本身正确。
- `L59 detect_color(img, lab_data, color, size=(320,240), min_area=50)`：直方图均衡 + LAB 阈值 + 形态学 + 最大轮廓 + 画框。**Bug/陷阱**：① 面积在 320×240 空间计算，但 yaml 注释写"640×480 画面下"（口径错误）；② `radius` 只按宽度映射回原图，若原图比例≠4:3 会错；③ 每帧 copy+均衡，树莓派上约 50–150ms。
- `L95 camera_to_world(cam_mtx, r, t, img_points)`：像素→平面世界坐标（官方算法复刻）。**无**。
- `L121 pixel_to_arm_coord(K, R, T, center, initial_coord=(0,15,5))`：像素→机械臂坐标。**Bug**：`int(-w[0])/10.0` 先截断毫米再转 cm 丢精度；`-w` 符号与官方示例需现场核对。
- `L130 load_undistort_maps(size=(640,480))`：加载畸变校正映射。**无**。
- `L141 correct_camera(img, rotate)`：画面旋转。**无**。

### 1.8 orientation.py — 朝向角

- `L14 block_orientation(contour, eps_factor)`：估算方形目标朝向角。**冗余：死代码**。
- `L40 orientation_error(angle, ref_angle)`：周期 90° 角度误差。**冗余：死代码**。

### 1.9 logs.py

- `L35 action_msg(progress, reason, action)`：结构化日志消息拼装。**无**。
- `L48 _module_file(log_root, name)`：日期/模块名/时刻 路径。**无**。
- `L57 _ensure_handlers(logger, log_root, console)`：装配 handler（幂等、目录不可写降级）。**无**。
- `L84 setup_logger` / `L90 get_logger`：入口（文件+终端）/模块（仅文件）logger。**无**。

### 1.10 tracker.py — 云台 PID 跟踪线程

- `L17 __init__`：PID 参数（P=0.2 已调）。**Bug 候选**：x/y 的 SetPoint 硬编码 320/240（当前画面 640×480，中心正确）。
- `L49 _update(r)`：计算 21/24 修正并下发（位置变化才发指令）。**Bug 候选**：水平方向没有 `pan_sign`，**方向未实测验证**——若反了会把目标越追越远（20:43 日志最大嫌疑，待目标就位后探针验证）。
- `L86 _run`：检测循环，移动后等 settle 再取帧。**无**（刚修）。
- `L100 start` / `L113 latest` / `L119 lost_frames` / `L123 stop`：线程生命周期与数据读取。**无**。

### 1.11 search.py — 搜索/逼近（核心）

- `L25 __init__`：参数加载。**冗余：死参数**——`detect_interval`、`pan_detect_interval`（旧边动边检测残留）、`edge_margin`、`px_per_deg`、`turn_deg`、`near_radius`、`radius_per_pulse`、`stop_pulse_min`、`stop_y`、`tof_stop_cm`、`fast_walk_mm`、`fast_speed`、`slow_radius`（新逼近逻辑不再用快慢步）。
- `L101 _tof_distance_cm()`：**死代码**（从不调用，注释也说明 search 阶段红外不可用）。
- `L113 _status(v)`：点阵状态显示。**无**。
- `L116 _home()`：恢复官方初始位。**Bug 候选**：写死 `ak.setPitchRangeMoving((10,15,30),...)`，与 `arm.detect_pose=[0,15,5]` 矛盾（两套魔法数字）。
- `L125 _cam(duration)` / `L128 _set_cam(x, y)`：云台移动。**冗余**：`_set_cam` 无调用。
- `L139 _confirm(tries=4, need_hits=2)`：多帧居中确认。**无**（margin 已收紧到 x=320±120 / y=240±140）。
- `L167 _lock_on(det)`：云台转向居中 + 确认。**Bug 候选**：x 方向符号未验证（同 tracker）；12 次后直接 confirm。
- `L189 _turn_body(angle)`：转身（含 turn_sign）。**无**。
- `L198 _blocked()`：超声波避障判断。**无**。
- `L203 _smooth_tilt_to(x, target_y)`：24 号小步扫描，每步移动→到位→检测。**无**（刚修）。
- `L242 _smooth_pan_to(target_x)`：21 号小步扫描。**无**（刚修）。
- `L277 _obstacle_monitor()`：独立避障线程。**无**。
- `L297 _vertical_sweep(x)`：固定 21 号，24 号上下扫。**无**。
- `L309 search()`：扫描编排（先 500 垂直扫，再各档位 水平+垂直）。**无**。
- `L343 _approach(det)`：逼近。**无**（刚重构：小步走 + 每步等云台拉回中部）。
- `L455 _wait_target(timeout)`：等 tracker 重新看到目标。**无**（新增）。
- `L467 _wait_centered(timeout)`：等云台把目标拉回画面中部。**无**（新增）。
- `L481 run()`：入口（search → approach）。**冗余**：返回 `(center, cy)`，第二个元素无人用。
- `L491 reset_pose()`：云台回中（21=500/24=260）。**无**（新增）。
- `L503 stop()`：停止追踪/避障线程。**无**。

### 1.12 grab.py — 前伸+占比+红外下降夹取

- `L25 __init__`：参数加载。**冗余**：`K/R/T`、`ultrasonic` 传入但完全不用（占比夹取不换算坐标、不用超声）。
- `L65 _status(v)`：点阵显示。**无**。
- `L68 _move(coord, move_ms, settle_ms)`：IK 移动，俯仰降级判失败。**无**（刚修）。
- `L86 _tof_height_cm()`：红外中位数采样。**无**。
- `L104 _descend(y)`：红外下降闭环。**无**（刚修：失败明确返回并记原因）。
- `L135 _close_lift(last_servos)`：闭夹 + 22 号上抬。**无**（新增）。
- `L143 _reach()`：前伸 + 画面占比闭环。**无**（刚加 `reach_y_max`/`max_reach_steps` 硬上限）。
- `L203 run()`：attempts 重试编排。**无**。

### 1.13 grab_official.py

- `L22 official_color_grab(...)`：官方 block_fetch 定点夹取。**冗余：死代码**（无入口引用，与 grab.py 并存）。

---

## 2. tasks

### 2.1 auto_fetch.py — 唯一实际入口

- `L26 main()`：编排（硬件初始化→搜索→逼近→夹取→复位）。**Bug 候选**：`params['grab']['grab_area_ratio'] = args.ratio` 直接改共享配置（副作用式，Phase 2 应改为实例参数）。
- `L92 detect(min_area=150)`：取帧→畸变校正→检测→推流。**无**（刚加推流）。

### 2.2 ceshi/ — 工具脚本

| 文件 | 作用 | 备注 |
|---|---|---|
| `CS-gd.py` / `CS-gd-sx.py` | 高度/单轴扫描数据采集 | 保留 |
| `CS-sx.py` | 单舵机扫描测试 | 保留 |
| `CS-grab-tof.py` | 前伸占比夹取单测 | **与 grab.py 算法重复**，仅调试用 |
| `CS-zq.py` | 纯 x,y,z IK 夹取单测 | 保留 |
| `CS-grab-alt.py` | 双策略夹取对比 | 保留 |
| `CS-grab-servo.py` | 官方 color_track PID 跟踪+红外夹取 | 直连官方 SDK，未走 agcs_lib |
| `CS-servo-cal.py` | 视觉伺服夹取标定 | 保留 |
| `CS-video.py` | 推流链路测试 | **独占摄像头，勿与主程序同跑**（曾导致 18:54 取不到帧） |

---

## 3. communication/task_server.py

- `L53 publish_frame(frame, max_fps=10)`：JPEG 限流推流。**无**。
- `L79 set_status(**kwargs)`：更新共享状态。**无**。
- `L85 get_next_task(timeout)`：从队列取任务。**无**。
- `L93 _create_app()`：Flask 路由。**无**。
- `L100 receive_task()`：POST /task 校验 target.x/y 并入队。**Bug 候选**：无任务 id/时间戳校验；`TASK_FILE` 写入无锁。
- `L116 status()` / `L120 video_mjpeg()`：GET /status 与 MJPEG 流。**无**。
- `L138 start_server()`：后台线程启动 Flask。**无**。

---

## 4. functions / kinematic_routines / advanced（旧体系，待迁移）

### 4.1 functions/robot_config.py、vision_utils.py

**冗余：兼容转发壳**（re-export agcs_lib），仅旧代码 import 路径可用。Phase 3 后删除。

### 4.2 functions/color_detect.py

- `init/start/stop/exit/run`：官方 RPC 风格接口（SpiderPi.py 加载用，**硬约束保留**）。
- `main()`：独立运行入口。**无**。

### 4.3 functions/obstacle_avoidance.py

- `ObstacleAvoidance.__init__/distance_cm/blocked/turn/walk_forward`：旧避障行走。**冗余**：与 `search._blocked` 重复实现。**Bug 候选**：`distance_cm()` 里 `self.ultrasonic.getDistance()` 无异常保护。

### 4.4 functions/yolo_detect.py（ultralytics 版）与 yolo_detect_onnx.py

- 两者接口一致（detect 返回 dict/None）。**冗余**：ultralytics 版依赖 torch，树莓派上重；D3 决定保留 ONNX 版。
- ONNX 版 `_letterbox` / `_nms` / `detect`：**无**（本地推理实现正确）。

### 4.5 kinematic_routines/arm_pick.py（ArmPicker）

- `open_gripper/close_gripper/move_to/reset_pose/pick_at/release`：官方定点夹取封装。**冗余**：与 `agcs_lib/arm.py` 职责重叠；仅旧 autonomous_pick 使用。Phase 2 并入 arm.py。

### 4.6 advanced/autonomous_pick.py（旧状态机）

- `AutonomousPick.__init__/reset_pose/_detect/_step_search/_step_approach/_step_pick/_step_nav/_sync_status/run` + `main`：NAV/SEARCH/APPROACH/PICK 隐式状态机。
- **冗余**：与 auto_fetch 平行（D1 决定迁移导航或整体删除）。
- **Bug 候选**：① 隐式状态机（靠返回值切换），无转移日志；② `_step_pick` 用 `walk.reach_x/y` 判断可及范围（口径与新版不一致）；③ `_step_approach` 丢失判定 `lost_streak > 10` 无时间概念；④ 直接 `ultrasonic.getDistance()` 无异常保护。

---

## 5. 结论与建议（按优先级）

1. **立即可删**（死代码）：`orientation.py`、`grab_official.py`、`_set_cam`、`_tof_distance_cm`、`move_body/move_body_xyz/go_back`、search 的 13 个死参数、v2 残留 `.pyc`。
2. **尽快修**：`pixel_to_arm_coord` 的截断丢精度；`detect_color` 面积口径注释（320×240）；`_home` 魔法数字。
3. **待实测**（目标就位后）：21 号水平修正方向——加 `pan_sign` 配置，用探针验证后固定。
4. **结构收敛**（Phase 1–4）：删 functions 转发壳与旧程序，统一入口 `tasks/main.py`。
