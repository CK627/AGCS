# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目一句话

SpiderPi Pro 六足机器人（树莓派 Pi5）比赛演示 + 自主行走 + 视觉识别 + 机械臂抓取。**当前主线是比赛流程脚本**（`Code/CompetitionUse/` 的 2.1~2.5 步），夹取高度这个老卡点已由**奥比中光 Astra Pro 深度相机**解决。最终目标仍是夹取自备虫子 / 虫子模型，颜色方块只是调算法的中间对象。

> **新会话先读 `进度清单.md`（现状对齐），改 NO6/NO7 脚本前读 `CompetitionUse/NO6-NO7-流程说明.md`。** 本文与这两份互补：它们记「当前做到哪、坑在哪」，本文记「架构与契约」。

## 开发工作流

**代码在机器人上才能运行**（依赖官方 SDK、串口、深度相机），本地只做语法检查（`python3 -m py_compile`）。流程：本地改 `Code/` → scp 单文件到机器人 `/home/pi/spiderpi` → SSH 上跑调试。

```bash
# 同步（默认连 AP 地址；机器人是 STA 局域网模式，IP 会变，传当前 IP）
cd Code
./sync_to_robot.sh pi@10.194.228.89        # 整目录同步（见下方 ⚠️ 警告，慎用）
./pull_from_robot.sh pi@10.194.228.89      # 拉回机器人上现场标定的配置

# ⚠️ 当前（2026-09-15）机器人上的 fixed_route.json 比本地新，整目录同步会覆盖它。
#    只 scp 单个 .py 文件：
export SSHPASS=<机器人密码>
sshpass -e scp CompetitionUse/Auto-capture.py pi@10.194.228.89:/home/pi/spiderpi/CompetitionUse/

# 连机器人 & 调试前必停自启服务（抢串口 /dev/ttyAMA0）
ssh pi@10.194.228.89
sudo systemctl stop spiderpi               # joystick 若在跑也停

# 跑比赛脚本（当前主线）
cd /home/pi/spiderpi/CompetitionUse
python3 VisualTracking.py --color yellow                   # 2.1 视觉追踪
python3 AutoPathfinding.py --color yellow                  # 2.2 自动寻路
python3 AutonomousCrawling.py                              # 2.3 自主抓取
python3 Auto-capture.py --color red                        # 2.5 正常版 (NO6)
python3 Auto-capture-1.py --color blue --pull-up 400       # 2.5 进阶版 (NO7, YOLO)

# 看最新日志（debug 只进文件，终端只打 info）
ls -t /home/pi/spiderpi/logs/*/*.log | head -1 | xargs tail -50

# NO6/NO7 整段运行日志（脚本自己抄的 stdout+stderr，含每块 yaw/误差/颜色微调序列）
ls -t /home/pi/spiderpi/logs/*/autocapture/*.log | head -1 | xargs tail -80
```

无测试框架、无 lint、无构建——"验证"就是在机器人上跑。`Code/tasks/CS/` 是 30+ 个单测 / 标定 / 建图脚本（CS-zq 纯 IK 夹取、CS-sx 搜索、scan_2d 建图、calib_pitch 标俯仰等），按需手动运行。

## 代码架构

### 仓库根（`/Users/jj/Documents/MyCode/AGCS`）是多机器人 monorepo

| 目录 | 作用 |
|------|------|
| `GroundRobot/` | **本文所在**。六足机器人（`Code/` 二开算法 + `Dashboard/` 地面机器人仪表盘） |
| `GroundStation/` | 地面站**中枢**（轮询各仪表盘状态，自动推进度） |
| `Dron/` | 无人机（Code + Dashboard），经 EWRF 图传 / MAVLink 数传 |
| `YOLOModel/` | YOLO 模型训练与检测仪表盘 |
| `config.yaml` | 各仪表盘端口 / 服务地址汇总（参考文档，非运行配置） |

端口约定：中枢 20000、机器人仪表盘 20001、无人机仪表盘 20002、YOLO 仪表盘 20003、广播 20004、无人机图传 20005；机器人 `task_server` 在树莓派 5000；MAVLink 数传 14550 / QGC 8080。

### `Code/` 分层（关键抽象）

`Code/` 镜像机器人 `~/spiderpi`。核心是 **`agcs_lib/` 二开封装层**：业务代码（`CompetitionUse/`、`tasks/`、`tasks/CS/`）只 import `agcs_lib`，绝不直接 import 官方 SDK（`common` / `calibration` / `arm_ik` / `sensor`）。`agcs_lib/__init__.py` 统一 re-export 工厂函数。

| 模块 | 职责 |
|------|------|
| `CompetitionUse/` | **比赛流程 2.1~2.5（当前主线）**，见下节 |
| `agcs_lib/depth.py` | Astra Pro 深度相机（ctypes 直调 `libOpenNI2.so`），单位 mm |
| `agcs_lib/geometry.py` | 像素→机械臂 x,y,z 核心算法（合并版：反投影地面 + 方向校正） |
| `agcs_lib/vision.py` | 颜色检测、轮廓、畸变校正、`pixel_to_arm_coord` |
| `agcs_lib/search.py` | `Searcher`：找目标 + 逼近（已稳定，**不要擅动**） |
| `agcs_lib/grab.py` / `grab_official.py` | 两套夹取实现，**主入口已不走**，仅测试脚本用 |
| `agcs_lib/pcl.py` / `pcl2d.py` | 3D 点云 ICP 配准 / 2D 占据栅格 + 2D ICP |
| `agcs_lib/depthscan.py` | 深度当 2D 激光雷达用：取帧 + 标俯仰 + 点云压 2D |
| `agcs_lib/localize.py` | 深度点云 ICP 匹配预建地图，估相机位姿（校正 IMU 漂移） |
| `agcs_lib/mapview.py` | 2D 地图可视化（终端 ASCII + PNG） |
| `agcs_lib/marker.py` | ArUco 检测 + 位姿估计 |
| `agcs_lib/motion.py` | 六足步态（IK：立正/前进/后退/转身/升降体态） |
| `agcs_lib/arm.py` | 机械臂 IK + 方向校正（`flip_servos`，舵机顺序 24,23,22,21） |
| `agcs_lib/tracker.py` | 独立线程 PID 云台跟踪，主线程经 `latest()` 取数据 |
| `agcs_lib/sensors.py` / `camera.py` / `logs.py` / `params.py` / `hardware.py` / `orientation.py` / `ClampRemoval.py` / `restore.py` | 超声波+点阵 / 取帧 / 日志 / 参数 / Board / 朝向角 / 固定夹持与复位 |
| `communication/task_server.py` | Flask 服务（`/status` `/task` `/video.mjpeg`），机器人↔地面站通信 |
| `tasks/auto_fetch.py` | **旧主入口**（search → grab_official），已被比赛脚本取代 |
| `functions/` `advanced/` `kinematic_routines/` `spiderpi_sdk/` | 官方文件，别动 |

### 当前主线：比赛流程 `CompetitionUse/`

各步骤做成**独立脚本**，通过 `task_server` 上报 `/status`（状态+位置+朝向+已抓取+任务+结果+消息），**地面站中枢轮询该接口按状态关键词自动推进度，机器人不主动通知中枢**。2.4 是团队讨论环节，无代码。

| 步骤 | 脚本 | 作用 | 依赖 |
|------|------|------|------|
| 2.1 | `VisualTracking.py` | 云台 21/24 PID 跟随色块 + 深度估位置 | 仅官方 SDK |
| 2.2 | `AutoPathfinding.py` | 扫描→追踪居中→转身→逼近→深度判距 | 仅官方 SDK |
| 2.3 | `AutonomousCrawling.py` | 固定路线夹取→人脸识别→递物 | 仅官方 SDK |
| 2.5 | `Auto-capture.py`（NO6） | JSON 路线 + IMU 航向 + 颜色微调 + 固定脉宽夹取/放下 | `agcs_lib` + `task_server` |
| 2.5 进阶 | `Auto-capture-1.py`（NO7） | NO6 全套 + YOLO 检测 + 雅可比精对准 | 同上 + `calib_pick1/2.json`、`models/best.onnx` |

辅助：`_common.py`（2.1~2.5 公共初始化 `build_runtime`）、`fixed_route.json`（2.5 路线动作序列）、`depth_3d_grasp.py`（方案 A 深度 3D 抓取验证）、`calib_cam2arm.py`（手眼标定，一次性）。

**NO6/NO7 关键设计**（改这两个脚本前读 `NO6-NO7-流程说明.md`）：forward/back 不立即执行，累加到 `pending_forward` 遇非直行动作才一次性走掉；距离切 100mm chunk 小步闭环；航向由 `agcs_lib/imu.py` 后台线程连续积分，IMU 只修一次转向误差就 `reset_imu()` 归零（`--imu-straight off` 可整体关掉直线段修正）；直线段航向死区**必须左右对称**（`TURN_TOL_DEG=3.0`，`--turn-tol` 可调）——写不对称会把机身稳态推向一侧，装在身上的相机跟着歪，颜色微调就一路往那边平移；颜色微调是平移、**不能**重置 `target_yaw`（重置等于把已攒下的航向误差一笔勾销，误差永不收敛）；夹取/放下前按电压补偿步长；第一次放下后关颜色微调、累计 6 次转弯再开。

## 关键契约与数据流

- **`detect()` 契约**：`detect(min_area)` 返回 `dict(center=(cx,cy), radius, area, color, contour)` 或 `None`。search / competition 脚本 / tracker 都消费这个接口；换 YOLO 检测器只改调用处的闭包内部，下游不动。
- **参数全部在 `config/robot_params.yaml`**，经 `load_params()` 读入。顶层组：`vision` / `walk` / `obstacle` / `nav` / `arm` / `search` / `align` / `gimbal_fetch` / `grab`。调行为 = 改 yaml。注意：删掉过一批 gimbal 调参键，代码用 `.get(key, 默认值)` 兜底不崩，但 settle 等待回落默认值（见 `进度清单.md` §7.3）。
- **距离判定**：主距离用**深度相机**（Astra Pro，mm）；视觉面积估距 `dist = area_k / sqrt(area)`（`gimbal_fetch.area_k`）仅作粗略参考；超声波只做避障（近距离乱跳）。
- **像素→机械臂坐标**：单目用 `geometry.py`（地面平面假设 + `pick_z` 高度参数）；深度相机直接测 `(x,y,z)`，不走平面假设。手眼标定 `config/camera_cal.yaml` 的 `block_params` 只存在于机器人端，本地 `load_block_params()` 失败属正常；`cam2arm.yaml` 本地仍是占位值（R=I,t=0），需现场 `calib_cam2arm.py` 标定。
- **日志**：`logs.py`，`action_msg(progress, reason, action)` 拼结构化中文消息；写到 `/home/pi/spiderpi/logs/<日期>/<时-分>.log`，debug 只进文件。

## 深度相机（Astra Pro）—— 高度问题的解法

- 机器人上 `/home/pi/orbbec_sdk/` 是**自包含 OpenNI2 运行时**（`libOpenNI2.so` + `liborbbec.so`），`depth.py` 用 ctypes 直调；**系统 apt 装的 OpenNI2 不配套，枚举不到设备**。
- **深度**走 OpenNI2（USB `2bc5:0403`），单位 mm；**彩色**走 OpenCV `/dev/video0`（设备 `2bc5:0501` 被内核 uvcvideo 占用），**不要走 `depth.read_color()`**（OpenNI2 `SENSOR_COLOR` 超时读不到帧）。
- 点云坐标：相机系 `X 右 / Y 上 / Z 前`（mm）；机器人系 2D `x 右 / y 前`，绕竖直轴转 `φ = (500 - pan) * 90 / 400`（21 号舵机 500=正前、900=左90°、100=右90°）。

## 硬件事实与坑（改算法前必读）

- **关节**：21=底座横转、22=肩、23=肘、24=腕俯仰（相机装这里）、25=夹爪。搜索动 21/24 + 六足，夹取动 21-25。相机看的方向完全由 21/24 脉宽决定（0~1000，500=朝前，24 起始 260）。
- **机械臂复位位**：`arm.reset_pulses = {21:500, 22:705, 23:90, 24:330, 25:700}`。**本机 IK 与实物有偏差**：复位用标定好的固定脉宽，夹取才用 IK，两者别混。
- **IMU 只有陀螺仪没有磁力计**：yaw 是 gz 按 dt 积分出的相对值，会缓慢漂移。长路线必须「每转一次弯就 `reset_imu()` 重标零漂 + yaw 归零」，误差不跨段传播；NO7 另用深度 ICP 对预建地图校正航向。
- **IMU 必须密集采样**（`agcs_lib/imu.py` 的 `ImuTracker` 后台线程，约 105Hz）。官方 SDK 的 `imu_queue` 是 `maxsize=1` 且满了就丢，`get_imu()` 只能拿到「上次取走之后到达的第一个样本」——**两次调用之间的样本全丢**。所以**绝不能**「要用的时候读一个样本」再乘上一段多秒的 `dt`，那会灌进随机方向的假转角，直线段控制会发散（2026-09-15 现场就是这么转飞的）。`get_imu()` 只读队列不写串口，后台线程不会跟舵机指令抢 `/dev/ttyAMA0`。现场排查用 `CompetitionUse/imu_probe.py`。
- `board.bus_servo_read_position()` 本机**只能写、不能读**物理脉宽（返回 None）。
- **串口 `/dev/ttyAMA0` 同一时刻只能一个进程**占，调试前必 `systemctl stop spiderpi`（joystick 也停）。
- 超声波近距离读数乱跳（-1、突跳几十 cm），不可作主距离。
- 云台 PID 积分累积会抬头过头、把目标追出画面——目标已居中就跳过 PID 更新（tracker 死区）。
- 官方 `action_group_control_demo.py` 的 `turn_right_low` 用 `times=0` 会无限转圈停不下来，别直接用（详见 `复用清单.md`）。
- 摄像头画面远程看不到，需现场确认（夹没夹到、画面）。

## 当前状态与工作约定

- **主线是比赛流程脚本，不是夹取算法**。寻路（`search.py`）与夹取高度（深度相机）都已解决；`tasks/auto_fetch.py` 是旧入口，别当主入口改。
- **当前卡点**：NO6/NO7 第一次夹取后的「拔起」方向/幅度，动的是 22 号「肩」。**基准是夹取位不是复位位**——拔起发生在夹爪闭合之后，此时 22 号停在路线 JSON 的夹取位（当前路线 `22:395`），`restore_travel` 还没跑；所以「往上抬」= 把 22 调到**比夹取位大**。现场已验证 `785` 抬得太高（395→785，+390），当前 `PULL_UP_22 = 450`（395→450，+55，小幅抬）。试值走 `--pull-up N`，不用改代码。
- **机器人上的 `fixed_route.json` 比本地新**（pick 位置整个挪了），别整目录同步，只 scp 单文件。
- 用户反复强调的算法原则（针对自主寻路/夹取，**不是**比赛脚本的固定路线）：
  - 不硬编码距离/步数/固定夹取点；要闭环、按实时画面自动调整。
  - 检测到颜色就锁定，不转圈；不居中/不正就主动调整。
  - 夹取前先看一眼距离；目标略超可及范围就前进微调，不轻易放弃定点夹取。
  - 高处目标要有保底（体态升降 + 抬头检测）。
  - 官方 `block_fetch.py` / `intelligent_fetch.py` / `color_track.py` 是可靠参考。
- 机器人类似坑点/历史/下一步详见 `进度清单.md`；官方 SDK 参考在 `复用清单.md` 与 `资料/SpiderPi Pro六足机器人专业开发套件/3 源码资料/`（只读）。
