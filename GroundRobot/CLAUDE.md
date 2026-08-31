# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目一句话

SpiderPi Pro 六足机器人（树莓派 Pi5）自主行走 + 视觉识别 + 机械臂抓取。当前先用红/绿/蓝/黄颜色方块调夹取算法，最终目标是夹取自备的虫子 / 虫子模型（不是毛豆荚上的幼虫）。代码分两块：`Code/`（二开算法，本文重点）与 `Dashboard/`（Windows 地面站 HTTP 服务）、`资料/`（官方 SDK/镜像，只读参考）。

## 开发工作流

**代码在机器人上才能运行**（依赖官方 SDK、串口、摄像头），本地只做语法检查。流程：本地改 `Code/` → rsync 同步到机器人 `/home/pi/spiderpi` → SSH 上运行调试。

```bash
# 同步（目录结构与机器人一一对应；默认连 AP 地址，局域网模式要传当前 IP）
cd Code
./sync_to_robot.sh pi@10.194.228.89        # 当前机器人 IP，会变
./pull_from_robot.sh pi@10.194.228.89      # 拉回机器人上现场标定的配置/SDK

# 连机器人 & 调试前必停自启服务（抢串口 /dev/ttyAMA0）
ssh pi@10.194.228.89       # 密码 raspberrypi
sudo systemctl stop spiderpi               # joystick 若在跑也停

# 跑主入口
cd /home/pi/spiderpi/tasks
python3 auto_fetch.py --color blue         # 颜色检测
python3 auto_fetch.py --detector yolo --model models/worm_best.onnx   # 虫识别 ONNX

# 看最新日志（debug 只进文件，终端只打 info）
ls -t /home/pi/spiderpi/logs/*/*.log | head -1 | xargs tail -50

# 单文件手工同步
sshpass -p raspberrypi scp 文件 pi@10.194.228.89:/home/pi/spiderpi/...
```

无测试框架、无 lint、无构建——"验证"就是在机器人上跑。`Code/tasks/ceshi/` 是单测脚本（CS-zq 纯 IK 夹取、CS-sx 搜索等），按需手动运行。

## 代码架构

### 分层（关键抽象）

`Code/` 镜像机器人 `~/spiderpi`。核心是 **`agcs_lib/` 二开封装层**：业务代码（`tasks/`、`ceshi/`）只 import `agcs_lib`，绝不直接 import 官方 SDK（`common` / `calibration` / `arm_ik` / `sensor`）。`agcs_lib/__init__.py` 统一 re-export 全部工厂函数。

| 模块 | 职责 |
|------|------|
| `tasks/auto_fetch.py` | 主入口：编排 search → grab |
| `agcs_lib/search.py` | `Searcher`：找目标 + 逼近（只动六足 1-20 与云台 21/24） |
| `agcs_lib/grab_official.py` | `official_color_grab`：官方 block_fetch 式色块定点夹取 |
| `agcs_lib/grab.py` | `Grabber`：另一套夹取实现（体态升降/抬头夹取/重扫），当前**未接入主入口**，仅测试脚本用 |
| `agcs_lib/tracker.py` | `ColorTracker`：独立线程 PID 云台跟踪（复刻官方 color_track.py），主线程经 `latest()` 取数据 |
| `agcs_lib/vision.py` | 颜色检测、轮廓、`pixel_to_arm_coord` 像素→机械臂坐标、畸变校正 |
| `agcs_lib/motion.py` | 六足步态封装（IK：立正/前进/后退/转身/升降体态） |
| `agcs_lib/arm.py` | 机械臂 IK 封装 + 方向校正（`flip_servos`，舵机顺序 24,23,22,21） |
| `agcs_lib/sensors.py` / `camera.py` / `logs.py` / `params.py` / `hardware.py` / `orientation.py` | 超声波+点阵、取帧、日志、参数加载、Board、朝向角 |

### 关键契约与数据流

- **`detect()` 契约**：`detect(min_area)` 返回 `dict(center=(cx,cy), radius, area, color, contour)` 或 `None`。search/grab/tracker 全部消费这个接口；换 YOLO 检测器只改 `auto_fetch.py` 里闭包内部，下游不动。
- **参数全部在 `config/robot_params.yaml`**，经 `load_params()` 读入。调行为 = 改 yaml，不是改代码。搜索参数在 `search` / `gimbal_fetch`，夹取参数在 `grab` / `arm`。加新参数也要加进 yaml。
- **距离判定**：视觉面积估距 `dist = area_k / sqrt(area)`（`gimbal_fetch.area_k`）；超声波仅做避障（近距离读数乱跳，不可作主距离）。
- **像素→机械臂坐标**：`pixel_to_arm_coord(K, R, T, center, initial_coord)` 依赖机器人上 `config/camera_cal.yaml` 的手眼标定 `block_params`；标定只存在于机器人端，本地无此文件，`load_block_params()` 本地调用会失败属正常。
- **日志**：`logs.py`，`action_msg(progress, reason, action)` 拼结构化中文消息；写到 `/home/pi/spiderpi/logs/<日期>/<时-分>.log`，debug 只进文件。

### 当前流水线

`auto_fetch.py`：`Searcher.run()` 找到目标并逼近到位（返回 center）→ `official_color_grab()` 按官方 block_fetch 动作时序夹取。夹取前有 `time.sleep(1)` 等机器人站稳，避免把未停稳数据传给 IK。

## 硬件事实与坑（改算法前必读）

- **关节**：21=底座横转、22=肩、23=肘、24=腕俯仰（相机装这里）、25=夹爪。搜索动 21/24 + 六足，夹取动 21-25。相机看的方向完全由 21/24 脉宽决定（0~1000，500=朝前，24 起始 260）。
- **机械臂复位位**（现场确认过）：`arm.reset_pulses = {21:500, 22:705, 23:90, 24:330, 25:700}`。**本机 IK 与实物有偏差**：复位用标定好的固定脉宽，夹取才用 IK；两者不要混。
- `board.bus_servo_read_position()` 本机**只能写、不能读**物理脉宽（返回 None）。
- **串口 `/dev/ttyAMA0` 同一时刻只能一个进程**占（机械臂/摄像头），调试前必 `systemctl stop spiderpi`。
- 超声波近距离读数乱跳（-1、突跳几十 cm），**不可作主距离**。
- 云台 PID 积分累积会抬头过头、把目标追出画面——目标已居中就跳过 PID 更新（tracker 死区）。
- 官方 `action_group_control_demo.py` 的 `turn_right_low` 用 `times=0` 会无限转圈停不下来，别直接用（详见 `复用清单.md` / `开始配置.md`）。
- 官方 SDK 参考在 `复用清单.md`（动作组/Board/传感器/IK/ArmIK 调用方式）与 `资料/SpiderPi Pro六足机器人专业开发套件/3 源码资料/`（只读）。

## 当前状态与工作约定

- **寻路算法已完成**（search.py）：能正常找到目标并逼近到位，不要再动。**核心未解决的是夹取**：夹取算法反复尝试（grab.py / grab_v2 / grab_official）始终卡在**目标高度导致的夹取偏差**——目标不在固定高度（或高度一高）夹取点就偏、凑近过头、偶尔空夹。
- **高度是夹取偏差的根源**。当前主入口 `search → grab_official`，而 `grab_official` 把 z 硬编码成固定 `pick_z=5.0`（假设目标在地面）；`grab.py` 里虽有一堆高度补偿参数（`height_gain_pulse` / `area_z_gain` / `ultra_weight` 等），但大多标 0、未标定，等于没生效。高度补偿方案本身还没定。
- **v2（search_v2/grab_v2）已判定不如 v1 并删除**。最近一次提交后又有未提交改动：grab.py / search.py / robot_params.yaml / auto_fetch.py 已改，grab_official.py / tracker.py / tasks/ceshi/ / YOLO/ 为新增。
- **工作重点在夹取算法与高度估计**。不要擅自动 search.py（寻路已好）。配合时：跑测试、读/解释日志、调 robot_params.yaml 参数、分析高度补偿方案。
- 用户反复强调的算法原则（别违背）：
  - 不硬编码距离/步数/固定夹取点；要闭环、按实时画面自动调整。
  - 检测到颜色就锁定，不转圈；不居中/不正就主动调整。
  - 夹取前先看一眼距离；目标略超可及范围就前进微调，不轻易放弃定点夹取。
  - 高处目标要有保底（体态升降 + 抬头检测）。
  - 官方 `block_fetch.py` / `intelligent_fetch.py` / `color_track.py` 是可靠参考。
- 机器人是 STA 局域网模式，**IP 会变**；连不上先确认 IP。摄像头画面远程看不到，需要现场确认（夹没夹到、画面）时让用户现场看。
- 机器人类似的坑点/历史/下一步详见 `进度清单.md`（新会话先读它对齐现状）。
