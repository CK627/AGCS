# CompetitionUse —— 比赛流程脚本

本目录放的是机器人在比赛里要演示的 **2.1~2.5 各步骤**，每个步骤做成一个**独立脚本**，
各自运行、互不依赖。它们通过 HTTP 服务（端口 5000）上报 `/status`，地面站中枢轮询该接口、
按状态关键词自动推进进度（机器人不主动通知中枢）。2.4 是团队讨论环节，无代码。

> 运行前提（串口 `/dev/ttyAMA0` 同一时刻只能一个进程占用）：
> ```bash
> ssh pi@<机器人IP>
> sudo systemctl stop spiderpi        # 停自启服务；joystick 若在跑也要停
> cd /home/pi/spiderpi/CompetitionUse
> ```

## 文件清单

| 文件 | 是什么 | 依赖 |
|------|--------|------|
| `VisualTracking.py` | **2.1 视觉追踪**：云台 21/24 PID 跟随色块 + 深度相机估位置 | 仅官方 SDK，完全独立 |
| `AutoPathfinding.py` | **2.2 自动寻路**：扫描找目标 → 追踪居中 → 转身对准 → 小步逼近 → 深度判距到位 | 仅官方 SDK，完全独立 |
| `AutonomousCrawling.py` | **2.3 自主抓取**：固定路线夹取 → 左转 → 人脸识别(Haar) → 递物放手 | 仅官方 SDK，完全独立 |
| `Auto-capture.py` | **2.5 自动捕获（NO6，基础版）**：读固定路线 JSON + IMU 航向保持 + 颜色微调 + 固定脉宽夹取/放下 | `agcs_lib` + `communication/task_server` |
| `Auto-capture-1.py` | **2.5 自动捕获（NO7，进阶版）**：NO6 全套 + 夹取前 YOLO 检测 + 雅可比精对准 | 同上 + `calib_pick*.json`、`models/best.onnx` |
| `1.py` | **临时测试**：NO6 寻路 + YOLO 夹取（不读 JSON 的 pick，place 仍读 JSON） | 用 importlib 加载 `Auto-capture-1.py`，猴补丁 `do_pick` |
| `fixed_route.json` | **2.5 固定路线动作序列**（forward/back/turn_left/pick/place 的步长、角度、夹取脉宽）。现场调路线改这里，不改代码 | 被 NO6/NO7/1.py 读取 |
| `calib_cam2arm.py` | **手眼标定（一次性）**：求彩色相机 → 机械臂坐标系的 R/t，写 `config/cam2arm.yaml` | `agcs_lib.marker` + `agcs_lib.camera` |

## 运行

```bash
python3 VisualTracking.py --color yellow                     # 2.1
python3 AutoPathfinding.py --color yellow                    # 2.2
python3 AutonomousCrawling.py                                # 2.3
python3 Auto-capture.py --color red                          # 2.5 基础版(NO6)
python3 Auto-capture-1.py --model models/best.onnx --color blue   # 2.5 进阶版(NO7)
python3 1.py --color red                                     # 临时：NO6寻路 + YOLO夹取
```

## 关于「建图 / 定位」的说明（当前已知问题）

**结论：现阶段不依赖建图。** 2.1~2.5 的脚本**都不需要预先建图**，它们用的是：

- 直线走偏 → 靠 **IMU 航向保持**（`agcs_lib/imu.py` 后台线程连续积分）＋**颜色块左右微调**闭环修正；
- 转弯转少 → 靠 IMU 测转角、转完补一次 + 每转完一段就 `reset_imu()` 归零，误差不跨段传播；
- 到没到位 → 靠**深度相机**（奥比中光 Astra Pro）直接测距，不做全局地图。

原先设想的一条「深度相机当 2D 激光雷达 → 扫描建图 → ICP 定位校正航向」的路子（涉及
`agcs_lib/depthscan.py`、`pcl.py`、`localize.py`、`mapview.py` 以及已删除的
`tasks/CS/scan_2d.py` 等建图脚本），在当前条件下**不可用**，原因：

1. **没有激光雷达**，只能拿深度相机模拟，视场窄、范围小；
2. **周围环境驳杂**，扫描点云里墙/杂物/花盆混杂，ICP 配准容易跑飞，建出的图用不了；
3. 建图方式受限于「原地转身拍多视角」这一种，自由度很低。

目前这条链路**没有接入任何比赛脚本的主流程**，只残留在 `Auto-capture-1.py`（NO7）里作为
一段**可选**代码：只有当 `models/map.npz` 存在时才尝试用 ICP 校正航向，文件不存在就静默跳过，
完全不影响 NO7 正常跑。也就是说——**要不要建图，比赛流程都能跑**。

> 若后续想复活建图/定位：建图脚本已随 2026-09-15 的非比赛文件清理删除，可从 git 取回
> （`git show 9ef5f09^:./Code/tasks/CS/CS-build-map.py`）；但按当前环境，建议先把
> 「IMU + 颜色 + 深度」这套闭环调稳，再考虑是否值得上全局定位。
