# CompetitionUse —— 比赛流程脚本

机器人在比赛里要演示的步骤做成独立脚本，集中在本目录。各脚本独立运行，
通过 `task_server` 上报 `/status`（状态 + 位置 + 朝向 + 已抓取 + 任务 + 结果 + 消息），
地面站中枢轮询该接口、按状态关键词自动推进进度（机器人不主动通知中枢）。
其中 2.4 合作讨论模型为团队讨论环节，无代码。

## 步骤与脚本

| 步骤 | 脚本 | 作用 |
|------|------|------|
| 2.1 视觉追踪 | `VisualTracking.py` | 云台 21/24 PID 跟随色块 + 深度估位置 |
| 2.2 自动寻路 | `AutoPathfinding.py` | 扫描找目标 → 追踪居中 → 转身对准 → 小步逼近 → 深度判距到位 |
| 2.3 自主抓取 | `AutonomousCrawling.py` | 固定路线夹取 → 左转 → 人脸识别 → 递物放手 |
| 2.4 合作讨论模型 | （无代码） | 团队讨论环节，机器人无动作 |
| 2.5 自动捕获 | `Auto-capture.py` | 读固定路线 json + IMU 航向 + 颜色微调 + 固定脉宽夹取/放下 |
| 2.5 自动捕获（进阶） | `Auto-capture-1.py` | 阶段一沿用 2.5 导航，阶段二 YOLO 检测 + 雅可比精对准 |

## 辅助脚本

| 脚本 | 作用 |
|------|------|
| `_common.py` | 2.1~2.5 公共初始化（board / IK / 相机 / 检测闭包） |
| `fixed_route.json` | 2.5 固定路线动作序列（前进/转弯/夹取/放下脉宽） |
| `depth_3d_grasp.py` | 方案 A：深度 3D 抓取（颜色找目标 → 深度测 3D → IK 夹取） |
| `calib_cam2arm.py` | 手眼标定（一次性），产出 `config/cam2arm.yaml` |

## 运行

先停掉自启服务（抢串口 `/dev/ttyAMA0`），再跑对应脚本：

```bash
ssh pi@<机器人IP>
sudo systemctl stop spiderpi

cd /home/pi/spiderpi/CompetitionUse
python3 VisualTracking.py --color yellow                     # 2.1
python3 AutoPathfinding.py --color yellow                    # 2.2
python3 AutonomousCrawling.py                                # 2.3
python3 Auto-capture.py --color red                          # 2.5 正常版
python3 Auto-capture-1.py --model models/fake_bug.onnx --color blue   # 2.5 进阶版(YOLO)
```

## 说明

- `VisualTracking.py` / `AutoPathfinding.py` / `AutonomousCrawling.py` 完全独立，
  只依赖官方 SDK（`common`）；`Auto-capture*.py` 依赖上级 `agcs_lib` 与
  `communication/task_server`。
- 2.5 正常版结束上报 `state='END'`（停留 5 秒）+ `last_result='done'`；进阶版
  在夹取点用 YOLO 模型检测 + 雅可比矩阵把像素误差换算成 21-24 号舵机增量做精对准。
- `depth_3d_grasp.py` / `calib_cam2arm.py` 是「方案 A（深度 3D）」的独立验证，
  先用 `calib_cam2arm.py` 标定一次，再跑 `depth_3d_grasp.py`；标定和抓取时
  云台 21/24 位姿要保持一致。
