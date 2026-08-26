# GroundRobot Code —— 自主行走 + 视觉识别 + 机械臂抓取

本目录存放 SpiderPi Pro 的开发代码，目录结构与树莓派上的 `~/spiderpi` 保持
**一一对应**，写好后直接同步即可运行。

## 1. 目录结构与机器人文件列表对照

| 本地目录 | 机器人路径 | 内容 |
|----------|-----------|------|
| `config/` | `~/spiderpi/config/` | 行为参数 `robot_params.yaml` |
| `functions/` | `~/spiderpi/functions/` | 基础模块：视觉工具、颜色识别、避障行走、YOLO 检测 |
| `advanced/` | `~/spiderpi/advanced/` | 整合程序 `autonomous_pick.py` |
| `kinematic_routines/` | `~/spiderpi/kinematic_routines/` | 机械臂抓取封装 `arm_pick.py` |
| `communication/` | `~/spiderpi/communication/` | 地面站↔机器人通信：任务接收、状态上报 |
| `spiderpi_sdk/` | `~/spiderpi/spiderpi_sdk/` | SDK 参考副本（从机器人拉取，不修改） |

`action_groups/`、`SpiderPi.py` 等机器人原有文件不在本仓库维护，同步脚本不会触碰。

## 2. 文件说明

| 文件 | 作用 |
|------|------|
| `config/robot_params.yaml` | 所有可调参数（目标颜色、步幅、避障阈值、抓取坐标） |
| `functions/robot_config.py` | 读取参数文件 |
| `functions/vision_utils.py` | 颜色检测、轮廓提取、像素→机械臂坐标变换、畸变校正 |
| `functions/color_detect.py` | 颜色识别（可独立运行，含 RPC 兼容接口） |
| `functions/obstacle_avoidance.py` | 超声波自主避障行走 |
| `functions/yolo_detect.py` | YOLO 检测器（可选，替换颜色检测） |
| `kinematic_routines/arm_pick.py` | 机械臂逆运动学抓取封装（夹爪=舵机25） |
| `advanced/autonomous_pick.py` | **整合状态机**：收任务→导航→扫描→对准→接近→抓取；每帧回传画面 |
| `communication/task_server.py` | HTTP 服务：`POST /task` 收任务、`GET /status` 上报状态、`/video.mjpeg` 推流画面 |
| `communication/__init__.py` | 通信模块包标记 |
| `sync_to_robot.sh` | 把代码同步到树莓派 |
| `pull_from_robot.sh` | 把树莓派配置/SDK 拉回本地 |

## 2.1 教学文档（每个 py 文件配套）

每个 `.py` 文件旁都有一份 `*.md` 教学文档，讲解模块来源、代码逐段含义、
排查方法和动手练习。**运行前先读对应文档**，出问题时按文档里的排查表定位。

| 代码文件 | 教学文档 | 重点内容 |
|----------|----------|----------|
| `functions/robot_config.py` | [robot_config.md](functions/robot_config.md) | 路径引导、YAML、参数化思想 |
| `functions/vision_utils.py` | [vision_utils.md](functions/vision_utils.md) | LAB 阈值、轮廓、相机标定、坐标变换 |
| `functions/color_detect.py` | [color_detect.md](functions/color_detect.md) | 摄像头循环、RPC 接口、OpenCV 窗口 |
| `functions/obstacle_avoidance.py` | [obstacle_avoidance.md](functions/obstacle_avoidance.md) | 超声波滤波、六足运动学 API、避障决策 |
| `functions/yolo_detect.py` | [yolo_detect.md](functions/yolo_detect.md) | YOLO 输出结构、模型部署路线 |
| `kinematic_routines/arm_pick.py` | [arm_pick.md](kinematic_routines/arm_pick.md) | 逆运动学、夹爪舵机、工作空间 |
| `advanced/autonomous_pick.py` | [autonomous_pick.md](advanced/autonomous_pick.md) | 状态机、数据流、整合调试方法论 |

> 教学文档不会被同步到树莓派（`sync_to_robot.sh` 排除了 `*.md`），只保留在
> 本地仓库，方便查阅和复习。

## 3. 快速开始

### 3.1 同步到树莓派

```bash
cd GroundRobot/Code
./sync_to_robot.sh                    # 默认连 pi@192.168.149.1（AP 直连）
./sync_to_robot.sh pi@192.168.1.100   # 局域网模式下指定机器人 IP
```

> **Windows 地面站注意**：这两个脚本是 bash 写的，Windows 下用 **Git Bash**
> 或 **WSL** 运行；或者直接用资料包里的 **WinSCP** 把 `GroundRobot/Code/`
> 下的文件拖到树莓派 `~/spiderpi/` 对应目录，效果一样。

### 3.2 单项验证（在树莓派 VNC 终端，先关闭自启主程序）

```bash
sudo systemctl stop spiderpi

# 第一步：颜色识别
python3 ~/spiderpi/functions/color_detect.py --color red

# 第二步：避障行走（前方放障碍物测试）
python3 ~/spiderpi/functions/obstacle_avoidance.py
```

### 3.3 整合运行（自主行走 + 识别 + 抓取）

```bash
python3 ~/spiderpi/advanced/autonomous_pick.py --color red
python3 ~/spiderpi/advanced/autonomous_pick.py --detector yolo --model /home/pi/best.pt --classes damaged_pod
```

启动后主程序自动在后台开启 HTTP 服务（5000 端口）并进入 **NAV** 状态等待
地面站下发任务；不想等任务直接扫描可加 `--no-wait-task`。

程序状态机：

```
NAV ──(收到任务)──▶ SEARCH ──(发现目标)──▶ APPROACH ──(进入可及范围)──▶ PICK ──(完成)──▶ SEARCH
 ▲                     ▲                     │ 对准/接近                    │
 └──(无任务/等下一个)───┘                     └────(丢失/超范围)────┘        └── 后退一步
```

- NAV：等地面站 `POST /task`，收到后转向目标、避障前进，进入粗定位半径后切 SEARCH
- SEARCH：原地 15° 逐步左转扫描，一整圈没找到就前进一段继续找
- APPROACH：目标偏离画面中心就转向对准，对准后前进（超声波避障）
- PICK：目标面积达标后计算机械臂坐标，抓取、抬起、松开、复位
- ESC 退出时机械臂复位、机器人立正

下发任务示例（地面站）：

```bash
curl -X POST http://192.168.1.101:5000/task \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"T001","target":{"x":2.0,"y":0.0,"confidence":0.87},"arrive_radius":0.8}'
```

机器人状态上报（地面站查询）：

```bash
curl http://192.168.1.101:5000/status
```

## 4. 关键参数（`config/robot_params.yaml`）

| 参数 | 说明 | 建议调试顺序 |
|------|------|--------------|
| `vision.target_color` | 目标颜色 | 先用红/绿色块 |
| `vision.min_area` | 有效目标最小面积 | 调大过滤远处干扰 |
| `walk.approach_area` | 判定"够近可抓"的面积 | 配合机械臂可及范围调整 |
| `walk.reach_x / reach_y` | 机械臂可及范围（cm） | 参考官方 block_fetch 的 ±8cm / 24cm |
| `obstacle.threshold` | 避障距离（cm） | 视场地调整 |
| `arm.*` | 抓取高度、夹爪开合脉宽、抬起/放置位 | 先单独跑 `arm_pick` 校准 |

## 5. 与 README 开发路线的对应关系

- **自主行走**：`obstacle_avoidance.py`（第二步：六足行走 + 避障）
- **视觉识别**：`color_detect.py` + `vision_utils.py`（第三步：颜色阈值/位置校准；
  后期用 `yolo_detect.py` 替换，接口一致）
- **机械臂抓取**：`arm_pick.py`（第四步：官方 `block_fetch.py` 的重构封装）
- **整合**：`autonomous_pick.py`（"识别→移动→抓取"完整闭环）

YOLO 切换示例：

```bash
python3 ~/spiderpi/advanced/autonomous_pick.py \
  --detector yolo --model /home/pi/yolov8n.pt --classes pod
```

## 6. 注意事项

1. 运行任何玩法前先 `sudo systemctl stop spiderpi`，避免开机自启主程序抢占
   舵机/摄像头资源。
2. 像素→机械臂坐标依赖 `~/spiderpi/config/camera_cal.yaml` 的 `block_params`，
   首次使用前必须在机器人上完成位置校准（官方 `camera_cal_main.py` + 上位机），
   并用 `./pull_from_robot.sh` 拉回本地备份。
3. `functions/color_detect.py` 同步后会替换官方同名文件（我们的版本保留
   init/start/stop/exit/run 接口，可被 SpiderPi.py 加载）；官方原版在
   `资料/3 源码资料/SpiderPi_Pro.zip`。
4. 所有代码在树莓派（ARM Linux）上运行，本地仅做语法检查；联调时注意
   AP 模式无外网，`pip3 install` 需在 STA 局域网模式下进行。
5. 地面站是 Windows：任务下发/状态回传走 HTTP（`communication/task_server.py`），
   与系统无关；无人机数据用 pymavlink 读，见 Dron 教学文档。
