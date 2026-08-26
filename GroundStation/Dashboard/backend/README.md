# 地面站仪表盘

显示无人机、地面机器人、YOLO 模型三类信息，并支持从网页直接给机器人下发任务。
全部走 HTTP 服务，不依赖图形界面。

## 目录

- `backend/`：Flask 后端（本目录）
- `frontend/`：网页前端
- `scripts/`：三平台快速启动脚本

## 启动

先在 `GroundStation/Dashboard` 目录安装依赖：

```bash
python -m pip install -r backend/requirements.txt
```

再启动后端：

```bash
cd backend
python app.py
```

浏览器打开 `http://localhost:20001`（或局域网内 `http://地面站IP:20001`）。

也可以直接用快速启动脚本：

- macOS：`bash GroundStation/Dashboard/scripts/macOS/start.sh`
- Linux：`bash GroundStation/Dashboard/scripts/Linux/start.sh`
- Windows：`GroundStation\Dashboard\scripts\Windows\start.bat`

脚本支持 `start / stop / restart / status / install / update / uninstall / help`
等命令，例如 `bash GroundStation/Dashboard/scripts/macOS/start.sh start`。

服务优先用 waitress 启动；未安装 waitress 时自动退回 Flask 开发服务器。

依赖：

```bat
python -m pip install flask waitress requests pymavlink
```

（视频预览需要 opencv-python，YOLO 检测需要 ultralytics，已随 YOLO 环境装好；
无人机数据由 pymavlink 直接读 MAVLink，**不需要 ROS/MAVROS**。）

## 配置

改 `config.py`：

| 配置项 | 说明 |
|--------|------|
| `ROBOT_URL` | 机器人任务服务地址（当前 `http://10.194.228.87:5000`） |
| `DRONE_MAVLINK` | pymavlink 数传连接串（默认 `udpin:0.0.0.0:14550`） |
| `DRONE_RTSP` | 无人机图传 RTSP 地址 |
| `DASHBOARD_HOST` / `DASHBOARD_PORT` | 仪表盘监听地址/端口（默认 `0.0.0.0:20001`） |
| `MODEL_INFO` | YOLO 模型路径、类别、置信度阈值 |

## 接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 仪表盘页面 |
| `/api/drone/status` | GET | 无人机状态（pymavlink 读 MAVLink 缓存） |
| `/api/robot/status` | GET | 机器人状态（代理树莓派 `/status`） |
| `/api/model/status` | GET | YOLO 模型信息与检测统计 |
| `/api/robot/task` | POST | 把任务转发给机器人（JSON：`target.x/y`、`arrive_radius`） |
| `/video.mjpeg` | GET | 无人机画面 MJPEG 预览（YOLO 画框） |

## 前置条件

1. 机器人：已切成 STA 局域网模式，`autonomous_pick.py` 运行中（自动开启
   HTTP 服务，监听 5000）。
2. 无人机：数传在线（QGC 能连上），pymavlink 监听的 14550 端口能收到 MAVLink，
   图传 RTSP 可达。
3. 网络：地面站能 `ping` 通机器人；仪表盘端口如需跨网段访问，按
   [YOLO教学与空地协同.md](../../../YOLO教学与空地协同.md) 7.8 做端口转发。

## 常见问题

| 现象 | 排查 |
|------|------|
| 机器人卡片显示离线 | `ping 10.194.228.87`；确认 `autonomous_pick.py` 已启动、Flask 已装 |
| 无人机卡片显示未接入 | 确认数传在线、QGC 能连上；确认防火墙放行 UDP 14550；数传端把地面站 IP+14550 加为目标 |
| 画面显示 no video | 确认 RTSP 地址可达（VLC 能打开）；摄像头/数传在线 |
| 模型未加载 | 检查 `MODEL_INFO['path']` 是否存在、模型是否训练完成 |
