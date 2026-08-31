# 地面站仪表盘

显示无人机、地面机器人、YOLO 模型三类信息。
全部走 HTTP 服务，不依赖图形界面。

## 数据流

```text
机器人：树莓派 autonomous_pick.py / CS-video.py（每帧 publish_frame 压缩 JPEG）
  → task_server.py /video.mjpeg（5000 端口）
  → 本后端 /robot_video_feed（代理转发）+ /api/robot/status（状态代理）
  → 浏览器实时显示机器人画面与参数

无人机：数传 MAVLink → pymavlink 后台线程 → /api/drone/status
图传：EWRF 接收机（USB 摄像头，Dron/Dashboard 独占采集，20000 端口）
  → 本后端拉流 /video_feed（可叠 YOLO 画框）→ /video.mjpeg + /api/model/status
```

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

**临时配置（机器人 IP / 无人机地址 / 视频开关 / 端口）统一改本仪表盘的
[data/config.yaml](../data/config.yaml)，保存即热重载生效，无需重启**
（端口除外，改端口需重启对应仪表盘）。

`config.py` 只放默认值和硬件绑定项（无人机数传/图传地址、YOLO 模型路径）：

| 配置项 | 说明 |
|--------|------|
| `DRONE_MAVLINK` | pymavlink 数传连接串（默认 `udpin:0.0.0.0:14550`） |
| `DRONE_URL` | 无人机电脑端仪表盘地址（走本仪表盘 data/config.yaml `drone.url`；中枢从这里拉图传画面，不在同一台电脑时填那台电脑的 IP） |
| `DASHBOARD_HOST` | 监听地址（默认 `0.0.0.0`；端口走本仪表盘 data/config.yaml `dashboard.hub_port`） |
| `MODEL_INFO` | YOLO 模型路径、类别、置信度阈值 |
| `MODEL_INFO` | YOLO 模型路径、类别、置信度阈值 |

## 接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 仪表盘页面 |
| `/api/drone/status` | GET | 无人机状态（pymavlink 读 MAVLink 缓存） |
| `/api/robot/status` | GET | 机器人状态（代理树莓派 `/status`） |
| `/api/model/status` | GET | YOLO 模型信息与检测统计 |
| `/robot_video_feed` | GET | 机器人摄像头画面 MJPEG（代理树莓派 `/video.mjpeg`） |
| `/video.mjpeg` | GET | 无人机图传画面 MJPEG（拉 Dron/Dashboard 的流，可叠 YOLO 画框；未装 YOLO 时原样转发） |

## 无人机自动发现

地面站启动后会监听 UDP 20003，自动接收无人机电脑端广播的地址
（无人机仪表盘每 2 秒广播一次，见 `Dorn/Dashboard/backend/README.md`）；
`drone.url` 没写或还是 127.0.0.1 时，自动使用广播发现到的地址。
注意：广播只在同一局域网内有效，跨网段需在 `data/config.yaml` 手动填地址。

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
| 机器人卡片显示离线 | `ping 10.194.228.89`；确认 `autonomous_pick.py` / `CS-video.py` 已启动、Flask 已装 |
| 机器人画面不显示 | 确认机器人端在跑 `autonomous_pick.py` 或 `CS-video.py`（推流源）；浏览器直开 `http://机器人IP:5000/video.mjpeg` 可单独验证机器人端 |
| 无人机卡片显示未接入 | 确认数传在线、QGC 能连上；确认防火墙放行 UDP 14550；数传端把地面站 IP+14550 加为目标 |
| 无人机画面显示 no video / 状态卡 | 确认无人机电脑端 Dorn/Dashboard 已运行（20000）；EWRF 接收机已插 USB 且 `drone.camera` 序号正确（`capture_external_camera.py --list` 查）；中枢和它不在同一台电脑时改 data/config.yaml `drone.url` |
| 模型未加载 | 未装 ultralytics 或 `MODEL_INFO['path']` 无效时自动退化为原始画面转发，不影响看画面；要画框就装好环境并改对模型路径 |
