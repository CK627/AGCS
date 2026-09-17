# 地面站中枢（GroundStation/Dashboard）

四个仪表盘的中枢，**纯接收方**——唯一能「发」的是给地面机器人下发任务，其余都是接收，不自己检测/广播。

## 端口

| 端口 | 用途 |
|---|---|
| **20000** | 中枢网页 |

## 数据来源（全部接收）

| 数据 | 来源 | 中枢路由 |
|---|---|---|
| 无人机遥测 | Dron `/api/telemetry`（20002） | `/api/drone/status` |
| 无人机纯图传 | Dron `/video_feed`（**20005**） | `/drone_video_feed` |
| YOLO 检测流 | YOLO `/video_feed`（20003，已标注） | `/video.mjpeg` |
| YOLO 检测统计 | YOLO `/api/detections`（20003） | `/api/model/status` |
| 机器人画面 | task_server `/video.mjpeg`（5000） | `/robot_video_feed` |
| 机器人状态 | task_server `/status`（5000） | `/api/robot/status` |

## 配置（data/config.yaml）

| 字段 | 说明 |
|---|---|
| `drone.url` | 无人机仪表盘地址（默认 20002） |
| `drone.video_port` | 无人机图传视频流端口（默认 **20005**） |
| `robot.url` | 机器人 task_server 地址（默认 5000，IP 会变需核对） |
| `yolo.url` | YOLO 仪表盘地址（默认 20003） |
| `drone.video` / `yolo.video` / `robot.video` | 各视频开关 |
| `dashboard.hub_port` | 中枢端口（默认 20000） |

## 页面布局

- 第一行：`YOLO 模型检测流`（左）+ `YOLO 模型状态`（右）
- 第二行：`无人机画面（纯图传）`（左）+ `机器人画面`（右）

## 端口总览

| 端口 | 归属 |
|---|---|
| 20000 / 20001 / 20002 / 20003 | 中枢 / 机器人 / 无人机 / YOLO 仪表盘 |
| 20004 | 仪表盘自动发现广播（UDP） |
| 20005 | 无人机图传视频流转发 |
| 5000 | 地面机器人 task_server（HTTP） |
| 14550 | MAVLink 数传（UDP） |
| 8080 | QGC 数传（UDP） |
