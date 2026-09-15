# 地面机器人仪表盘（GroundRobot/Dashboard）

地面机器人独立仪表盘：摄像头画面回传 + 实时状态 + 下发任务。

## 端口

| 端口 | 用途 |
|---|---|
| **20001** | 机器人独立仪表盘网页 |

## 配置（data/config.yaml）

| 字段 | 说明 |
|---|---|
| `robot.url` | 机器人 task_server 地址（默认 5000，IP 会变需核对） |
| `robot.video` | 机器人画面开关 |
| `dashboard.robot_port` | 仪表盘端口（默认 20001） |

## 数据流

```
机器人 task_server（5000）: /status  /task  /video.mjpeg
  → 本后端代理转发
  → 网页显示实时状态 + 摄像头画面 + 下发任务
```
