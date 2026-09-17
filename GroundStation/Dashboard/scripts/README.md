# scripts 管理脚本（服务 GroundStation/Dashboard）

一套命令管理**两个服务**（一起启停 / 重启）：

| 服务 | 端口 | 说明 |
|------|------|------|
| 中枢仪表盘 | 20000 | 数据采集 + 四个面板（`backend/app.py`） |
| 研发进度流程图 | 20010 | 独立网页，进度数据从中枢取（`progress/app.py`） |

启动、停止、重启、查看状态、安装、更新、卸载。

## macOS

```bash
bash GroundStation/Dashboard/scripts/macOS/start.sh start
bash GroundStation/Dashboard/scripts/macOS/start.sh status
bash GroundStation/Dashboard/scripts/macOS/start.sh stop
```

## Linux

```bash
bash GroundStation/Dashboard/scripts/Linux/start.sh start
bash GroundStation/Dashboard/scripts/Linux/start.sh status
bash GroundStation/Dashboard/scripts/Linux/start.sh stop
```

## Windows

```bat
GroundStation\Dashboard\scripts\Windows\start.bat start
GroundStation\Dashboard\scripts\Windows\start.bat status
GroundStation\Dashboard\scripts\Windows\start.bat stop
```

常用命令：`start [中枢端口] [进度页端口]`、`stop`、`restart [中枢端口] [进度页端口]`、
`status`、`install`、`update`、`uninstall`、`help`。
默认端口 `20000` / `20010`。

## 运行时文件

| 文件 | 内容 |
|------|------|
| `data/server.pid` `data/server.port` | 中枢进程号与端口 |
| `data/progress.pid` `data/progress.port` | 进度流程图进程号与端口 |
| `logs/nohup.log` | 中枢日志 |
| `logs/progress.log` | 进度流程图日志 |

> 停止/重启时会先按 PID 文件杀，再按端口兜底（`lsof` / `netstat`），
> 所以 PID 文件丢了也能停干净。
