# scripts 管理脚本（只服务 GroundStation/Dashboard）

启动、停止、重启、查看状态、安装、更新、卸载地面站仪表盘。

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

常用命令：`start [端口]`、`stop`、`restart [端口]`、`status`、`install`、`update`、`uninstall`、`help`。
默认端口 `20001`。
