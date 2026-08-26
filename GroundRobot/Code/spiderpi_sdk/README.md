# spiderpi_sdk（SDK 参考副本）

本目录对应树莓派上的 `~/spiderpi/spiderpi_sdk/`，即官方 SDK 源码包：

| 子目录 | 安装包 | 用途 |
|--------|--------|------|
| `common_sdk` | `common` | 控制板通信（`ros_robot_controller_sdk`）、六足运动学（`kinematics`）、动作组、yaml 工具 |
| `arm_ik_sdk` | `arm_ik` | 机械臂逆运动学（`arm_move_ik.ArmIK`） |
| `sensor_sdk` | `sensor` | 超声波、TTS、语音识别等传感器 |
| `camera_calibration_sdk` | `calibration` | 相机类与标定参数路径 |

开发时**不要直接修改**这里：SDK 以安装包形式存在于树莓派环境（`pip` 安装的
`common` / `arm_ik` / `sensor` / `calibration`），运行时会优先加载已安装版本。

本目录用于：

- 同步开发环境：执行 `./pull_from_robot.sh` 即可把树莓派上的 SDK 拉取到这里，
  保证本地看到的文件列表与机器人一致；
- 查阅 API：例如 `common_sdk/common/kinematics_control_demo.py` 展示了六足
  `ik.go_forward / ik.turn_left / ik.stand` 的用法。

我们的业务代码位于 `functions/`、`advanced/`、`kinematic_routines/`，
通过 `sync_to_robot.sh` 同步到机器人对应目录。
