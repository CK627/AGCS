# Dron 教学文档索引

无人机部分的教学文档，按"从硬件到算法"的顺序组织。**运行前先读对应文档**，
出问题时按文档里的排查表定位。

| 文档 | 对应模块 | 重点内容 |
|------|----------|----------|
| [PX4与QGroundControl.md](PX4与QGroundControl.md) | 地面站/固件 | 飞控是什么、PX4 架构、QGC 功能、固件烧写、参数 |
| [遥控器与飞行模式.md](遥控器与飞行模式.md) | 遥控飞行 | 通道、混控、7 种飞行模式、解锁/急停 |
| [数传与图传通信.md](数传与图传通信.md) | 通信 | 数传/图传区别、Minihomer、WiFi HaLow、组网方式、RTSP |
| [传感器与定位.md](传感器与定位.md) | 感知 | IMU、卡尔曼滤波、GPS 原理、光流、RTK |
| [固件与参数.md](固件与参数.md) | 维护 | 固件烧写、参数保存/加载、三类校准 |
| [MAVLink与MAVROS.md](MAVLink与MAVROS.md) | 协同 | MAVLink 协议、pymavlink 读写数据（Windows 版，替代 MAVROS） |
| [机载视觉与视频流.md](机载视觉与视频流.md) | 视觉 | 摄像头规格、RTSP 拉流、GStreamer、YOLO 部署 |
| [无人机操控与自主飞行教学手册](../无人机操控与自主飞行教学手册.md) | 总手册 | 四阶段规划、电脑互通、Python 方案、报错总表 |
| [Code/README.md](../Code/README.md) | 代码 | 6 个 py 文件规划、运行顺序、安全红线 |
| [Python与无人机互通-库函数详解.md](Python与无人机互通-库函数详解.md) | 互通原理 | pymavlink 每个函数对应无人机的什么 |
| [sim_drone.md](../Code/sim_drone.md) | 无真机练习 | 假无人机模拟器，先跑通数据链路 |
| [backend_exercise.md](../Code/backend_exercise.md) | 后端训练 | 测试数据 → 约定格式 JSON 输出（网页由队友写） |

每个文档统一结构：**模块来源 → 关键概念教学 → 操作步骤 → 常见问题排查 →
动手练习**。

代码文件（`Code/`）每个 py 都配有教学文档：`drone_config.md`、`connect.md`、
`read_drone.md`、`view_data.md`、`plot_data.md`、`mission.md`，先读文档再运行。
