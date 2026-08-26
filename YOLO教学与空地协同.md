# YOLO 模型教学与空地协同引导文档

> 前置状态：地面站为 **Windows**（机械革命笔记本，RTX 5060 8G / 32G / 1TB），
> NVIDIA 驱动、Python + YOLO（torch cu128）、QGC、pymavlink 已按
> [GroundStation/开始配置.md](GroundStation/开始配置.md) 装好。不再使用
> Ubuntu / ROS：无人机数据用 pymavlink 读 MAVLink，机器人任务走 HTTP。
>
> 本文档以 [README.md](README.md) 的三阶段目标为基准，回答三个问题：
> **YOLO 怎么用、怎么测？数据怎么准备、模型怎么训练（预训练）？部署前怎么
> 验证（预测试）？无人机、地面站、地面机器人怎么连起来协同？**

---

## 0. 学习地图：这份文档怎么用

### 0.1 本文档与 README 的对应关系

| 本文档章节 | 对应 README 内容 | README 验收标准 |
|------------|------------------|-----------------|
| 第 3 章 模型使用与测试 | 2.3 第八步 YOLO 依赖、2.4 第一阶段验收 | 可导入 ultralytics，能运行示例推理 |
| 第 4 章 预训练：数据准备与训练 | 3.3 第三步"在地面站 PC 上训练 YOLO 模型识别豆荚螟危害特征" | 训练出可用的豆荚螟模型 |
| 第 5 章 预测试：部署前验证 | 七、风险表"YOLO 识别准确率低→扩充数据集" | 误检/漏检可量化、可反馈 |
| 第 6 章 模型导出与部署 | 3.3"轻量化部署到树莓派端（或 Wi-Fi 调用地面站 API）" | 机器人端可用 |
| 第 7 章 空地协同 | 4.3、4.4 完整协同流程 | 侦察→定位→抓捕全流程 <5 分钟，延迟 <200ms |

### 0.2 五步路线图

```
第1步 跑通通用 YOLO（第3章）        —— 谢啸负责，先把工具链跑顺
        │
        ▼
第2步 采集毛豆/豆荚螟数据（4.2节） —— 朱杨杰（无人机俯拍）+ 马晨轩（机器人近景）
        │
        ▼
第3步 标注 + 训练（4.3~4.5节）      —— 谢啸负责，在 GroundStation GPU 上训练
        │
        ▼
第4步 预测试（第5章）               —— 谢啸 + 朱杨杰 + 马晨轩 三方验证
        │
        ▼
第5步 部署 + 空地协同（第6、7章）   —— 全体：杨梦享定通信协议，谢啸搭链路
```

> 学习原则和 README 一致：**每完成一步、验收一步，再进下一步**。不要急着
> 直接训豆荚螟模型——先拿官方预训练模型把"图片→视频→RTSP 流"全流程跑通，
> 后面换模型时只有数据不同，代码不用大改。

### 0.3 谁负责什么（与 README 6.1 分工一致）

| 角色 | 姓名 | 自己的学习路线 | 在这份文档里的任务 |
|------|------|----------------|--------------------|
| 网络架构师 | 谢啸 | [GroundStation/学习路线.md](GroundStation/学习路线.md)（新建） | 第 3~6 章主线：模型使用、训练、导出、部署；第 7 章网络链路 |
| 空中巡护员 | 朱杨杰 | [Dron/学习路线.md](Dron/学习路线.md) | 无人机俯拍数据采集（4.2）；机载视频验证（5.2）；第 7 章无人机端联调 |
| 地面田卫士 | 马晨轩 | [GroundRobot/学习路线.md](GroundRobot/学习路线.md) | 机器人近景数据采集（4.2）；部署 `yolo_detect.py`；执行抓取任务 |
| 中枢调度员 | 杨梦享 | [Internet/学习路线.md](Internet/学习路线.md) | 第 7 章通信协议与任务调度；坐标消息格式；整链路计时验收 |
| 网络部门（IT） | 外部支持 | 不在此仓库 | 按 7.8/7.9 端口清单做端口转发、流信号转发与安全配置 |

### 0.4 与其他成员文档的关系

本文档是 YOLO 主线的主教学文档；设备层面的细节不在本文重复展开，按需跳转：

| 本文档章节 | 需要配合看的成员文档 | 为什么 |
|------------|----------------------|--------|
| 第 1 章 | [GroundStation/开始配置.md](GroundStation/开始配置.md) | 环境安装与 CUDA 排查的完整步骤 |
| 第 4 章 训练实操 | [GroundStation/YOLO训练教程.md](GroundStation/YOLO训练教程.md) | 从零训练：置信度/类别名概念 + 带注释代码 |
| 第 4 章 Mac 训练 | [MacYoLo/README.md](MacYoLo/README.md) | 本机 M2 Pro 的 macOS 训练方式（MPS） |
| 第 4.2 节 数据采集 | [Dron/学习路线.md](Dron/学习路线.md)、[GroundRobot/学习路线.md](GroundRobot/学习路线.md) | 朱杨杰/马晨轩各自的采集任务与验收 |
| 第 6 章 部署 | [GroundRobot/Code/README.md](GroundRobot/Code/README.md) | `yolo_detect.py`、`autonomous_pick.py` 的接口与同步脚本 |
| 第 7.2 节 视频流与位姿 | [Dron/教学/机载视觉与视频流.md](Dron/教学/机载视觉与视频流.md)、[Dron/教学/MAVLink与MAVROS.md](Dron/教学/MAVLink与MAVROS.md) | RTSP 地址、GStreamer、pymavlink 读写 MAVLink 的细节 |
| 第 7.3 节 机器人收任务 | [GroundRobot/开始配置.md](GroundRobot/开始配置.md) 第 7 节 | 机器人 STA 局域网模式切换是联调前置条件 |
| 第 7.11 节 仪表盘 | [GroundStation/dashboard/README.md](GroundStation/dashboard/README.md) | 仪表盘启动、配置、接口与排查 |

> 无人机型号说明：README 中称"F410"，官方手册名为 **F450 V6C**，两者是同一
> 平台（轴距同为 410mm），无人机文档均按官方名称编写，本文档沿用该口径。

---

## 1. 环境自检（10 分钟）

系统装好后先花十分钟确认环境是好的，**后面所有问题都不用到"环境"上猜**：

```bash
# 1) 显卡驱动
nvidia-smi

# 2) PyTorch 是否能用 GPU
python -c "import torch; print(torch.__version__, 'CUDA:', torch.cuda.is_available())"

# 3) ultralytics 和 OpenCV 版本
python -c "import ultralytics, cv2; print('ultralytics', ultralytics.__version__, '| cv2', cv2.__version__)"
```

期望输出：`CUDA: True`，且三个库都能正常打印版本。

> 本机硬件：**机械革命笔记本，RTX 5060 8G 显存 / 32G 内存 / 1TB 固态**。
> 这台机器的"环境成功标准"（`nvidia-smi` 识别 5060、torch 必须是 `+cu128` 版
> 等）已写在 [GroundStation/开始配置.md](GroundStation/开始配置.md)
> 第 3~4、10 节，按那份文档验收即可，本文档不再重复。核心提醒只有一条：
> **RTX 5060 是 Blackwell 架构（sm_120），torch 必须装 cu128 版，装错会报
> `sm_120 not compatible`**。

建一个统一的工作目录（后面所有数据集、模型、结果都放这里，避免散落）：

```bash
mkdir D:\yolo\datasets D:\yolo\runs D:\yolo\weights D:\yolo\scripts
```

> 建议：训练和数据操作前，在 Windows 里**创建系统还原点**（控制面板 → 系统 →
> 系统保护 → 创建），万一环境弄坏了可以回滚。

**验收**：`CUDA: True` 且无 `sm_120` 警告（详细标准见开始配置.md 第 10 节），
目录建好，能在终端说出来三行命令各自在查什么。

---

## 2. 概念课：YOLO 是什么（15 分钟）

YOLO 是一个**目标检测**模型：输入一张图片，输出图中每个目标的**类别**和
**位置框**。它同时完成"这是什么"（分类）和"在哪"（定位）两件事。

### 2.1 输入与输出

```text
输入：一张图（如 640×640）
        │
        ▼
YOLO 模型
        │
        ▼
输出：N 个检测框，每个框带
  - xyxy：左上角/右下角像素坐标（x1, y1, x2, y2）
  - conf：置信度 0~1（模型认为"这里确实是目标"的概率）
  - cls ：类别编号（对应类别名，如 0=person）
```

同一目标可能被框多次，模型内部会用 NMS（非极大值抑制）合并重复框，最后
每个目标只留一个框。

### 2.2 模型规格怎么选

| 模型 | 参数量（约） | 速度 | 精度 | 适合场景 |
|------|--------------|------|------|----------|
| YOLOv8n | 3.2M | 最快 | 最低 | 树莓派实时推理、低延迟 |
| YOLOv8s | 11.2M | 快 | 中 | 地面站实时检测 |
| YOLOv8m | 25.9M | 中 | 较高 | 地面站 |
| YOLOv8l / x | >40M | 慢 | 最高 | 离线/高精度分析 |

（数值以 ultralytics 官方文档为准，不同版本略有差异。）

**本项目选型结论**：

- 地面站训练和实时检测：`yolov8n` 或 `yolov8s`（GPU 上很快，足够用）
- 树莓派部署：**必须** `yolov8n` + 小 `imgsz`（树莓派没有 GPU）
- 通用预训练权重 `yolov8n.pt`：在 COCO 数据集（80 类常见物体）上训练好的
  权重，直接可用来"跑通流程"，也是我们训练豆荚螟模型的**起点**

### 2.3 与项目进度的关系

```
现在：用 yolov8n.pt（COCO 预训练）跑通代码流程
中期：收集毛豆/豆荚螟数据，在地面站微调训练出 best.pt
后期：导出轻量化模型部署到树莓派，或由地面站推理后下发坐标
```

---

## 3. 模型使用与测试

目标：把"推理"这件事完全玩熟——图片、视频、摄像头、RTSP 流都能跑，能测
速度，能读懂输出。对应 README 第一阶段验收。

### 3.1 命令行快速测试

```bash
# 单张图片（自动下载 yolov8n.pt，保存标注结果到 runs/detect/predict）
yolo predict model=yolov8n.pt source=D:/yolo/test.jpg save=True

# 视频
yolo predict model=yolov8n.pt source=D:/yolo/test.mp4 save=True

# USB 摄像头（实时窗口，按 q 退出）
yolo predict model=yolov8n.pt source=0 show=True

# 网络视频流（无人机图传的 RTSP 地址，见 Dron 教学文档）
yolo predict model=yolov8n.pt source='rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=1.sdp?' save=True
```

用官方测试图先跑一次：

```bash
python -c "from ultralytics import YOLO; m=YOLO('yolov8n.pt'); m.predict('https://ultralytics.com/images/bus.jpg', save=True, device=0)"
```

> 国内网络下自动下载权重可能很慢或失败，可提前手动下载
> `https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt`
> 放到 `D:\yolo\weights\`，之后用 `model=D:\yolo\weights\yolov8n.pt` 指定。

### 3.2 Python API：读懂输出结构

写一个最小推理脚本 `D:\yolo\scripts\predict_one.py`：

```python
from ultralytics import YOLO

model = YOLO('yolov8n.pt')              # 加载模型（首次会自动下载）
results = model.predict('test.jpg', conf=0.45, device=0, verbose=False)[0]

print('类别表:', results.names)          # {0: 'person', 1: 'bicycle', ...}
print('检测框数:', len(results.boxes))

for box in results.boxes:
    cls_id = int(box.cls[0])            # 类别编号
    conf   = float(box.conf[0])         # 置信度
    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]   # 像素坐标
    print(f'{results.names[cls_id]:12s} conf={conf:.2f} box=({x1},{y1})-({x2},{y2})')
```

运行：

```bash
cd /d D:\yolo && python scripts\predict_one.py
```

三个字段（`cls`、`conf`、`xyxy`）就是整个项目里 YOLO 的全部"输出接口"，
后续训练、部署、协同都用它们。

### 3.3 四种输入源的测试顺序

| 顺序 | 输入源 | 意义 | 测什么 |
|------|--------|------|--------|
| 1 | 图片 | 单帧 | 模型能不能用、类别名对不对 |
| 2 | 视频 | 连续帧 | 稳定性、FPS |
| 3 | USB 摄像头 | 机器人近景 | 现场实时性、画面差异 |
| 4 | RTSP 流 | 无人机图传 | 协同链路最关键的输入 |

测试 RTSP 实时检测（带 FPS 显示）：

```python
import time
import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
cap = cv2.VideoCapture('rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=1.sdp?')

while True:
    t0 = time.time()
    ok, frame = cap.read()
    if not ok:
        break
    result = model.predict(frame, imgsz=640, conf=0.45, verbose=False, device=0)[0]
    annotated = result.plot()                        # 画好框的图像
    fps = 1.0 / (time.time() - t0)
    cv2.putText(annotated, f'FPS {fps:.1f}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('drone_yolo', annotated)
    if cv2.waitKey(1) == 27:                          # ESC 退出
        break

cap.release()
cv2.destroyAllWindows()
```

### 3.4 性能测试：GPU vs CPU

```bash
# GPU（应远快于 CPU）
yolo benchmark model=yolov8n.pt device=0

# CPU（模拟树莓派端，会慢很多）
yolo benchmark model=yolov8n.pt device=cpu
```

记录三个数字，后面部署要对比：

| 指标 | GPU 值 | CPU 值 | 说明 |
|------|--------|--------|------|
| FPS | 60+（5060 上 n 模型通常 100+） | 5~15 | 每秒处理帧数 |
| 单帧延迟 | <20ms（5060 通常 <10ms） | 60~200ms | 从进模型到出结果 |
| imgsz 影响 | 640→480 更快 | 同样更快 | 小图更快、小目标更易漏 |

> README 第三阶段要求视频流延迟 <200ms：**网络传输 + 编码**占大头，检测本身
> 在地面站 GPU 上只占很小一部分，所以检测一定要放地面站，不要在树莓派上跑大模型。

### 3.5 与机器人代码的接口契约（重要）

项目里 `GroundRobot/Code/functions/yolo_detect.py` 已经把 YOLO 封装成和颜色
检测**同一种返回结构**：

```python
{'center': (x, y), 'area': int, 'color': 类别名, 'conf': float}
```

上层 `autonomous_pick.py` 只认 `center` 和 `area` 两个字段，所以训练好的模型
换上去后主程序不用改：

```bash
# 在树莓派上，把检测器从颜色换成 YOLO
python3 ~/spiderpi/advanced/autonomous_pick.py \
  --detector yolo --model /home/pi/yolov8n.pt --classes damaged_pod
```

`--classes` 用逗号分隔，多个类别如 `--classes damaged_pod,healthy_pod`，**类别名
必须和训练时 data.yaml 里的 names 完全一致**，这是最常见的部署错误之一。

### 3.6 第一阶段验收清单

| 检查项 | 验收标准 |
|--------|----------|
| 图片推理 | 保存的结果图能画出框和标签 |
| 视频/摄像头 | 能连续检测，程序稳定不闪退 |
| RTSP 流 | 能拉无人机图传并检测（可先拿普通网络摄像头模拟） |
| 性能 | 能说出 GPU 上的 FPS 和单帧延迟 |
| 接口 | 理解 `xyxy / conf / cls / names` 四个字段的含义 |

---

## 4. 预训练：数据准备与训练

> 这里的"预训练"有两层意思，都在这章解决：
> 1. **用 COCO 预训练权重做起点**（迁移学习）：`model=yolov8n.pt` 就是让模型
>    从"见过 80 类常见物体"的起点，去学我们的小数据集，而不是从零开始；
> 2. **整体上指部署前的训练环节**：数据准备 → 标注 → 训练 → 产出 `best.pt`。

### 4.1 任务定义与类别设计

README 1.2 的核心目标：识别**豆荚螟危害特征**——豆荚表面的蛀孔、变色、异常
形态，引导机器人精准摘除受害豆荚。

类别设计建议（**先少后多**）：

| 方案 | 类别 | 适用时机 |
|------|------|----------|
| 起步（推荐） | `damaged_pod` 受害豆荚（单类） | 先跑通"检测→下发→抓取"闭环 |
| 数据够 500 张后 | `damaged_pod` + `healthy_pod` | 健康豆荚当负样本，抑制误检 |
| 数据充足 | 细分 `borer_hole` / `discolored_pod` / `healthy_pod` | 需要区分危害类型时 |

要点：

- **类别编号一旦确定就不要随意改**。YOLO 标注文件里存的是数字，数字和 names
  的对应关系写死在 data.yaml 里，训练和部署必须用同一份 names。
- 机器人只关心"这棵豆荚要不要摘"，所以单类起步完全够用；类别太多反而会让
  小数据集更难收敛。

### 4.2 数据采集（与朱杨杰、马晨轩的协作）

采集来源（README 7 风险表的应对）：

| 来源 | 谁 | 拍什么 | 要求 |
|------|-----|--------|------|
| 无人机俯拍 | 朱杨杰 | 不同高度（3~10m）、角度、航线下拍的豆荚田 | 和实际巡检视角一致 |
| 机器人近景 | 马晨轩 | 离豆荚 10~40cm 的清晰近景 | 和机械臂抓取视角一致 |
| 手机补充 | 全员 | 蛀孔/变色豆荚特写 | 补充细节特征 |

拍摄清单（保证多样性，否则现场效果差）：

- 不同光照：晴天/阴天、早中晚、顺光/逆光
- 不同遮挡：豆荚被叶子挡住一半、重叠、模糊
- 不同距离和角度：俯视、侧视、斜视
- **负样本**：没有目标的田垄、叶片、土地画面（用来压误检）

数量建议：每类 ≥300 张起步，宁少勿滥，**标注质量 > 数量**。

> 前期可以先从百度图片批量爬取"大青虫"等关键词做初筛素材（脚本在
> [Crawler/](../Crawler/)），爬完**必须人工清洗**（删错图/水印图/卡通图）再标注。

视频抽帧脚本 `~/yolo/scripts/extract_frames.py`：

```python
import cv2, os

os.makedirs('raw_frames', exist_ok=True)
cap = cv2.VideoCapture('drone_01.mp4')
n = 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    if n % 15 == 0:                    # 每 15 帧取一张，避免相邻帧太相似
        cv2.imwrite(f'raw_frames/{n:06d}.jpg', frame)
    n += 1
cap.release()
print('共扫描', n, '帧')
```

> 无人机视频里连续帧几乎一样，全存下来会让训练集"虚胖"且容易过拟合；
> 每隔若干帧抽一张即可。

### 4.3 数据整理与标注

标注工具任选其一：

| 工具 | 说明 |
|------|------|
| [LabelImg](https://github.com/HumanSignal/labelImg) | 本地桌面工具，导出 YOLO 格式 |
| CVAT | 网页版，多人协作标注 |
| Roboflow | 在线标注 + 数据集管理（免费额度够起步） |

YOLO 标注格式：每张图对应一个 `.txt`，每行一个目标：

```text
0 0.5234 0.4188 0.1211 0.0875
0 0.7345 0.6221 0.0984 0.0762
```

每行含义：`类别编号 中心x 中心y 宽 高`（中心坐标和宽高都是**归一化到 0~1**
的值，即除以图片宽高）。

目录结构（按 Ultralytics 约定）：

```text
D:\yolo\datasets\pod_pest\
├── data.yaml
├── images/
│   ├── train/    # 70%
│   ├── val/      # 20%
│   └── test/     # 10%（预测试用，训练时用不到）
└── labels/
    ├── train/
    ├── val/
    └── test/
```

划分规则：

- `images/train/xxx.jpg` 与 `labels/train/xxx.txt` **同名同目录**；
- 同一来源（同一段视频/同一天同角度）的帧**必须分到同一集合**，否则验证结果
  虚高（叫"数据泄漏"）；
- test 集训练和验证阶段都不要碰，留到第 5 章预测试。

`data.yaml`：

```yaml
path: /home/你的用户名/yolo/datasets/pod_pest   # 改成实际路径
train: images/train
val: images/val
test: images/test

names:
  0: damaged_pod
  1: healthy_pod
```

标注后抽查一遍：随机打开 20 张图，确认没有漏标、错标、框出半个目标。**标注
错误直接决定模型上限**，这是全流程最值得花时间的环节。

### 4.4 训练（迁移学习）

为什么从预训练权重开始：COCO 预训练模型已经学会了"边缘、纹理、形状"这些
通用视觉特征，我们的数据只要几百张就能微调出可用模型；从零训练需要几万张。

训练命令：

```bash
cd /d D:\yolo
yolo detect train \
  data=datasets/pod_pest/data.yaml \
  model=yolov8n.pt \
  epochs=100 imgsz=640 batch=16 device=0 \
  patience=20 project=runs name=pod_pest_v1
```

等价 Python：

```python
from ultralytics import YOLO

model = YOLO('yolov8n.pt')          # 加载 COCO 预训练权重
model.train(
    data='datasets/pod_pest/data.yaml',
    epochs=100, imgsz=640, batch=16,
    device=0, patience=20,
    project='runs', name='pod_pest_v1',
)
```

关键参数：

| 参数 | 作用 | 建议 |
|------|------|------|
| `epochs` | 训练轮数 | 先 100，看曲线再调 |
| `imgsz` | 输入分辨率 | 640 起步；树莓派部署再降到 480/320 |
| `batch` | 每批图片数 | 显存不够就减半（16→8→4） |
| `device` | 0=GPU，cpu=CPU | 地面站必须 0 |
| `patience` | 多少轮没提升就早停 | 20 左右，省时间 |
| `lr0` | 初始学习率 | 默认即可，异常了再调 |

> 8GB 显存（RTX 5060）参考：imgsz=640 时，`yolov8n` 可 batch 32、`yolov8s`
> 建议 16、`yolov8m` 建议 8；报 `CUDA out of memory` 就减半，或把 imgsz 降到
> 480。

训练产物在 `runs/pod_pest_v1/`：

```text
runs/pod_pest_v1/
├── weights/
│   ├── best.pt    # 验证集上表现最好的权重（部署用这个）
│   └── last.pt    # 最后一轮的权重（断点续训用）
├── results.png    # 损失和指标曲线
├── confusion_matrix.png
├── results.csv    # 每轮的数值，可导入表格分析
└── args.yaml      # 本次训练的所有参数，复现用
```

中断了不用重来：

```bash
yolo detect train model=runs/pod_pest_v1/weights/last.pt resume=True
```

### 4.5 训练常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| `CUDA out of memory` | 显存不够 | `batch` 减半，或换 `yolov8n` |
| 训练 loss 降、验证指标不升 | 过拟合 | 数据太少/标注不一致；加数据增强、提前早停 |
| 某类 mAP 特别低 | 类别不平衡 | 给少数类补数据，或调 `class_weights` |
| 训练很快但 mAP=0 | 标注全错/names 不对 | 检查 txt 里类别数字和 data.yaml 是否一致 |
| 验证比训练好很多 | 数据泄漏 | 同源帧跨集合了，重新划分 |

---

## 5. 预测试：部署前的模型验证

目标：在**上真机之前**，先在测试集和模拟场景里把模型验明白，别把问题带到
田里去。对应 README 第三阶段"验证并提升识别稳定性"。

### 5.1 测试集评估

```bash
# 用 test 集评估（验证阶段用的是 val，这里换成 test，第一次见的数据）
yolo detect val \
  model=runs/pod_pest_v1/weights/best.pt \
  data=datasets/pod_pest/data.yaml split=test
```

输出里关注四个指标：

| 指标 | 含义 | 对应问题 | 起步建议 |
|------|------|----------|----------|
| Precision（精确率） | 检出来的目标里，有多少是对的 | 误检 | ≥0.85 |
| Recall（召回率） | 真实目标里，检出多少 | 漏检 | ≥0.85 |
| mAP50 | IoU=0.5 时的平均精度 | 综合 | ≥0.80 |
| mAP50-95 | 更严格的综合指标 | 框得准不准 | 记录即可 |

指标不达标怎么调：

- 误检多（Precision 低）→ 提高 `conf` 阈值；加负样本（无目标画面）
- 漏检多（Recall 低）→ 降低 `conf` 阈值；补小目标/遮挡样本；加大 `imgsz`
- 都差 → 先查标注质量，再考虑加数据

### 5.2 场景预测试（三人在各自环境跑一遍）

| 场景 | 谁 | 怎么做 |
|------|-----|--------|
| 无人机视角 | 朱杨杰 | 用一段没参与训练的巡检视频喂模型，看能否稳定框出受害豆荚 |
| 机器人近景 | 马晨轩 | 树莓派上 `--detector yolo --model /home/pi/best.pt` 对着模拟豆荚跑 |
| 地面站离线 | 谢啸 | 批量跑测试图片，统计误检/漏检 |

记录模板（每人一份，反馈给谢啸）：

| 编号 | 截图/帧号 | 真值（是什么） | 模型输出 | 问题类型（误检/漏检/框偏） | 环境说明（光照/距离） |
|------|-----------|----------------|----------|--------------------------|----------------------|
| 001 | drone_02 帧 1200 | 受害豆荚 | 未检出 | 漏检 | 逆光、5m 高度 |
| 002 | 近景 045.jpg | 叶片 | damaged_pod 0.82 | 误检 | 叶片卷曲像蛀孔 |

### 5.3 迭代闭环（对应 README 风险应对）

```
现场采集 ──▶ 标注 ──▶ 训练 ──▶ 预测试 ──▶ 部署
   ▲                                        │
   └──────── 误检/漏检案例反馈 ◀────────────┘
```

**预测试验收**：test 集 mAP50 ≥0.80 起步，无人机视角和机器人近景各 50 个目标
的漏检率 ≤15%；把记录表反馈给谢啸后，能说出"下一轮数据要补什么"。

---

## 6. 模型导出与部署

训练好 `best.pt` 后，按部署方式导出不同格式。

### 6.1 导出

```bash
# 通用 ONNX（树莓派/地面站都能用）
yolo export model=runs/pod_pest_v1/weights/best.pt format=onnx imgsz=640

# TensorRT（仅 NVIDIA GPU，地面站推理最快）
yolo export model=runs/pod_pest_v1/weights/best.pt format=engine imgsz=640 device=0
```

### 6.2 方案 A：树莓派本地部署（离线可用）

```bash
# 在树莓派上（局域网模式下联网安装）
pip3 install ultralytics onnxruntime
```

把 `best.pt` 或 `best.onnx` 传到树莓派 `/home/pi/`，用整合程序切换检测器：

```bash
sudo systemctl stop spiderpi          # 先关掉开机自启主程序，避免抢占资源
python3 ~/spiderpi/advanced/autonomous_pick.py \
  --detector yolo --model /home/pi/best.pt --classes damaged_pod
```

树莓派是 CPU，性能要点：

- 必须用 `yolov8n`（换大模型会掉到 1~2 FPS）
- `imgsz` 降到 480 或 320（`yolo_detect.py` 的 `size` 参数）
- 真机验证阶段够用；整链路实时检测仍以地面站 GPU 为主

> 机器人近景定位沿用官方相机标定：抓取坐标靠 `camera_cal.yaml` 的 K/R/T 参数
> 做像素→机械臂坐标换算（`pixel_to_arm_coord`），首次使用前必须先在机器人上
> 完成官方位置校准（`camera_cal_main.py`），否则近景 YOLO 框得准也抓不准。

> 注意：`functions/yolo_detect.py` 单独运行时用的是代码里写死的默认参数
> （`YoloDetector()`），想直接单测自己的模型，把 `main()` 里的
> `YoloDetector()` 改成 `YoloDetector(model_path='/home/pi/best.pt',
> target_classes=('damaged_pod',))` 即可；不想改代码就用上面整合程序的
> `--detector yolo` 方式。

### 6.3 方案 B：地面站推理服务（推荐给实时巡检）

地面站接收无人机 RTSP 流并跑 YOLO，机器人只收"任务坐标"不跑模型。地面站
Flask 服务最小示例（正式版由杨梦享扩展成统一调度服务）：

```python
from flask import Flask, request, jsonify
from ultralytics import YOLO

app = Flask(__name__)
model = YOLO('/home/用户名/yolo/runs/pod_pest_v1/weights/best.pt')

@app.route('/detect', methods=['POST'])
def detect():
    # 教学版：直接喂路径/URL；正式版接入视频流后逐帧调用
    data = request.get_json(force=True)
    result = model.predict(data['source'], conf=0.45, verbose=False, device=0)[0]
    targets = []
    for box in result.boxes:
        cls_id = int(box.cls[0])
        if result.names[cls_id] in ('damaged_pod',):
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            targets.append({
                'class': result.names[cls_id],
                'conf': float(box.conf[0]),
                'center': [(x1 + x2) // 2, (y1 + y2) // 2],
            })
    return jsonify({'targets': targets})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
```

### 6.4 部署验收

| 检查项 | 验收标准 |
|--------|----------|
| 类别名 | 部署端 `--classes`/接口里的名字与 data.yaml 的 names 完全一致 |
| 树莓派端 | `autonomous_pick.py --detector yolo` 能框出模拟豆荚 |
| 地面站端 | `/detect` 接口返回 JSON，目标数、置信度正确 |
| 性能 | 树莓派 ≥3 FPS（够近景用）；地面站 ≥20 FPS（巡检实时） |

---

## 7. 空地协同：无人机 + 地面站 + 机器人互联

目标：把 README 4.4 的五步协同流程跑通——

```
无人机起飞巡检 → 视频回传地面站 → YOLO 检测 → 坐标下发机器人 → 机器人导航+机械臂抓捕
```

### 7.1 网络拓扑与固定 IP

所有设备连同一个双频千兆路由器，**在路由器 DHCP 里按 MAC 绑定固定 IP**（地面站
文档第 13 节 Q&A 也建议过）。以下用 `192.168.1.x` 示例，实际以路由器网段为准：

| 设备 | 角色 | 示例 IP |
|------|------|---------|
| 地面站 | 推理 + 调度 + pymavlink | 192.168.1.100 |
| SpiderPi Pro | 接收任务 + 近景确认 + 抓取 | 192.168.1.101 |
| 无人机图传（RTSP） | 视频源 | 192.168.1.10 |
| 无人机数传（QGC/pymavlink） | 位姿源 | 192.168.1.123（UDP 8080/14550） |

```text
        ┌──────────────────┐
        │  无人机 F450 V6C  │──RTSP 视频流──┐
        │  (Pixhawk 6C)    │──MAVLink 数传──┐│
        └──────────────────┘               ││
                              ┌────────────▼▼────────────┐
                              │  地面站 192.168.1.100     │
                              │  YOLO + pymavlink + 调度  │
                              └────────────┬─────────────┘
                                           │ HTTP POST /task
                                           ▼
                    ┌──────────────────────────────────────┐
                    │  SpiderPi Pro 192.168.1.101          │
                    │  导航 + 近景确认 + 机械臂             │
                    └──────────────────────────────────────┘
```

> **联调前置条件**：机器人默认是 AP 直连模式（机器人自己开热点，IP
> 192.168.149.1），地面站访问不到它。协同联调前必须先按
> [GroundRobot/开始配置.md](GroundRobot/开始配置.md) 第 7 节把机器人切成
> **STA 局域网模式**（`hiwonder-toolbox/wifi_conf.py` 中 `HW_WIFI_MODE=2`，
> 填路由器 SSID/密码），拿到 192.168.1.101 这类局域网 IP。

> 若网络部门需要从跨网段/公网接入（远程监控、异地联调），由网络部门按 7.8
> 的端口清单做端口转发，正式巡检仍以本地局域网为准。

**华为设备组网（本方案）**：

- 无线互通：用华为无线设备（如华为 AirEngine 系列 AP 或华为路由器）建一个
  局域网 SSID，地面站、SpiderPi Pro 全部以 STA 接入；建议优先 **5GHz 频段**
  （带宽大、干扰小），2.4GHz 作为兜底；
- 固定 IP：在华为设备 DHCP 里做**静态绑定**（按设备 MAC 分配固定 IP），保证
  7.3 的 HTTP 任务地址不变；
- 接入方式：地面站、机器人可无线接入，也支持网线接华为设备 LAN 口/交换机
  （有线更稳，推荐地面站有线 + 机器人无线）；
- 隔离（可选）：若网络部门要求，可在华为设备上划 VLAN，把"业务网段"
  （无人机/机器人/地面站）与办公网隔开，转发规则按 7.8、7.9 配置。

### 7.2 无人机 → 地面站：视频流 + 位姿

**通信系统总览（官方"通信系统简介"口径）**：

通信系统三要素是**源系统、传输系统、目的系统**：无人机一侧负责"发"
（画面 + 飞行参数），地面站一侧负责"收"。本项目有两条并行的无线链路：

| 链路 | 传什么 | 本机实现 | 关键参数 |
|------|--------|----------|----------|
| 图传（FPV） | 摄像头实时画面 | Minihomer 图数传一体 + IVG-G4 网络摄像头 | RTSP：`rtsp://192.168.1.10:554/...` |
| 数传（Telemetry） | 位置/姿态/高度/电量等参数 | Minihomer（Sub-1G 频段，WiFi HaLow，最远 1200m） | 地面站静态 IP `192.168.1.123`，QGC UDP 8080 |

Minihomer 基站端接口：**网口 1/网口 2**（接电脑或路由器）、**串口 1/串口 2**、
SMA 天线口、**配对按键**、**模式转换按键**。WiFi HaLow 基于 IEEE 802.11ah，
工作在 900MHz 低频段，穿透力强、传输远、低功耗，支持自适应速率和多信道并发，
组网方式有点对点/星型/树型/网状。

**本项目组网选择**：

- 无人机 ↔ 地面站：**点对点**（Minihomer 一对一，数传 1200m 覆盖）
- 无人机/地面站 ↔ 机器人：**星型**（路由器做 AP 中心，各设备 STA 接入）

> 官方通信系统简介：
> <https://docs.amovlab.com/f450-v6c-wiki/#/src/扩展帮助/通信系统简介>
> 配套教学（含本机配置与排查表）：
> [Dron/教学/数传与图传通信.md](Dron/教学/数传与图传通信.md)

**视频流**（检测输入）：

- 机载摄像头是 RTSP 网络流，地面站直接用 OpenCV/YOLO 拉流（3.3 节脚本）
- 或按 README 4.3 用 GStreamer UDP 推流/接收
- 巡检建议用辅码流（流畅），取证的用主码流（清晰）

**位姿**（坐标计算的另一半）：

```bat
:: 启动 pymavlink 读取脚本（监听 UDP 14550，Windows 下替代 MAVROS）
python D:\yolo\scripts\read_drone.py
```

`read_drone.py`（见 [GroundStation/开始配置.md](GroundStation/开始配置.md)
第 6 节）从 MAVLink 的 `LOCAL_POSITION_NED` / `GLOBAL_POSITION_INT` 消息里
读出局部坐标（米，NED）和 GPS，换算成 ENU 后就是 7.4 坐标换算里的"无人机在哪"。

> pymavlink 有两种连法：数传 UDP 监听（`udpin:0.0.0.0:14550`）和 USB 串口
> （`serial:COM3,57600`）。本机 F450 V6C 出厂走 Minihomer 数传，联调时以到货
> 硬件为准——先按 [Dron/教学/MAVLink与MAVROS.md](Dron/教学/MAVLink与MAVROS.md)
> 用 QGC 确认链路，再开 pymavlink。

### 7.3 地面站 → 机器人：任务下发

**首选：HTTP JSON**（树莓派不用装 ROS，最简单）。接收端已经集成进主程序：
`autonomous_pick.py` 启动时自动在后台开启 HTTP 服务（`communication/task_server.py`，
监听 5000 端口），并进入 **NAV** 状态自主判断、接收消息——**不用再单独启动
任何 py 文件**。

> 前置：先通过 VNC 连上树莓派桌面（官方资料包"6.远程桌面工具安装及连接"，
> 配置见 [GroundRobot/开始配置.md](GroundRobot/开始配置.md) 第 4~5 节），确认
> 已按 7.1 切成 STA 局域网模式；运行前先 `sudo systemctl stop spiderpi`。

```bash
# 机器人端（树莓派 VNC 终端）：启动主程序即自动收任务
python3 ~/spiderpi/advanced/autonomous_pick.py \
  --detector yolo --model /home/pi/best.pt --classes damaged_pod

# 确认 HTTP 服务已起来（看到 [TASK] HTTP 服务已启动 即成功）
```

地面站下发：

```bash
curl -X POST http://192.168.1.101:5000/task \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"T001","target":{"x":12.5,"y":8.3,"heading_deg":0.0,"confidence":0.87,"source":"drone_yolo"}}'
```

机器人状态查询（地面站仪表盘就是轮询这个接口）：

```bash
curl http://192.168.1.101:5000/status
```

返回示例：

```json
{"online": true, "state": "NAV", "position_m": {"x": 0.0, "y": 0.0},
 "heading_deg": 0.0, "picked_count": 0, "last_task": null,
 "last_result": null, "message": "等待地面站下发任务"}
```

消息格式约定（杨梦享统一维护，字段名前后端要一致）：

```text
{
  "task_id": "T20260823-001",
  "time": 1780000000.0,
  "target": {
    "x": 12.5,          // 目标局部坐标 x（米）
    "y": 8.3,           // 目标局部坐标 y（米）
    "heading_deg": 0.0, // 建议朝向（可选）
    "confidence": 0.87, // 检测置信度
    "source": "drone_yolo"
  }
}
```

机器人回执（状态回传，README 4.4 步骤 5 的"任务完成"）：

```json
{"status": "accepted"}
{"status": "done", "picked": true}
{"status": "failed", "reason": "target_lost"}
```

**进阶：统一指令协议**。Windows 方案下机器人端统一走 HTTP；如需更实时/双向
的通道，可升级为 WebSocket 或 MQTT（由杨梦享约定字段，格式与上面 JSON 一致）。
无人机端指令（转向/模式/位置设定）走 pymavlink 发 MAVLink，见 7.10。

### 7.4 目标坐标怎么算（像素 → 米）

这是协同链路最容易出错的一环，README 风险表专门列了"坐标对齐误差"。教学版
用**比例换算（GSD）**，先跑通，再逐步加精度。

**第一步：悬停 + 画面中心近似**

无人机悬停在目标上空，假设相机垂直向下、画面中心 ≈ 无人机正下方：

```python
img_cx, img_cy = frame.shape[1] // 2, frame.shape[0] // 2
dx_px = target_center_x - img_cx     # 目标相对画面中心（像素）
dy_px = target_center_y - img_cy
```

**第二步：标定 GSD（每像素多少米）**

在地面放一块已知边长的标定物（如 1m×1m 布），在相同高度拍一张图，量出它在
画面中的像素宽度：

```python
marker_m = 1.0                       # 标定物边长（米）
marker_px = 320                      # 现场量出来的像素宽
gsd = marker_m / marker_px           # 米/像素

dx_m = dx_px * gsd
dy_m = dy_px * gsd
```

**第三步：加上无人机坐标**

```python
# drone_x, drone_y 来自 pymavlink 读到的 MAVLink 局部位置（已换算成 ENU）
target_x = drone_x + dx_m
target_y = drone_y + dy_m
```

误差控制三招（对应 README 风险应对）：

1. **多帧平均**：悬停时连续取 20 帧的检测中心，去掉最大最小值后取平均；
2. **机头先朝正北**：教学版先固定机头方向再做换算；进阶再引入偏航角旋转
   （`dx_m*cos(yaw)+dy_m*sin(yaw)` 这类公式，符号以现场标定为准）；
3. **机器人二次确认**：机器人到目标附近后，用自己的近景 YOLO/颜色检测再确认
   一次，再执行抓取——这是 README 4.4 步骤 5 的"机载摄像头二次确认目标"。

### 7.5 完整协同时序（README 4.4 五步对照）

| 步骤 | 谁 | 做什么 | 输入→输出 |
|------|-----|--------|-----------|
| 1 | 无人机 | 沿航线巡检 | 摄像头画面 → RTSP 流 |
| 2 | 地面站 | YOLO 实时检测 | 视频帧 → 目标框 + 置信度 |
| 3 | 地面站 | 坐标计算与下发 | 目标像素 + 无人机位姿 → 任务 JSON |
| 4 | 机器人 | 自主导航至目标 | 任务坐标 → 六足行走 |
| 5 | 机器人 | 近景确认 + 机械臂摘除 | 近景画面 → 抓取 → 状态回传 |

数据流图：

```text
F450 V6C 机载摄像头 ──RTSP/GStreamer──▶ 地面站视频流
F450 V6C 飞控 ──MAVLink──▶ pymavlink（Windows 地面站）
                                      │
              ┌───────────────────────┘
              ▼
        地面站 YOLO 检测（best.pt）
              │  目标像素 + 无人机位姿 + GSD
              ▼
        目标局部坐标（x, y）
              │  HTTP POST /task（或 ROS Topic）
              ▼
        SpiderPi Pro 接收任务 ──▶ 自主导航 ──▶ 近景 YOLO 二次确认 ──▶ 机械臂摘除
              ▲                                                        │
              └────────────── HTTP 状态回传（done/failed）◀─────────────┘
```

### 7.6 第三阶段验收标准（对照 README 4.5）

| README 验收 | 怎么测 | 达标 |
|-------------|--------|------|
| 无人机自主飞行 | QGC 航线试飞 | 按预设航线完成巡检 |
| pymavlink 通信 | `python D:\yolo\scripts\read_drone.py` | 位置/姿态实时更新 |
| 视频传输延迟 <200ms | 画面时间戳对比或打点计时 | 端到端 <200ms |
| YOLO 识别豆荚螟 | 预测试集 + 现场 | mAP50 ≥0.80 起步，现场稳定 |
| 完整协同 <5分钟 | 从起飞到抓取完成全程计时 | 全流程 <5 分钟 |

### 7.7 分工与协作接口

| 接口 | 提供方 | 接收方 | 内容 |
|------|--------|--------|------|
| RTSP 视频流 | 朱杨杰（无人机） | 谢啸（地面站） | 巡检画面 |
| 无人机位姿 | 杨梦享（pymavlink） | 谢啸（坐标计算） | MAVLink `LOCAL_POSITION_NED`（转 ENU） |
| 目标任务 JSON | 谢啸（地面站） | 马晨轩（机器人） | 坐标 + 置信度 |
| 执行状态 | 马晨轩（机器人） | 杨梦享（调度） | done/failed |
| 误检/漏检案例 | 朱杨杰、马晨轩 | 谢啸（训练） | 记录表 → 下一轮数据集 |
| 端口转发/流信号转发 | 网络部门（IT） | 谢啸、杨梦享 | 按 7.8/7.9 配置 NAT、视频流转发与安全策略 |

### 7.8 网络部门对接与端口转发

> 场景：网络部门（IT）负责通信链路。当联调需要**跨网段/公网访问**（例如网络
> 部门远程监控地面站、或外部网络连入设备）时，需要在路由器/防火墙上做
> **端口转发（NAT）**。

**第一步：先定好内网固定 IP**（7.1 节已做）。端口转发必须指向固定的内网
地址，所以先在所有设备联网后，在路由器 DHCP 里按 MAC 绑定 IP。

**第二步：确定要转发的端口清单**：

| 服务 | 协议 | 端口 | 内网设备 | 用途 | 默认建议 |
|------|------|------|----------|------|----------|
| QGC 数传 | UDP | 8080 | 地面站 192.168.1.100 | 无人机飞行参数/图传会话 | 转发 |
| pymavlink 数传 | UDP | 14550 | 地面站 | MAVLink 数据（可选） | 视需要 |
| RTSP 图传 | TCP | 554 | 地面站/摄像头 | 拉取无人机画面 | 转发 |
| GStreamer 推流 | UDP | 5000 | 地面站 | 机载视频流接收 | 视需要 |
| 地面站 YOLO API | TCP | 8000 | 地面站 | 远程调用检测服务 | 按需 |
| 机器人任务接口 | TCP | 5000 | 机器人 192.168.1.101 | 任务下发/状态回传 | 默认不公网转发 |
| SSH | TCP | 22 | 各设备 | 远程调试 | 仅内网/VPN |

**第三步：路由器端口转发配置**（以常见路由器为例）：

1. 路由器后台 → 转发规则 / 端口映射（NAT）；
2. 新建规则：外部端口 = 内部端口（或自定义外部端口），内网 IP = 设备固定 IP；
3. 协议按上表选 TCP / UDP / 两者；
4. 保存后从外部网络用 `公网IP:端口` 验证。

**验收命令**（网络部门配置后，两边同时测）：

```bash
# 本端确认服务在监听
ss -lntup | grep -E '8080|5000|8000|554'

# 对端确认端口通（以地面站 YOLO API 8000 为例）
nc -vz 公网IP或内网IP 8000
curl http://公网IP:8000/      # HTTP 服务可直接验证
```

**安全要求（重要）**：

- 控制类端口（机器人 5000、SSH 22）**不建议直接暴露公网**，容易被外部接管设备；
- 优先只把网络部门的固定公网 IP 加入防火墙白名单；
- 更稳妥的替代方案：VPN（WireGuard / OpenVPN）或内网穿透（Tailscale / frp /
  ZeroTier），不开公网端口也能远程联调；
- 无人机数传/图传端口跨公网转发时注意 UDP 丢包和延迟，可能不满足 <200ms
  要求——**跨网段联调只用于调试，正式巡检全程在本地局域网进行**。

**给网络部门的对接信息**（直接复制这段发过去）：

```text
联调内网：192.168.1.0/24（华为无线设备建网，5GHz SSID）
固定 IP：地面站 192.168.1.100、SpiderPi Pro 192.168.1.101、无人机图传 192.168.1.10
需转发端口：UDP 8080（QGC 数传）、TCP 554（RTSP）、UDP 5000（GStreamer，按需）、
           TCP 8000（地面站 API，按需）、TCP 8554（MediaMTX 视频转发，按需）
无人机流信号：由地面站转发出（MediaMTX RTSP 或 GStreamer UDP，见 7.9）；
           华为设备如用组播需开 IGMP Snooping
安全：仅允许指定公网 IP 访问；机器人 5000 / SSH 22 默认不公网转发
```

### 7.9 无人机流信号转发出去（无线/有线均可）

> 目标：把无人机画面从地面站转发出去，供网络部门或其他设备查看，路径可以是
> 华为无线网，也可以是网线接到华为交换机/上联网络。

**方式一：地面站软件转发（推荐，灵活）**

用 MediaMTX 把拉到的 RTSP 流"再发布"一次，转发和源隔离，多台设备可同时看：

```bash
# 1) 下载并运行 MediaMTX（GitHub releases），默认监听 8554
./mediamtx

# 2) 把无人机 RTSP 流推给本地 MediaMTX（ffmpeg 转封装、不转码，开销很小）
ffmpeg -i "rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=1.sdp?" \
       -c copy -f rtsp rtsp://127.0.0.1:8554/drone

# 3) 任何设备都能拉流查看：
#    rtsp://地面站IP:8554/drone
```

或者用 GStreamer 直接把流 UDP 推给单个目标 IP（无线或有线路径都行）：

```bash
gst-launch-1.0 rtspsrc location="rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=1.sdp?" latency=200 ! \
  rtph264depay ! h264parse ! rtph264pay ! udpsink host=目标IP port=5000
```

**方式二：华为设备网络层转发**

- 在华为设备上做端口映射/静态 NAT：把地面站的 RTSP 554（或 MediaMTX 8554、
  GStreamer UDP 5000）转发到上联/公网端口；
- 目标设备在同一局域网时直接用"目标 IP:端口"拉流即可，不需要转发；
- 若要多台设备同时收 UDP 组播，华为设备需开启 **IGMP Snooping** 并放行
  组播地址段。

两种方式对比：

| 方式 | 延迟 | 多路观看 | 需要改华为设备 | 适用 |
|------|------|----------|----------------|------|
| 地面站软件转发（MediaMTX/ffmpeg） | 低 | 支持 | 基本不用（端口转发可选） | 推荐，调试和演示 |
| 华为设备端口/组播转发 | 最低 | 组播需配 IGMP | 需要 | 固定链路、网络部门统一管理 |

> 提醒：跨网段/公网转发视频要注意带宽和延迟，<200ms 的要求以本地局域网为准；
> 转发时可用辅码流（800×448@25fps）减小带宽占用。

### 7.10 自动化控制闭环：AI 决定"左转/右转/前进"

> 核心思想：自动化 = **感知（YOLO 检测目标位置）→ 决策（算偏移，生成
> LEFT/RIGHT/FORWARD 指令）→ 通信（把指令传给执行端）→ 执行（无人机/机器人
> 动作）**。YOLO 除了回答"这是不是受害豆荚"，还要回答"目标在画面哪个方向、
> 离我多远"——这两个量就是左转/右转/前进的依据。

**三个核心量**：

| 量 | 怎么算 | 决定什么 |
|----|--------|----------|
| `offset_x` | `target_center_x - 画面中心x` | 左转 / 右转 |
| `offset_y` / 目标大小 | `target_center_y - 画面中心y` 或检测框面积 `area` | 前进 / 后退 / 高度调整 |
| `area` / `conf` | 检测框面积、置信度 | 是否够近、能否执行抓取/悬停 |

**通用转向规则**（机器人和无人机同一套逻辑，负反馈）：

```python
img_cx = frame.shape[1] // 2
offset = det['center'][0] - img_cx     # 目标中心相对画面中心（像素）
tol = 40                               # 容差：在容差内认为已对准

if abs(offset) <= tol:
    command = 'FORWARD'                # 已对准 → 前进（或保持）
elif offset > 0:
    command = 'RIGHT'                  # 目标在画面右侧 → 右转
else:
    command = 'LEFT'                   # 目标在画面左侧 → 左转
```

**地面机器人应用（已有实现）**：

`GroundRobot/Code/advanced/autonomous_pick.py` 的 APPROACH 状态就是这套逻辑：
SEARCH（没目标就转圈找）→ APPROACH（`offset` 决定左/右转，对准后前进）→
PICK（`area` 够大且进入机械臂可及范围→抓取）。换成 YOLO 检测器即可：

```bash
python3 ~/spiderpi/advanced/autonomous_pick.py \
  --detector yolo --model /home/pi/best.pt --classes damaged_pod
```

**无人机应用（分三步走）**：

1. **半自动（人飞，AI 提示）**：YOLO 检测后，程序在画面上叠加
   "目标在左侧/右侧/已对准"提示，人按提示转向——先验证"检测→决策"逻辑对不对；
2. **自动对准（悬停中）**：把 `offset` 换算成目标相对无人机的偏移（米，
   用 7.4 的 GSD），通过 pymavlink 给飞控发 MAVLink 指令（机头转向或位置
   微调），把目标拉回画面中心；
3. **全自动巡检**：目标进入画面 → 自动对准 → 悬停测距 → 坐标下发机器人 →
   继续巡检。

```python
# 无人机端：画面偏移 → 转向指令（示意）
yaw_step = 5                            # 度，每次转向步长
if offset > tol:
    print('右转', yaw_step)             # 正式版：pymavlink 发 SET_ATTITUDE_TARGET
elif offset < -tol:
    print('左转', yaw_step)
else:
    print('目标在画面中心，保持/前进')
```

进阶：位置级控制用 pymavlink 发 `SET_POSITION_TARGET_LOCAL_NED`（内容为 7.4
算出的目标点 `target_x, target_y`），PX4 切 OFFBOARD 模式执行，无人机即可
自动飞到目标正上方。

**通信的角色（AI 出主意，通信去传达）**：

- AI（YOLO + 决策逻辑）算出指令：`LEFT / RIGHT / FORWARD / HOLD / PICK`；
- 通信层把指令送达执行端：机器人走 7.3 的 HTTP POST `/task`，无人机走
  MAVLink（pymavlink 发送）；
- 消息里带上指令和坐标，例如：

```json
{"task_id": "T001", "cmd": "RIGHT", "step_deg": 5, "target": {"x": 12.5, "y": 8.3}}
```

- 执行端回执 `executing / done / failed`，形成闭环，AI 根据回执决定下一步。

**验收**：

| 环节 | 验收标准 | 对应 README |
|------|----------|-------------|
| 转向判定 | 目标放画面左/中/右，AI 输出 LEFT/HOLD/RIGHT 正确率 100% | 3.3 视觉识别 |
| 机器人闭环 | 自主"识别→对准→抓取"成功率 ≥70% | 3.5 第二阶段验收 |
| 无人机自动对准 | 悬停时目标保持在画面中心 ±50px | 4.5 视频传输/YOLO |
| 全流程 | 侦察→定位→抓捕 <5 分钟 | 4.5 完整协同 |

### 7.11 地面站仪表盘（无人机/机器人/模型信息）

仪表盘是一个跑在地面站的 HTTP 服务（Flask），汇总三类信息并支持网页下发
任务，代码在 [GroundStation/dashboard/](GroundStation/dashboard/)：

```bat
cd GroundStation\dashboard
python app.py
```

浏览器打开 `http://localhost:20001`。结构：

```text
┌────────────────── 地面站仪表盘 (HTTP 20001) ──────────────────┐
│  无人机卡片 ← pymavlink 读 MAVLink（位姿/GPS/模式/电量）     │
│  机器人卡片 ← GET http://192.168.1.101:5000/status（轮询）   │
│  模型卡片   ← YOLO 检测线程（RTSP 拉流，统计 FPS/检测数）    │
│  视频预览   ← /video.mjpeg（YOLO 画框后的画面）              │
│  下发任务   ← POST /api/robot/task → 转发给机器人 /task      │
└──────────────────────────────────────────────────────────────┘
```

主要接口：

| 接口 | 方法 | 说明 |
|------|------|------|
| `/api/drone/status` | GET | 无人机状态（pymavlink 读 MAVLink 缓存） |
| `/api/robot/status` | GET | 机器人状态（代理机器人 `/status`） |
| `/api/model/status` | GET | 模型信息与检测统计（FPS、各类别计数、最近检测） |
| `/api/robot/task` | POST | 从网页下发任务给机器人 |
| `/video.mjpeg` | GET | 无人机画面预览 |

配置在 `dashboard/config.py`：机器人 IP、pymavlink 连接串、RTSP 地址、模型
路径、仪表盘端口。依赖：`python -m pip install flask requests pymavlink`。
网络部门如需跨网段访问仪表盘，转发
DASHBOARD_PORT（20001）即可（见 7.8）。

---

## 8. 三阶段学习路线（引导）

| 阶段 | 主题 | 本文档章节 | 动手任务 | 验收 |
|------|------|------------|----------|------|
| 一 | 从人工到自动 | 第 3 章 | 图片/视频/摄像头/RTSP 四种输入全跑通 | 能测 FPS，能读输出 |
| 二 | 从单机到协同 | 第 4、5、7 章 | 采集标注训练出 best.pt；预测试通过；网络链路通；YOLO→转向闭环（7.10） | mAP50≥0.80；LEFT/HOLD/RIGHT 判定正确；无人机/机器人数据链路打通 |
| 三 | 从可用到可靠 | 第 6、7 章 | 部署 + 现场全流程联调 | README 4.5 全部验收项达标 |

---

## 9. 常见问题排查总表

| 现象 | 可能原因 | 排查/解决 |
|------|----------|-----------|
| `CUDA: False` | 驱动或 torch 版本问题 | `nvidia-smi`；`pip show torch` 看是否带 `+cu128`；按开始配置.md 第 3~4 节重装 |
| 权重下载失败 | 国内网络访问 GitHub 慢 | 手动下载到 `D:\yolo\weights\` 再指定路径 |
| 训练 `CUDA out of memory` | 显存不够 | `batch` 减半；换 `yolov8n`；降 `imgsz` |
| 训练正常但 mAP 很低 | 标注错误/类别不一致/数据太少 | 抽查标注；核对 txt 类别数字与 names；补数据 |
| 现场效果比测试差 | 训练数据与现场光照/角度差太远 | 用现场视角补拍（无人机俯拍 + 机器人近景都要） |
| 误检多 | 阈值太低/负样本少 | 提高 `conf`；加无目标画面负样本 |
| 漏检多 | 阈值太高/小目标 | 降低 `conf`；加大 `imgsz`；补小目标样本 |
| RTSP 卡顿 | 用了主码流/带宽不足 | 换辅码流；H.264 + UDP；降低分辨率 |
| 机器人收不到任务 | IP/端口/防火墙/JSON 字段 | `ping` 通不通；`curl` 能不能访问；字段名与约定一致 |
| pymavlink 收不到心跳 | 端口/防火墙/数传目标 | 先让 QGC 能连上；放行 UDP 14550；数传端加地面站 IP+14550 |
| 跨网段/公网连不上设备 | 端口没转发或防火墙拦截 | 按 7.8 核对端口清单与 NAT 规则；两边 `nc -vz` 验证 |
| 机器人到了但抓不到 | 坐标误差 | 多帧平均；二次确认；重新标定 GSD |
| 部署端不画框 | 类别名不匹配 | 打印 `results.names`，和 data.yaml 核对 |

---

## 10. 动手练习清单

1. 用 `yolov8n.pt` 检测一张图，打印每个框的 `cls/conf/xyxy`（第 3 章）
2. 把 `conf` 从 0.45 调到 0.9，观察误检和漏检怎么变化
3. 用 `yolo benchmark` 对比 GPU 和 CPU 的 FPS，记下来（第 3 章）
4. 用手机/相机拍 20 张豆荚照片，自己用 LabelImg 标注成 YOLO 格式（第 4 章）
5. 写一个 20 张图的 mini 训练，跑 10 个 epochs，看懂 results.png（第 4 章）
6. 用 test 集跑 `yolo detect val ... split=test`，解读 Precision/Recall/mAP50
   （第 5 章）
7. 导出 ONNX，并用 ONNX 权重跑一次推理（第 6 章）
8. 让机器人端跑起 `/task` 服务，用地面站 `curl` 下发一条任务并收到 accepted
   （第 7 章）
9. 悬停模式下量一次 GSD，把画面中心 +200px 的目标换算成米坐标（第 7 章）
10. 完整走一遍：模拟目标画面 → 地面站检测 → 下发 → 机器人执行 → 状态回传
    （第 7 章，全程计时）
11. 把目标放在画面左/中/右，跑一个打印 LEFT/HOLD/RIGHT 的判定脚本
    （第 7.10 节）
12. 无人机悬停，人工把机头偏开，验证"目标偏左→左转"的提示/指令输出
    （第 7.10 节）

---

## 附录 A：命令速查

| 用途 | 命令 |
|------|------|
| 图片推理 | `yolo predict model=yolov8n.pt source=test.jpg save=True` |
| 摄像头推理 | `yolo predict model=yolov8n.pt source=0 show=True` |
| RTSP 推理 | `yolo predict model=yolov8n.pt source='rtsp://...' save=True` |
| 性能测试 | `yolo benchmark model=yolov8n.pt device=0` |
| 训练 | `yolo detect train data=data.yaml model=yolov8n.pt epochs=100 imgsz=640 batch=16 device=0` |
| 断点续训 | `yolo detect train model=runs/xxx/weights/last.pt resume=True` |
| 验证 | `yolo detect val model=runs/xxx/weights/best.pt data=data.yaml` |
| 测试集评估 | `yolo detect val model=best.pt data=data.yaml split=test` |
| 导出 ONNX | `yolo export model=best.pt format=onnx imgsz=640` |
| 导出 TensorRT | `yolo export model=best.pt format=engine imgsz=640 device=0` |
| 看无人机位姿 | `python D:\yolo\scripts\read_drone.py`（pymavlink） |
| 下发任务 | `curl -X POST http://192.168.1.101:5000/task -H 'Content-Type: application/json' -d '{"target":{"x":12.5,"y":8.3}}'` |

## 附录 B：术语表

| 术语 | 含义 |
|------|------|
| 目标检测 | 找出图中所有目标的类别和位置框 |
| 预训练权重 | 在大数据集（如 COCO）上训练好的模型参数，作为微调起点 |
| 迁移学习 | 用预训练权重在自家小数据集上继续训练 |
| 标注 | 给图片里的目标画框并标类别，YOLO 格式为 txt |
| 置信度 conf | 模型认为检测结果成立的概率 |
| NMS | 非极大值抑制，合并同一目标的重复框 |
| Precision / Recall | 精确率（误检）/ 召回率（漏检） |
| mAP50 / mAP50-95 | 检测综合指标，IoU 阈值不同 |
| GSD | 地面采样距离，每像素对应地面多少米 |
| RTSP | 网络视频流协议，无人机图传常用 |
| MAVLink / pymavlink | 飞控通信协议 / MAVLink 的官方 Python 库（跨平台） |
| HTTP JSON | 地面站↔机器人任务下发与状态回传的格式 |
| ENU | 局部坐标系：东-北-上，本项目坐标换算统一用 ENU（由 NED 转换） |
