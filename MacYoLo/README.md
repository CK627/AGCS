# MacYoLo —— macOS 本机 YOLO 训练方式

> 本机：**Apple M2 Pro / 32GB 统一内存**（已确认）。
> 用途：不依赖地面站（Windows + NVIDIA）也能在 Mac 上训练 YOLO，
> 适合小数据集试跑、外出标数据、模型验证；正式大训练和 RTSP 实时检测仍以
> 地面站 RTX 5060 为主。
>
> 概念讲解（置信度、类别名、指标等）见
> [GroundStation/YOLO训练教程.md](../GroundStation/YOLO训练教程.md)，
> 本文档只讲 macOS 与地面站**不同的部分**。

---

## 1. macOS 和地面站（Windows+CUDA）的区别

| 项目 | 地面站 Windows | Mac 本机（M2 Pro） |
|------|---------------|--------------------|
| GPU 加速 | CUDA（RTX 5060） | **MPS**（Metal Performance Shaders） |
| 设备参数 | `device=0` | `device='mps'`（不可用时 `'cpu'`） |
| 检查加速 | `nvidia-smi` | `torch.backends.mps.is_available()` |
| torch 安装 | 必须 `+cu128` 版 | 默认 pip 版自带 MPS，**不需要**特殊源 |
| 内存 | 32G 内存 + 8G 显存 | 32G **统一内存**（GPU/CPU 共享） |
| 驱动 | NVIDIA 驱动 ≥570 | 系统自带 Metal 驱动 |

**理解**：MPS 就是 PyTorch 在 Apple Silicon 上的"GPU 后端"，和 CUDA 是同一个
东西的两个平台实现。代码上唯一区别是 `device` 参数和安装方式。

## 2. 环境自检与安装

### 2.1 确认机器

```bash
sysctl -n machdep.cpu.brand_string    # Apple M2 Pro
sysctl -n hw.memsize                  # 34359738368 = 32GB
python3 --version                     # 本机 3.13.3
```

### 2.2 建虚拟环境并安装（推荐）

```bash
cd /Users/jj/Documents/MyCode/AGCS/MacYoLo
python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip
# 国内网络慢可加 -i https://pypi.tuna.tsinghua.edu.cn/simple
python -m pip install torch torchvision ultralytics opencv-python
```

> 不需要像 Windows 那样指定 `--index-url .../cu128`：macOS 的 PyTorch 默认
> 版本就包含 MPS 支持。

### 2.3 验证 MPS 可用

```bash
python3 -c "import torch; print('torch', torch.__version__, 'MPS:', torch.backends.mps.is_available())"
```

期望输出：

```text
torch 2.x.x MPS: True
```

`MPS: True` 即 GPU 加速可用。跑一次官方示例推理确认：

```bash
python3 -c "from ultralytics import YOLO; m=YOLO('yolov8n.pt'); m.predict('https://ultralytics.com/images/bus.jpg', save=True, device='mps')"
```

## 3. 训练

### 3.1 准备数据

抽帧、标注、划分、`data.yaml` 全部和地面站教程通用，直接复用
[GroundStation/YOLO训练教程.md](../GroundStation/YOLO训练教程.md) 第 3~4 节
的脚本（纯 Python，macOS 同样能跑）。前期想快速收集"大青虫"图片素材，
可用 [Crawler/](../Crawler/) 的百度图片爬虫（爬完记得人工清洗）。数据集建议放在：

```text
~/Documents/MyCode/AGCS/MacYoLo/datasets/pod_pest/
├── data.yaml
├── images/{train,val,test}
└── labels/{train,val,test}
```

### 3.2 训练脚本（带注释）

```bash
source .venv/bin/activate
python scripts/train_macos.py \
  --data datasets/pod_pest/data.yaml \
  --model yolov8n.pt \
  --epochs 100 --batch 16 --imgsz 640
```

脚本内容见 [scripts/train_macos.py](scripts/train_macos.py)，核心就一处和
Windows 不同：

```python
device = 'mps' if torch.backends.mps.is_available() else 'cpu'
model.train(..., device=device)
```

### 3.3 M2 Pro / 32G 的参数建议

| 模型 | imgsz | batch | 说明 |
|------|-------|-------|------|
| yolov8n | 640 | 32 | 推荐，小数据集首选 |
| yolov8s | 640 | 16 | 精度更高，速度慢一档 |
| yolov8m | 640 | 8 | 32G 统一内存也能跑，但很慢，不推荐在 Mac 上常用 |

训练时间参考（几百张图、100 epochs）：yolov8n 约 30~60 分钟、yolov8s 约
1~2 小时，**以实测为准**。训练完模型在 `runs/xxx/weights/best.pt`。

## 4. 推理/确认

```bash
python scripts/predict_macos.py --model runs/pod_pest_v1/weights/best.pt --source 0
```

`--source` 支持图片路径、视频路径、`0`（摄像头）、RTSP 地址。脚本自动选择
MPS，并对类别名/置信度/面积做筛选（概念见训练教程第 1、6 章）。

M2 Pro 推理性能参考：yolov8n @ 640 约 **30~60 FPS**（MPS），实时性足够
演示和近景验证；巡检级实时检测仍建议放地面站 5060。

## 5. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `MPS: False` | torch 版本旧/装成 x86 版 | `pip install --upgrade torch`；确认是 arm64 Python |
| MPS 训练中途报错 | 个别算子 MPS 不支持 | 先用 `--device cpu` 跑通，再升级 torch；或小 batch |
| 内存不足报错 | 统一内存被占满 | `batch` 减半；`imgsz` 降到 480；关掉大程序 |
| pip 装包很慢 | 网络 | 加清华镜像 `-i https://pypi.tuna.tsinghua.edu.cn/simple` |
| 首次下载 yolov8n.pt 失败 | GitHub 访问慢 | 手动下载放到项目目录，`--model ./yolov8n.pt` |
| 结果和地面站不一致 | 版本/设备差异 | 两边用同一份 `data.yaml` 和 `best.pt`；以地面站正式部署为准 |

## 6. 一句话总结

**Mac 当"便携训练台"用：安装只需一个 venv，训练把 `device=0` 换成
`device='mps'`，其余和地面站教程完全一样；数据、模型、`data.yaml` 两边通用。**
