#!/usr/bin/env python3
# coding=utf8
"""macOS（Apple Silicon / MPS）YOLO 训练脚本。

和地面站 Ubuntu 版唯一的关键区别：device 用 'mps' 而不是 '0'。

用法：
    source ../.venv/bin/activate
    python train_macos.py \
      --data ../datasets/pod_pest/data.yaml \
      --model yolov8n.pt --epochs 100 --batch 16 --imgsz 640
"""
import argparse

import torch
from ultralytics import YOLO


def pick_device():
    """自动选择加速后端：MPS（Apple GPU）→ CPU。"""
    if torch.backends.mps.is_available():
        return 'mps'          # M2 Pro 的 Metal GPU 加速
    return 'cpu'              # 保险兜底，慢但一定能跑


def main():
    parser = argparse.ArgumentParser(description='macOS YOLO 训练')
    parser.add_argument('--data', default='datasets/pod_pest/data.yaml',
                        help='数据集配置（含类别名）')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='预训练权重（迁移学习起点）')
    parser.add_argument('--epochs', type=int, default=100, help='训练轮数')
    parser.add_argument('--batch', type=int, default=16, help='每批图片数')
    parser.add_argument('--imgsz', type=int, default=640, help='输入分辨率')
    parser.add_argument('--device', default=None,
                        help='强制指定 mps/cpu（默认自动选择）')
    parser.add_argument('--name', default='pod_pest_v1', help='输出目录名')
    args = parser.parse_args()

    device = args.device or pick_device()
    print('使用加速后端:', device)

    model = YOLO(args.model)      # 加载 COCO 预训练权重，迁移学习
    model.train(
        data=args.data,           # 数据集配置
        epochs=args.epochs,       # 训练轮数
        batch=args.batch,         # 每批图片数（32G 统一内存参考值见 README）
        imgsz=args.imgsz,         # 输入分辨率
        device=device,            # 'mps' 或 'cpu'（macOS 没有 CUDA）
        patience=20,              # 连续 20 轮没提升就早停
        project='runs',
        name=args.name,
        verbose=True,
    )
    print('\n训练完成！模型在：')
    print('  runs/%s/weights/best.pt  (部署用这个)' % args.name)
    print('  runs/%s/weights/last.pt  (续训用)' % args.name)


if __name__ == '__main__':
    main()
