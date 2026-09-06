#!/usr/bin/env python3
"""
YOLOv8n 模型训练脚本
针对4G显存优化：冻结backbone + 混合精度 + 小batch
用法：python3 train.py --data data.yaml --imgsz 416 --batch 8 --epochs 50
"""
import argparse
from ultralytics import YOLO

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='YOLOv8n 训练')
    parser.add_argument('--data', type=str, default='data.yaml', help='数据集配置文件')
    parser.add_argument('--imgsz', type=int, default=416, help='输入图像尺寸')
    parser.add_argument('--batch', type=int, default=8, help='batch size（4G显存建议8）')
    parser.add_argument('--epochs', type=int, default=50, help='训练轮数')
    parser.add_argument('--freeze', type=int, default=10, help='冻结backbone层数')
    parser.add_argument('--device', type=str, default='0', help='训练设备: 0/GPU/cpu')
    parser.add_argument('--project', type=str, default='runs/train', help='输出目录')
    parser.add_argument('--name', type=str, default='car_yolov8n', help='实验名称')
    args = parser.parse_args()

    # 加载COCO预训练模型（从预训练开始，不是随机初始化）
    # 这一步很关键：预训练模型已经学好了通用特征，小数据集也能快速收敛
    model = YOLO('yolov8n.pt')

    # 开始训练
    results = model.train(
        data=args.data,              # 数据集配置
        imgsz=args.imgsz,            # 输入尺寸，416比640省显存
        batch=args.batch,            # batch size
        epochs=args.epochs,          # 训练轮数
        freeze=args.freeze,          # 冻结backbone前N层，省显存+防过拟合
        amp=True,                    # 混合精度训练，省30%显存
        device=args.device,          # 训练设备
        project=args.project,        # 输出目录
        name=args.name,              # 实验名
        patience=20,                 # 早停：20轮不提升就停止
        close_mosaic=15,             # 最后15轮关闭mosaic增强，稳定精度
        mosaic=0.5,                  # mosaic增强比例，小数据集降低
        mixup=0.0,                   # 关闭mixup，小数据集不需要
        workers=4,                   # 数据加载线程数
        seed=42,                     # 随机种子，保证可复现
    )

    print(f"\n训练完成！")
    print(f"最佳模型路径: {args.project}/{args.name}/weights/best.pt")
    print(f"最后一轮模型: {args.project}/{args.name}/weights/last.pt")

if __name__ == '__main__':
    main()
