#!/usr/bin/env python3
"""
YOLOv8n 导出 ONNX 模型（为地平线部署优化）
关键参数：opset=12, simplify=True, 固定batch=1
用法：python3 export_onnx.py --weights best.pt --imgsz 416
"""
import argparse
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(description='导出ONNX模型')
    parser.add_argument('--weights', type=str, required=True, help='PyTorch权重文件(.pt)')
    parser.add_argument('--imgsz', type=int, default=416, help='输入尺寸')
    parser.add_argument('--opset', type=int, default=12, help='ONNX opset版本（地平线兼容11/12）')
    parser.add_argument('--simplify', action='store_true', default=True, help='简化计算图')
    args = parser.parse_args()

    # 加载模型
    model = YOLO(args.weights)

    # 导出ONNX
    # 关键说明：
    # 1. opset=12: 地平线工具链对opset11/12兼容性最好
    # 2. simplify=True: 用onnxsim简化计算图，去除冗余算子，减少转换问题
    # 3. dynamic=False: 固定输入尺寸，边缘部署不需要动态shape
    # 4. batch=1: 单帧推理，边缘设备通常一次处理一帧
    model.export(
        format='onnx',
        imgsz=args.imgsz,
        opset=args.opset,
        simplify=args.simplify,
        dynamic=False,
        batch=1,
    )

    # 验证导出的ONNX模型
    import onnx
    onnx_path = args.weights.replace('.pt', '.onnx')
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)

    print(f"\n导出成功: {onnx_path}")
    print(f"输入节点: {[i.name for i in onnx_model.graph.input]}")
    print(f"输出节点: {[o.name for o in onnx_model.graph.output]}")
    print(f"算子总数: {len(onnx_model.graph.node)}")

if __name__ == '__main__':
    main()
