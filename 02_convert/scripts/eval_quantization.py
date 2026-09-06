#!/usr/bin/env python3
"""
量化精度对比评估脚本
对比 FP32/FP16/INT8 三个版本的精度、延迟、内存
用法：python3 eval_quantization.py --models model_fp32.bin model_fp16.bin model_int8.bin \
        --data ../dataset/images/val --output ../06_reports/quantization/
"""
import argparse
import os
import time
import json
import numpy as np
import cv2

def parse_args():
    parser = argparse.ArgumentParser(description='量化精度对比评估')
    parser.add_argument('--models', nargs='+', required=True, help='模型文件列表')
    parser.add_argument('--data', type=str, required=True, help='测试图片目录')
    parser.add_argument('--annotations', type=str, help='COCO格式标注文件（可选）')
    parser.add_argument('--output', type=str, default='./', help='报告输出目录')
    parser.add_argument('--imgsz', type=int, default=416, help='输入尺寸')
    parser.add_argument('--num-images', type=int, default=100, help='评估图片数量')
    return parser.parse_args()

def load_model(model_path):
    """加载地平线模型（以实际hrt API为准）"""
    try:
        from horizon_tc_ui import hrt
        model = hrt.load_model(model_path)
        return model, 'hrt'
    except ImportError:
        print("[警告] 未找到地平线hrt，使用ONNX Runtime模拟")
        import onnxruntime as ort
        sess = ort.InferenceSession(model_path.replace('.bin', '.onnx'))
        return sess, 'ort'

def preprocess(img_path, imgsz):
    """图像预处理：resize + BGR2RGB + HWC2CHW + 归一化"""
    img = cv2.imread(img_path)
    img = cv2.resize(img, (imgsz, imgsz))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    tensor = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    tensor = np.expand_dims(tensor, 0)  # [1, 3, H, W]
    return tensor

def inference(model, model_type, tensor):
    """执行推理，返回(输出, 延迟ms)"""
    t0 = time.time()
    if model_type == 'hrt':
        # 地平线hrt推理
        outputs = model.forward({model.input_names[0]: tensor})
        output = outputs[model.output_names[0]]
    else:
        # ONNX Runtime fallback
        input_name = model.get_inputs()[0].name
        output = model.run(None, {input_name: tensor})[0]
    t1 = time.time()
    return output, (t1 - t0) * 1000

def postprocess(output, conf_thres=0.25, iou_thres=0.45):
    """YOLOv8输出解析 + NMS"""
    if output.ndim == 3:
        output = output[0]

    boxes = output[:, :4]  # xywh
    scores = output[:, 4] * output[:, 5:].max(axis=1)
    class_ids = output[:, 5:].argmax(axis=1)

    # xywh -> xyxy
    boxes_xyxy = np.zeros_like(boxes)
    boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    # NMS
    indices = cv2.dnn.NMSBoxes(
        boxes_xyxy.tolist(), scores.tolist(),
        score_threshold=conf_thres, nms_threshold=iou_thres
    )

    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            results.append({
                'box': boxes_xyxy[i].tolist(),
                'score': float(scores[i]),
                'class_id': int(class_ids[i])
            })
    return results

def main():
    args = parse_args()
    os.makedirs(args.output, exist_ok=True)

    # 获取测试图片列表
    images = [os.path.join(args.data, f) for f in os.listdir(args.data)
              if f.endswith(('.jpg', '.png', '.jpeg'))][:args.num_images]
    print(f"评估图片数: {len(images)}")

    results_table = []

    for model_path in args.models:
        model_name = os.path.basename(model_path)
        print(f"\n评估模型: {model_name}")

        model, model_type = load_model(model_path)

        latencies = []
        all_detections = []

        for i, img_path in enumerate(images):
            tensor = preprocess(img_path, args.imgsz)
            output, latency = inference(model, model_type, tensor)
            dets = postprocess(output)

            latencies.append(latency)
            all_detections.extend(dets)

            if (i + 1) % 20 == 0:
                print(f"  进度: {i+1}/{len(images)}")

        # 统计指标
        avg_latency = np.mean(latencies)
        p99_latency = np.percentile(latencies, 99)
        model_size = os.path.getsize(model_path) / 1024 / 1024  # MB

        result = {
            'model': model_name,
            'model_size_mb': round(model_size, 2),
            'avg_latency_ms': round(avg_latency, 1),
            'p99_latency_ms': round(p99_latency, 1),
            'num_detections': len(all_detections),
        }
        results_table.append(result)
        print(f"  平均延迟: {avg_latency:.1f}ms, P99: {p99_latency:.1f}ms")

    # 生成Markdown报告
    report_path = os.path.join(args.output, 'quantization_report.md')
    with open(report_path, 'w') as f:
        f.write('# 量化精度对比报告\n\n')
        f.write('| 指标 |')
        for r in results_table:
            f.write(f" {r['model']} |")
        f.write('\n|------|')
        for _ in results_table:
            f.write('------|')
        f.write('\n')

        for label, key in [('模型大小(MB)', 'model_size_mb'),
                           ('平均延迟(ms)', 'avg_latency_ms'),
                           ('P99延迟(ms)', 'p99_latency_ms'),
                           ('检测目标数', 'num_detections')]:
            f.write(f'| {label} |')
            for r in results_table:
                f.write(f" {r[key]} |")
            f.write('\n')

        f.write('\n## 分析结论\n\n')
        f.write('- 待补充：填入实际mAP数据和精度损失分析\n')

    print(f"\n报告已生成: {report_path}")

if __name__ == '__main__':
    main()
