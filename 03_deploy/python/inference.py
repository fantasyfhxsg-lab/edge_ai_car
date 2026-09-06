#!/usr/bin/env python3
"""
地平线板端 YOLOv8n 推理（Python 快速验证版）
用法：python3 inference.py --model yolov8n_car.bin --image test.jpg
      python3 inference.py --model yolov8n_car.bin --camera 0
"""
import argparse
import time
import numpy as np
import cv2

CLASS_NAMES = ['person', 'car', 'cone']
COLORS = [(0, 255, 0), (255, 0, 0), (0, 255, 255)]

def load_model(model_path):
    """加载地平线模型，返回(model, 类型字符串)"""
    try:
        from horizon_tc_ui import hrt
        model = hrt.load_model(model_path)
        print(f"[模型] 地平线hrt加载: {model_path}")
        print(f"  输入: {model.input_names}")
        print(f"  输出: {model.output_names}")
        return model, 'hrt'
    except ImportError:
        print("[警告] 未找到地平线hrt，使用ONNX Runtime")
        import onnxruntime as ort
        model = ort.InferenceSession(model_path.replace('.bin', '.onnx'))
        return model, 'ort'

def preprocess(img, imgsz=416):
    """预处理: resize→BGR2RGB→HWC2CHW→/255→增加batch维"""
    img = cv2.resize(img, (imgsz, imgsz))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    tensor = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    return np.expand_dims(tensor, 0)

def postprocess(output, imgsz=416, conf_thres=0.25, iou_thres=0.45):
    """YOLOv8输出解析 + NMS"""
    if output.ndim == 3:
        output = output[0]
    boxes = output[:, :4]
    scores = output[:, 4] * output[:, 5:].max(axis=1)
    class_ids = output[:, 5:].argmax(axis=1)
    # xywh -> xyxy
    boxes_xyxy = np.zeros_like(boxes)
    boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    indices = cv2.dnn.NMSBoxes(boxes_xyxy.tolist(), scores.tolist(), conf_thres, iou_thres)
    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            cls_id = int(class_ids[i])
            results.append({
                'box': boxes_xyxy[i].tolist(), 'score': float(scores[i]),
                'class_id': cls_id,
                'class_name': CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else str(cls_id)
            })
    return results

def inference(model, model_type, tensor):
    """执行推理，返回(输出, 延迟ms)"""
    t0 = time.time()
    if model_type == 'hrt':
        outputs = model.forward({model.input_names[0]: tensor})
        output = outputs[model.output_names[0]]
    else:
        input_name = model.get_inputs()[0].name
        output = model.run(None, {input_name: tensor})[0]
    return output, (time.time() - t0) * 1000

def draw_results(img, results, latency, imgsz=416, fps=None):
    """在图像上绘制检测结果"""
    h, w = img.shape[:2]
    for r in results:
        x1, y1, x2, y2 = [int(v * w / imgsz) for v in r['box']]
        color = COLORS[r['class_id'] % len(COLORS)]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, f"{r['class_name']} {r['score']:.2f}", (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    info = f"infer: {latency:.1f}ms" + (f"  FPS: {fps:.1f}" if fps else "")
    cv2.putText(img, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
    return img

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--image')
    parser.add_argument('--camera', type=int)
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--noshow', action='store_true')
    args = parser.parse_args()
    model, model_type = load_model(args.model)
    if args.image:
        img = cv2.imread(args.image)
        tensor = preprocess(img, args.imgsz)
        for _ in range(3): inference(model, model_type, tensor)  # 预热
        latencies = [inference(model, model_type, tensor)[1] for _ in range(50)]
        print(f"平均延迟: {np.mean(latencies):.1f}ms, P99: {np.percentile(latencies,99):.1f}ms")
        output, lat = inference(model, model_type, tensor)
        results = postprocess(output, args.imgsz)
        print(f"检测到 {len(results)} 个目标")
        img = draw_results(img, results, lat, args.imgsz)
        if not args.noshow:
            cv2.imshow('result', img); cv2.waitKey(0)
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        prev = time.time()
        while True:
            ret, frame = cap.read()
            if not ret: break
            tensor = preprocess(frame, args.imgsz)
            output, lat = inference(model, model_type, tensor)
            results = postprocess(output, args.imgsz)
            fps = 1.0 / (time.time() - prev); prev = time.time()
            frame = draw_results(frame, results, lat, args.imgsz, fps)
            if not args.noshow:
                cv2.imshow('result', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
        cap.release(); cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
