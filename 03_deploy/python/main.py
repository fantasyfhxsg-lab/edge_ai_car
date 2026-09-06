#!/usr/bin/env python3
"""
主程序：感知→决策→CAN控制 完整闭环（Python版）
用法：python3 main.py --model yolov8n_car.bin --can can0
"""
import argparse
import time
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inference import load_model, preprocess, postprocess, inference, draw_results
from aeb_controller import AEBController, estimate_distance
from can_bridge import CANBridge
import cv2

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--can', default='can0')
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--focal-length', type=int, default=800)
    parser.add_argument('--no-can', action='store_true')
    parser.add_argument('--noshow', action='store_true')
    args = parser.parse_args()

    model, model_type = load_model(args.model)
    aeb = AEBController(safe_distance=2.0, danger_distance=1.0)
    can_bridge = None
    if not args.no_can:
        can_bridge = CANBridge(channel=args.can)
        can_bridge.start()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("\n=== AEB系统启动 ===")
    frame_count = 0
    prev_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break
            tensor = preprocess(frame, args.imgsz)
            output, infer_latency = inference(model, model_type, tensor)
            results = postprocess(output, args.imgsz)

            # 距离估计（取最近障碍物）
            min_distance = None
            for r in results:
                box_h = r['box'][3] - r['box'][1]
                dist = estimate_distance(r['class_id'], box_h, args.focal_length)
                if dist and (min_distance is None or dist < min_distance):
                    min_distance = dist
                    r['distance'] = dist

            # AEB决策
            brake, speed_scale, state = aeb.update(min_distance, valid=True)

            # CAN控制
            if can_bridge:
                can_bridge.send_control(target_speed=int(50 * speed_scale), steering=0, brake=brake)

            # 绘制
            h, w = frame.shape[:2]
            for r in results:
                x1,y1,x2,y2 = [int(v*w/args.imgsz) for v in r['box']]
                color = (0,0,255) if brake else (0,255,0)
                cv2.rectangle(frame, (x1,y1),(x2,y2), color, 2)
                dist_str = f" {r.get('distance','?')}m" if 'distance' in r else ""
                cv2.putText(frame, f"{r['class_name']} {r['score']:.2f}{dist_str}", (x1,y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            fps = 1.0/(time.time()-prev_time); prev_time = time.time()
            info = [f"State: {state.value}",
                    f"Dist: {min_distance:.2f}m" if min_distance else "Dist: --",
                    f"Brake: {brake}", f"Infer: {infer_latency:.1f}ms", f"FPS: {fps:.1f}"]
            for i, line in enumerate(info):
                cv2.putText(frame, line, (10, 25+i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            if brake:
                cv2.rectangle(frame, (0,0),(w-1,h-1), (0,0,255), 5)

            if not args.noshow:
                cv2.imshow('AEB', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"帧#{frame_count} 状态={state.value} 距离={min_distance} 制动={brake}")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        cap.release()
        if can_bridge:
            can_bridge.send_control(target_speed=0, brake=True); time.sleep(0.1); can_bridge.stop()
        cv2.destroyAllWindows()
        print("=== AEB系统已停止 ===")

if __name__ == '__main__':
    main()
