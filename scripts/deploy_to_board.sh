#!/bin/bash
# 部署到地平线板
# 用法: bash deploy_to_board.sh <板子IP>
set -e
BOARD_IP=${1:-192.168.1.10}
BOARD_USER=root
BOARD_DIR=/userdata/edge_ai

echo "=== 部署到地平线板 $BOARD_IP ==="

# 创建目录
ssh $BOARD_USER@$BOARD_IP "mkdir -p $BOARD_DIR"

# 传输模型和Python代码
scp 02_convert/output_int8/yolov8n_car.bin $BOARD_USER@$BOARD_IP:$BOARD_DIR/
scp 03_deploy/python/*.py $BOARD_USER@$BOARD_IP:$BOARD_DIR/

# 传输C++可执行文件（如果已构建）
if [ -f 03_deploy/cpp/build/aeb_system ]; then
    scp 03_deploy/cpp/build/aeb_system $BOARD_USER@$BOARD_IP:$BOARD_DIR/
fi

echo "=== 部署完成 ==="
echo "板端运行: ssh $BOARD_USER@$BOARD_IP 'cd $BOARD_DIR && python3 main.py --model yolov8n_car.bin --can can0'"
