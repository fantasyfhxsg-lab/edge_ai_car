#!/bin/bash
# 配置CAN接口
# 用法: sudo bash setup_can.sh [can0] [500000]
CAN_IF=${1:-can0}
BITRATE=${2:-500000}

echo "配置 $CAN_IF @ ${BITRATE}bps"
sudo ip link set $CAN_IF down 2>/dev/null
sudo ip link set $CAN_IF type can bitrate $BITRATE
sudo ip link set $CAN_IF up
echo "配置完成，用 'candump $CAN_IF' 测试"
