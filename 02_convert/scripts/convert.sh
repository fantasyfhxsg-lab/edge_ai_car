#!/bin/bash
# 地平线模型转换脚本
# 用法：bash convert.sh [int8|fp16|fp32]
set -e

# 量化类型参数，默认int8
QUANT_TYPE=${1:-int8}
CONFIG="configs/yolov8n_config.yaml"
OUTPUT_DIR="output_${QUANT_TYPE}"

echo "============================================"
echo "  地平线模型转换"
echo "  量化类型: ${QUANT_TYPE}"
echo "  输出目录: ${OUTPUT_DIR}"
echo "============================================"

# 检查校准集是否存在且非空
if [ ! -d "calib_data" ] || [ -z "$(ls -A calib_data 2>/dev/null)" ]; then
    echo "[错误] 校准集 calib_data/ 不存在或为空"
    echo "请放入200~500张真实场景图片用于量化校准"
    exit 1
fi

# 检查ONNX模型是否存在
if [ ! -f "../01_train/best.onnx" ]; then
    echo "[错误] 未找到 ../01_train/best.onnx"
    echo "请先运行 01_train/export_onnx.py 导出ONNX模型"
    exit 1
fi

# 创建输出目录
mkdir -p ${OUTPUT_DIR}

# 执行模型转换
# 具体命令以你OE版本文档为准，典型命令：
# hb_mapper makertbin --config xxx.yaml --output-dir xxx
echo ""
echo "[1/2] 执行模型转换 + ${QUANT_TYPE} 量化..."
hb_mapper makertbin \
    --config ${CONFIG} \
    --output-dir ${OUTPUT_DIR} \
    2>&1 | tee ${OUTPUT_DIR}/convert.log

echo ""
echo "[2/2] 转换完成，输出文件："
ls -lh ${OUTPUT_DIR}/

echo ""
echo "============================================"
echo "  转换成功！"
echo "  模型文件: ${OUTPUT_DIR}/yolov8n_car.bin"
echo "============================================"
