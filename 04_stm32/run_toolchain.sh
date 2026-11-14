#!/usr/bin/env bash
# 启动地平线工具链容器+stm32并挂载项目根目录
# 用法: bash 04_stm32/run_toolchain.sh
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${TOOLCHAIN_IMAGE:-autoedge-toolchain:v2}"
# if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
#   echo "镜像不存在，先构建: docker build -f docker/Dockerfile.rdk -t ${IMAGE} docker/"
#   exit 1
# fi
docker run -it --rm \
  --privileged \
  --gpus all --shm-size=15g \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "${PROJECT_ROOT}":/workspace2 \
  --entrypoint /bin/bash "${IMAGE}"
