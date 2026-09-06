#!/bin/bash
# C++版构建脚本
set -e
cd "$(dirname "$0")/../03_deploy/cpp"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
echo "构建完成: $(pwd)/aeb_system"
