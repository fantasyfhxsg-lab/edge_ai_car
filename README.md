# 主要：边缘AI部署+寄存器版32
# 边缘AI部署小车项目

> 基于地平线RDK X3 + STM32F407的边缘AI部署实战项目
> 核心目标：完整实践"模型训练→转换→量化→优化→部署→系统集成"全流程

sudo chown -R fantasy:fantasy /home/fantasy/workspace2/edge_ai_car

## 硬件配置
| 模块 | 型号 | 说明 |
|------|------|------|
| AI计算 | 地平线 RDK X3 | 5TOPS BPU，运行YOLO推理 |
| 主控 | STM32F407 | 电机控制 + CAN通信 + 舵机转向 |STM32F407ZGT6
| 摄像头 | USB 1080p | /dev/video0，640x480 30fps |
| 通信 | CAN 500kbps | USB-CAN适配器(candleLight) |
| 底盘 | 阿克曼 | 后轮驱动 + 前轮舵机转向 |
| 电机 | 直流减速电机 | 带编码器，PID速度闭环 |JGB37-520减速电机
| 舵机 | MG996R | 前轮转向，PWM控制 |

TB6612FNG直流电机驱动模块

![alt text](image.png)
![alt text](image-1.png)

## 系统架构
```
地平线AI板（感知+决策）                 STM32（执行控制）
┌──────────────────────┐   CAN总线    ┌──────────────────────┐
│ 摄像头采集(V4L2)      │◄───────────►│ CAN接收/发送          │
│ 预处理(C++/NEON)      │   500kbps   │ 整车状态机            │
│ YOLOv8n INT8推理(BPU) │             │ 电机PID闭环控制       │
│ 后处理+距离估计        │             │ 舵机转向控制          │
│ AEB决策状态机         │             │ 心跳监控/故障降级     │
│ 日志系统(时间戳)       │             │ 编码器测速            │
└──────────────────────┘             └──────────────────────┘
```

## 快速开始
```bash
# 1. 构建开发镜像
docker build -t edge-ai-dev:v1.0 -f Dockerfile.dev .

# 2. 进入开发容器
docker run -it --rm -v $(pwd):/workspace --network host edge-ai-dev:v1.0

# 3. 按 docs/ 下的教程逐步执行
```

## 目录说明
| 目录 | 说明 |
|------|------|
| 01_train/ | 模型训练 + ONNX导出 |
| 02_convert/ | 地平线模型转换 + 量化 + 精度评估 |
| 03_deploy/python/ | Python快速验证版（推理+AEB+CAN） |
| 03_deploy/cpp/ | C++性能优化版（V4L2+NEON+多线程流水线） |
| 03_deploy/hil/ | HIL在环测试框架 + 测试用例 |
| 04_stm32/ | STM32固件（CAN协议+电机PID+舵机+状态机） |
| 05_system/ | 日志系统 + 故障注入工具 |
| 06_reports/ | 实验报告模板（量化/Profiling/延迟） |
| config/ | 硬件配置 + 系统参数 |
| docs/ | 详细教程文档 |
| scripts/ | 构建/部署/烧录脚本 |
| models/ | 模型文件（Git LFS管理） |

## 关键指标目标
- 模型：YOLOv8n INT8，输入416x416
- 推理延迟：< 40ms（地平线X3）
- 端到端延迟：< 70ms（摄像头→制动执行）
- 量化精度损失：< 3% mAP
- 连续运行：8小时无崩溃、无内存泄漏

## cline使用
## 以后重启流程

1. Windows 开 Clash
2. VSCode 打开项目文件夹
3. 右下角点 **"Reopen in Container"**（或 `Ctrl+Shift+P` → `Dev Containers: Reopen in Container`）
4. 等容器启动完，Cline 直接用

> 
> settings.json 里的代理配置已经永久生效了，不用每次改。
> 
> 
> 如果哪天 IP 变了（`172.23.144.1` 不通了），在 WSL 里查新 IP：
> 
> 
> ```
> cat /etc/resolv.conf | grep nameserver
> ```
> 
> 
> 然后改 `devcontainer.json` 里的 IP，再 Rebuild 一次。
cat /etc/resolv.conf | grep nameserver
export http_proxy="http://172.23.144.1:7897"
export https_proxy="http://172.23.144.1:7897"
curl -x http://172.23.144.1:7897 -I https://www.google.com --connect-timeout 5

# 备份 /workspace2/04_stm32 整个文件夹（含内部所有文件）：
bash
cd /workspace2 && tar czvf 04_stm32_backup_$(date +%Y%m%d_%H%M%S).tar.gz 04_stm32

# 查看备份内容（确认文件都在）
tar tzf 04_stm32_backup_20260927_1430.tar.gz | head

# 恢复到 /workspace2（会覆盖同名文件，恢复前确认）
cd /workspace2 && tar xzvf 04_stm32_backup_20260927_1430.tar.gz


备份整个 04_stm32 文件夹（含 build、缓存，全部内容）：
bash
cd /workspace2 && BK=04_stm32_backup_full_$(date +%Y%m%d_%H%M%S).tar.gz && tar czvf "$BK" 04_stm32 && ls -lh "$BK"
不带任何 --exclude，文件夹内所有文件（源码 + build + .cache + .clangd）全部打包
备份包生成在 /workspace2/ 根目录，文件名带时间戳
验证完整性：
bash
tar tzf /workspace2/04_stm32_backup_full_*.tar.gz | wc -l   # 文件总数
tar tzf /workspace2/04_stm32_backup_full_*.tar.gz | grep '/build/' | head   # 确认 build 在包内