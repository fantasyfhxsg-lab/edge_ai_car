# edge_ai_car 小车工程 · Cline 项目规则

> 本文件由 Cline 自动加载，作为本工作区所有代码任务的行为基准。
> 遵守这里的规则比"完成任务"本身更重要——本工程是车载嵌入式求职作品集，规则本身就是面试要讲的东西。

---

## 1. 项目是什么

STM32F407ZGT6 智能小车 + 地平线 RDK X3 边缘 AI 的端到端部署项目（车载边缘 AI / 车载嵌入式方向）。

```
地平线 RDK X3（感知+决策）                STM32F407ZGT6（执行控制）
┌────────────────────┐      CAN 500kbps     ┌────────────────────┐
│ YOLOv8n INT8 推理   │◄───────────────────►│ CAN1 收发(PA11/12)  │
│ AEB 决策状态机       │  0x100/0x101/0x102  │ 10/20/100ms 任务调度 │
│ can_bridge.py       │   /0x103 心跳+控制    │ 电机/舵机/编码器     │
└────────────────────┘                      │ 心跳超时刹车(失效保护)│
                                            └────────────────────┘
```

### 子工程与角色（改代码前先确认在改哪个）
| 目录 | 角色 | 状态 |
|---|---|---|
| `04_stm32_fixed/` | **主固件**：寄存器级裸机，CAN 遥控+心跳+编码器+电机/舵机 | ✅ 当前权威版本，改这个 |
| `plain_can_main/` | 纯 CAN 收发测试版（0x111 周期发送+自收自发），168MHz PLL | 测试用，勿并入主固件 |
| `motion_demo_main/` | 纯运动 demo 版 | 测试用 |
| `edge_ai_car_stm32_demo/` | 早期演示版（探索者板），与 fixed 接线/时钟不同 | 参考，勿作为修改基线 |
| `car_edge_ai/car_edge_ai/` | AI 侧：训练→ONNX→地平线转换量化→板端部署→HIL | 其中 `03_deploy/python/can_bridge.py` 与固件 CAN 协议**强同步** |
| `deploy_stm32.sh`、`edge_ai_setup*.sh` | 环境/部署脚本 | 只读参考 |

### 硬件事实（M144Z-M4 正点原子 STM32F407 Mini 板，LQFP144）
- 主控 STM32F407ZGT6；TB6612FNG ×1；JGB37-520 编码电机 ×2；MG996R 舵机 ×1；CAN 收发器 TJA1050（外接，板上无）
- 时钟：**本板 HSE 8MHz 不起振 → 主固件用 HSI 16MHz，不启 PLL**（SYSCLK/AHB/APB1=16MHz，APB2=1MHz(/16)）。`plain_can_main` 是 HSE+PLL 168MHz 的另一套时钟，两套代码**不要互相复制时钟段**
- 引脚（主固件）：TIM3_CH1/2=PA6/PA7→TB6612 PWMA/B；AIN1/2=PB0/PB1，BIN1/2=PB4/PB5，STBY=PB3（**必须拉高**）；TIM1_CH1=PA8→舵机；USART1=PA9/PA10（板载 CH340，115200 8N1）；CAN1=PA11/PA12；编码器左=TIM4(PD12/13)、右=TIM8(PC6/7)；LED0=PF9 低电平点亮
- 编译目标：`build/car.elf`、`build/car.bin`，链接脚本 `startup/stm32f407zgtx_flash.ld`

---

## 2. 构建与烧写（WSL 内执行）

```bash
cd 04_stm32_fixed
make          # 编译：arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -std=gnu11
              # 必须 0 警告通过（-Wall -Wextra）
make clean
make flash    # openocd 烧写（ST-Link V2）：program build/car.elf verify reset exit
make flash2   # 备用：st-flash --reset write build/car.bin 0x08000000
```

- 工具链：`gcc-arm-none-eabi make openocd`（或 stlink-tools）
- 链接参数 `-specs=nano.specs -lc -lm`：**main.c 里的 `_sbrk()` 是 newlib 堆支持，绝对不能删**
- `-ffunction-sections -fdata-sections --gc-sections`：未引用的驱动会被丢弃（plain_can_main 就是这么复用的）
- 改动任何文件后，交付前必须 `make clean && make`，以"0 warning 0 error"为验收线

---

## 3. 分层与模块契约（不许破坏）

```
main.c（应用+调度+平台服务）
  └─ 调用 Inc/*.h 抽象接口（can_driver / motor_driver / servo_driver / encoder）
       └─ 寄存器操作只允许在驱动层和 main.c 平台段，业务代码禁止直接碰寄存器
            └─ Core/Inc/stm32f407_regs.h（寄存器定义，勿删勿改语义）
```

### 公开 API（新代码沿用，不改签名）
| 模块 | 关键接口 | 说明 |
|---|---|---|
| CAN | `Can_Init/Can_Send/Can_ProcessRx/Can_RegisterRxCallback/Can_IsBusOff/Can_GetTxErrorCount/Can_GetRxErrorCount/Can_SendStatus/Can_SendHeartbeat` | 接收为 **10ms 轮询**，回调签名 `void(*)(uint16 id, const uint8* data, uint8 len)` |
| 电机 | `Motor_SetSpeed(sint16)/Motor_SetSpeedLR/Motor_EmergencyBrake/Motor_ReleaseBrake/Motor_GetSpeed/Motor_GetState/Motor_SelfTest` | speed -1000~1000；内部 20ms 斜坡限幅 |
| 舵机 | `Servo_SetAngle(sint8)/Servo_Init/Servo_SelfTest` | ±30° 映射，50Hz |
| 编码器 | `Encoder_GetCount/GetDelta/GetSpeedRPM/GetSpeedCmPerS/Encoder_Reset/SelfTest`，右轮 `_Right` 后缀 | ENCODER_PPR=836（11×19×4） |
| 平台 | `millis()/delay_ms()/uart_putc/uart_puts/uart_printf` | 见下方 uart_printf 警告 |

### 调度器契约
- 主循环是**非阻塞时间片轮询**：`Task_10ms`（CAN 轮询+回 0x101）、`Task_20ms`（编码器测速+执行控制）、`Task_100ms`（心跳监控+回 0x103+LED 闪）
- **任务函数内禁止任何 `delay_ms()` 阻塞**（历史事故：曾阻塞 30ms 导致 CAN 丢帧，周期被拉到 40~50ms）
- 中断与主循环共享的变量必须 `volatile`（如 `g_ticks`、`g_control`）

---

## 4. CAN 协议契约（改动前必须读，改动后必须同步 AI 侧）

波特率 500kbps，标准帧，多字节字段**小端**。

| ID | 方向 | 周期 | 内容 |
|---|---|---|---|
| 0x100 | AI→STM32 | 20ms | 控制帧：byte0 控制字(0x01=ENABLE)，byte1 目标速度(-100~100)，byte2 转向(-100~100，正=右转)，byte3 制动(0x01=急刹)，byte4~7 保留 |
| 0x101 | STM32→AI | 20ms | 状态帧：byte0 状态字，byte1 当前速度，byte2 电压×10，byte3 故障码(位掩码)，byte4~7 左右编码器计数 hi/lo |
| 0x102 | AI→STM32 | 1s | AI 心跳（byte0 计数器） |
| 0x103 | STM32→AI | 1s | STM32 心跳（byte0 计数器） |

- 故障码位掩码：`FAULT_NONE=0x00 / FAULT_AI_HEARTBEAT=0x01 / FAULT_MOTOR_LEFT=0x02 / FAULT_MOTOR_RIGHT=0x04 / FAULT_LOW_BATTERY=0x08 / FAULT_ENCODER=0x10`
- 状态字：`STATUS_NORMAL=0 / WARN=1 / BRAKE=2 / FAULT=3`
- **修改任何 ID、字节布局、长度、周期，必须同步改 `car_edge_ai/car_edge_ai/03_deploy/python/can_bridge.py` 和 `can_protocol.h`，并在注释里写协议版本说明**
- 失效保护：收到过 0x102 后启用 500ms 超时监控；超时 → `STATUS_FAULT + FAULT_AI_HEARTBEAT + Motor_EmergencyBrake()`；故障清除逻辑已实现，改动时保持"刹车优先、可恢复"语义

---

## 5. 硬性约束（绝对不做）

1. **主固件禁止引入 CubeMX 生成代码 / HAL 依赖**。本工程卖点就是"寄存器级裸机、无 HAL"，面试要能讲清 HAL 与寄存器的取舍。问 HAL 问题时可以对比讲解，但代码不改
2. 任务函数内禁止阻塞延时（见上）
3. **`uart_printf` 不做格式化**：`nano.specs` 下 `vsnprintf` 遇非法格式符会 HardFault（历史事故，注释里有）。它等价于 `uart_puts`。要打数字用字符串拼接或自写简易 itoa，不要改回 vsnprintf
4. 时钟段不要跨变体复制（HSI 16MHz 版 vs HSE/PLL 168MHz 版寄存器值完全不同）
5. `SYSCFG->PMC` 禁用 JTAG（PB3/PB4 被 TB6612 占用）必须在**任何 GPIO 配置之前**；`VTOR=0x08000000` 必须在使能中断前
6. TIM1（高级定时器）必须置 `BDTR.MOE`，否则舵机无输出
7. STBY(PB3) 不拉高 → TB6612 整片待机，电机不动（接线问题排查第一顺位）
8. 用 CAN1(PA11/12) 时不要插板载 USB SLAVE 口（OTG_FS 引脚冲突）
9. 不要删 `_sbrk`、startup 汇编、链接脚本；`stm32f407_regs.h` 是唯一寄存器权威定义
10. 类型风格：沿用 AUTOSAR 风格别名 `uint8/sint16/uint32/boolean`（platform.h 定义），新代码保持一致，不要同文件混用裸 `stdint_t` 写法

---

## 6. 编码规范（延续工程既有风格）

- 每个文件头注释：硬件连接表 / 数据流图 / "为什么这么做"（本项目注释习惯是解释原因，如"为什么不用 PLL"）
- 中文注释，写明数值推导（如 `PSC/ARR → 20kHz`、`BRR=0x008B → 115200`）
- 分层调用：业务代码只调用 `Motor_*/Can_*/Servo_*/Encoder_*` 抽象接口；寄存器位操作只在驱动层
- 失败处理：驱动返回 `boolean` 或设置故障位；失效默认动作是**刹车**（failsafe）
- 命名：`Task_10ms`、`MX_TIM3_PWM_Init`（寄存器初始化）、`App_CanRxCallback`（应用回调）、`g_` 前缀全局变量、`static` 优先
- 自检函数（`*_SelfTest`）保持存在，上电顺序：平台 → 外设 → 驱动 → 注册回调 → 自检 → Demo → 主循环

---

## 7. 车载工程化上下文（改代码时带着这些概念）

- 心跳监控 = 功能安全里的"通信超时监控"，面试要能讲：为什么收到第一帧心跳才启用保护
- 故障码位掩码 = 多故障叠加上报；状态字 = 整车状态机（NORMAL/WARN/BRAKE/FAULT）
- Bus-Off 监控（`Can_IsBusOff`）是 CAN 节点必备恢复逻辑
- 往 UDS / CAN-FD 迁移的路径：协议层已经隔离在 `can_protocol.h + can_driver.h`，后续加 UDS 诊断服务不要污染控制帧结构
- 用户求职方向是车载边缘 AI / 车载嵌入式，涉及方案对比时（HAL vs 寄存器、轮询 vs 中断、开环 vs PID），主动给出两者的取舍和本工程的取舍理由

---

## 8. AI 助手工作规范（对本 Cline 的要求）

1. **动手前先读**：改哪个模块，先读对应 `Inc/*.h` 和相关 `.c`，再读 `main.c` 中该模块的调用处；不确定改哪个变体先问
2. **改完必须验证**：`cd 04_stm32_fixed && make clean && make`，0 警告 0 错误；涉及烧写说明时给出 `make flash` 命令
3. 涉及 CAN 协议的任何改动，交付时**主动提醒**需要同步 `can_bridge.py`
4. 回答用简体中文，简洁、给结论；解释寄存器/HAL 取舍时用"本工程选择 + 面试怎么说"结构
5. 用户问"为什么/能不能改"时，先讲清原理和风险，再动手；**未经确认不重排引脚、不改协议、不换时钟方案**
6. 不编造寄存器地址或时序数值——拿不准就查 `stm32f407_regs.h` 或参考手册，禁止臆造
7. 复用已有文件优先于新建文件；新文件放入对应子工程目录，命名见名知义

---

## 9. 交付前验证清单

- [ ] `make clean && make` 0 警告 0 错误
- [ ] 改动是否触碰 CAN 协议（ID/字节/周期）→ 是否已同步 `can_bridge.py`
- [ ] 是否在任务函数里引入了阻塞延时
- [ ] 是否误引入了 HAL/CubeMX 依赖或跨变体复制了时钟代码
- [ ] 新增全局共享变量是否加了 `volatile`
- [ ] 改动后 0x100→运动、0x101/0x103 上报、心跳超时刹车三条链路是否仍然成立
