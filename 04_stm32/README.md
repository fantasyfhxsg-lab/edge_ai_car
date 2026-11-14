# 04_stm32 固件说明（v2.1 - M144Z-M4 正点原子 STM32F407 Mini板）

主控：STM32F407ZGT6（LQFP144，全部引脚引出）
板载：USB转串口(CH340↔USART1)、USB SLAVE(PA11/PA12)、LED0=PF9/LED1=PF10、
      TFTLCD接口(FSMC)、OLED/摄像头接口(DCMI)、TF卡、SPI Flash、EEPROM、SRAM

# WSL 里访问 Windows 的下载文件夹，复制到工程目录
cp /mnt/c/Users/<你的Windows用户名>/Downloads/.clinerules ~/edge_ai_car/


## 硬件连接（M144Z-M4）

### TB6612FNG（电机驱动）
| TB6612 | STM32 | 说明 |
|--------|-------|------|
| PWMA   | PA6   | 左轮 PWM (TIM3_CH1, 20kHz) |
| AIN1   | PB0   | 左轮方向 |
| AIN2   | PB1   | 左轮方向 |
| PWMB   | PA7   | 右轮 PWM (TIM3_CH2, 20kHz) |
| BIN1   | PB4   | 右轮方向 |
| BIN2   | PB5   | 右轮方向 |
| STBY   | PB3   | 必须接高（悬空整片不工作！） |
| VM     | 电池+（≤13.5V，建议 3S 11.1V） | 电机电源 |
| GND    | 电池- / 系统共地 | 必须与 STM32 共地 |

### 电机（JGB37-520 ×2）
| 电机线 | 接法 |
|--------|------|
| 红/白（电机两根动力线） | TB6612 AO1/AO2（左轮）、BO1/BO2（右轮） |
| 蓝/黑（编码器电源） | 3.3V / GND（编码器支持 3.3-5V） |
| 黄/绿（编码器 A/B） | 左轮: PD12/PD13(TIM4)；右轮: PC6/PC7(TIM8) |

> 动力线红白对调 = 电机反转；编码器 A/B 对调 = 计数方向反转。

### 舵机 MG996R
| 舵机线 | 接法 |
|--------|------|
| 橙色(信号) | PA8 (TIM1_CH1) |
| 红色(V+) | 独立 5-6V BEC/降压模块（严禁用 STM32 3.3V，峰值 2A+） |
| 棕色(GND) | 系统共地 |

### CAN（板上无收发器芯片，必须外接 TJA1050 模块）
| STM32 | TJA1050/SN65HVD230 模块 |
|-------|--------------------------|
| PA12 (CAN1_TX) | TXD |
| PA11 (CAN1_RX) | RXD |
| 5V | VCC（**TJA1050 必须 5V**；SN65HVD230 才是 3.3V） |
| GND | GND |
| - | CANH/CANL -> CAN 总线（与 USB-CAN 适配器并联） |

> ⚠ 板载 USB SLAVE 口占用 PA11/PA12（OTG_FS）。用 CAN1 时**不要插 USB SLAVE**。

### 调试串口（用板载 USB，无需外接）
M144Z-M4 板载 CH340 ↔ USART1(PA9/PA10)。跳线/拨码保持"USB串口"位置，
USB 线接板载 USB 口即出日志（115200 8N1）。若拨到"串口1"位置则需外接 USB-TTL 接 PA9/PA10。

### 用户 LED（板载）
LED0=PF9(红)、LED1=PF10(绿)，低电平点亮。代码接口：`platform.h` 的
`Led_Init()`/`Led_Set()`（LED0）与 `Led1_Init()`/`Led1_Set()`（LED1）；BSRR 写与引脚映射
都在 `Src/platform.c`，应用层/调度层不碰寄存器（换板子只改那里的 `LED_PORT/LED_PIN` 与
`LED1_PORT/LED1_PIN`）。

| 分支 | LED0 = PF9(红) | LED1 = PF10(绿) |
|---|---|---|
| **heartbeat_v1**（默认） | RDK 语义：蜂鸣快闪 / 刹停·HOLD 常亮 | **握手状态**：STATIC 灭 / HANDSHAKE 快闪≈200ms / CRUISE(含 STOPPING·HOLD) 慢闪≈1s / HS FAIL 常亮 |
| **brake_v2** | AEB 指示：等级 1 闪烁、等级 2/3 常亮、等级 0 熄灭 | 未使用（保持熄灭） |

> ⚠ 握手状态**不用 LED0**：LED0 已被 RDK 蜂鸣/刹停语义占用（brake_v2 分支还兼作 AEB 指示灯），
> 两者映射会「快闪 vs 快闪、常亮 vs 常亮」打架 → 握手状态改用 **LED1**，语义隔离。
> LED1 闪烁在 20ms 服务里按 `millis()` 相位生成，非阻塞、无 `delay_ms`。

## 编译（WSL）
```bash
sudo apt install gcc-arm-none-eabi make openocd   # 或 stlink-tools
cd 04_stm32
make                              # 默认镜像 = heartbeat_v1（Can_Init + TJA1050，0x100/0x101/0x102）
make PROTOCOL=brake_v2            # 切到 brake_v2（0x123/0x124/0x125）
make CAN_LOOPBACK=1               # 自测镜像 = 静默回环（BTR.LBKM|SILM，0x111 自发自收）
make PROTOCOL=brake_v2 CAN_LOOPBACK=1   # brake_v2 回环自测（⑤-A/⑤-B AEB 自测需 brake_v2）
```
> **CAN 协议编译期二选一（`.clinerules §5`）**：默认 `heartbeat_v1`，`make PROTOCOL=brake_v2`
> 切到 `brake_v2`（等价 `-DCAN_PROTOCOL_HEARTBEAT_V1=1/0 -DCAN_PROTOCOL_BRAKE_V2=0/1`）；
> 两个宏之和必须恰为 1，否则编译期 `#error`。**选中的版本必须与板端 `can.protocol` 一致**。
> ⚠ 切协议/切镜像**不再需要**手动 `clean`：Makefile 用**构建指纹**（`build/.buildflags`，内容=当前
> `CFLAGS/ASFLAGS/LDFLAGS`）让所有 `.o` 都依赖它 —— 参数一变指纹被重写，即**整树重编**（打印
> `[build] build flags changed -> full rebuild`）；参数不变则空跑。手动 `make clean` 仍可用。
> （历史坑：`.o` 规则只跟踪头文件、跟踪不到命令行宏，切变体不 `clean` 会链接出"混合"镜像。）
> 两套镜像只差一个构建参数，**源码不用改**：`Src/main.c` 的 `CAN_LOOPBACK_SELFTEST`
> 由 `#ifndef` 兜底为 `0`（=Normal）；`CAN_LOOPBACK=1` 时 Makefile 注入
> `-DCAN_LOOPBACK_SELFTEST=1`。串口启动行会打印 `[PROTO] heartbeat_v1 ...` / `[PROTO] brake_v2 ...`。

## 烧写（ST-Link V2）
```bash
# 方式 A
make flash
# 等价于:
# openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/car.elf verify reset exit"

# 方式 B（st-link）
make flash2
# st-flash --reset write build/car.bin 0x08000000
```

## 上电行为（协议二选一 × 镜像二选一；规则 §6.1 步骤①②）

| 镜像 | 构建命令 | CAN 模式 | 串口特征 |
|---|---|---|---|
| heartbeat_v1 联调（**默认**） | `make clean && make` | Normal（`Can_Init`） | `[PROTO] heartbeat_v1 (0x100/0x101/0x102)` + `[CAN] init OK: BTR=0x004E0003 ... 500kbps` |
| brake_v2 联调 | `make clean && make PROTOCOL=brake_v2` | Normal（`Can_Init`） | `[PROTO] brake_v2 (0x123/0x124/0x125)` + `[CAN] init OK: BTR=0x004E0003 ... 500kbps` |
| 回环自测 | `make clean && make PROTOCOL=brake_v2 CAN_LOOPBACK=1` | 静默回环（`Can_InitLoopback`，`BTR.LBKM\|SILM`） | `[CAN-LB] init OK: BTR=0xC04E0003 ...` + `[AEB-TEST] parser PASS (8/8)` + 每秒 `PASS` + 每轮 `AEBtest=PASS (10/10)` |

### 回环自测镜像（步骤①）
1. USART1 打印启动信息（115200 8N1，板载 USB 口即 CH340），含编码器自检 `[ENC] self-test = 1`
2. CAN1 静默回环初始化：`BTR=0xC04E0003`（BRP=4 / TS1=15 / TS2=5 / SJW=1 / LBKM=1 / SILM=1）
   —— **不需要** TJA1050、不需要对端、不需要 120Ω 终端电阻
3. 上电自动发一帧 0x111 `{counter,'C','A','N',0,0,0,0}`；`Task_10ms` 里 `Can_ProcessRx()`
   排空 FIFO0 回读该帧（`SILM/LBKM` 属 `CAN_BTR[31:30]`，**不在 MCR**；只置 SILM 是
   "只收不发"，自发自收必须 `LBKM|SILM`）
4. 每秒打印（`tx` 与 `rx` 同步 +1 = 链路通；`bad` 必须恒为 0）：
   `[CAN-LB] PASS  tx=1  rx=1  bad=0  round=0  AEBtest=running/3  TxErr=0  RxErr=0  BusOff=0  Ovf=0`
   `[AEB] lvl=0 latched=0 kmh=0 vL=0 vR=0 dist=none dist_rx=0`
   判据：`PASS` 且 `TxErr=RxErr=BusOff=Ovf=bad=0`；`bad>0` 表示回读到的 0x111
   内容/计数不符（脏帧），`FAIL` 先查 `bad → TxErr/RxErr → BusOff/Ovf`
5. **回环阶段就能验证车速链路**：手转车轮，`vL/vR/kmh` 应随动（km/h = cm/s × 0.036）
   —— 回环镜像里 `Task_20ms` 的周期性 0x125 上报被 `#if !CAN_LOOPBACK_SELFTEST` 屏蔽
   （`kmh=` 直接读 `g_speed_kmh`），这样 ⑤-B 的 0x125 回读不会被周期帧覆盖。

#### ⑤-A 解析器自测（**两种镜像都跑**；零硬件依赖，开机一次）
直接构造字节数组调 `Can_ParseBrakeCmd` / `Can_ParseObstacleDist`，覆盖 8 个契约用例：
0x123 等级 0/1/2/3 必须被接受、等级 4 必须被拒、0x124 小端 `{0x34,0x12} → 4660`、
0x124 `DLC=1` 必须被拒、0x123 `DLC=1`（等级合法）也必须被拒（§5 数据段固定 8 字节）。
收尾行 `[AEB-TEST] parser PASS (8/8)`（失败逐条打印 `FAIL ...`）。
不发帧、不动执行机构，所以 Normal 镜像也安全。

#### ⑤-B AEB 回环自测（**仅 `CAN_LOOPBACK=1`**；不需要 TJA1050，但**会真实驱动电机**）
自发 0x123/0x124/0x125 再回读，断言"解析 + 制动映射 + 失效保护"；10 步/轮、200ms 一步
（`Task_100ms` 每 2 拍推进 1 步），一轮 10 步 = 14 个 200ms 拍 ≈ 2.8s（步 8 多静默 3 拍），
间隔 `AEB_LB_ROUND_MS`=5s 自动重跑。
⚠ **测试时车轮会抱死（等级 3 短路制动）、会减速（等级 2）**：请架空车轮或断开电机动力线。

| 步 | STM32 自发帧 | 断言（只看抽象接口，不碰寄存器） | 期望串口 |
|---|---|---|---|
| 1 | 0x124 = 123cm | `g_obstacle_cm==123 && dist_rx>0` | `[AEB] dist=123cm` |
| 2 | 0x123 level=1 | `lvl==1 && !latched && 非 BRAKE` | `[AEB] level=1 WARN (no brake)` |
| 3 | 0x123 level=2 | `lvl==2 && latched` | `[AEB] level=2 PARTIAL (... latched)` |
| 4 | 0x123 level=3 | `Motor_GetState()==BRAKE && latched` | `[AEB] level=3 EMERGENCY (latched ...)` |
| 5 | 0x123 level=0 | `lvl==0 && !latched && 非 BRAKE` | `[AEB] level=0 RELEASE` |
| 6 | 0x123 level=4（非法） | 状态不变（整帧被拒） | `[AEB] 0x123 malformed frame ignored ...` |
| 7 | 0x124 = 0xFFFF | `g_obstacle_cm==0xFFFF` | `[AEB] dist=none` |
| 8 | 0x123 level=3 → **静默 600ms** | **仍 `latched` 且 `BRAKE`**（④ 失效保护） | `[AEB] 0x123 LOST -> keep brake latched ...` |
| 9 | 0x125 = 123 km/h | 回读小端还原 == 123（且 DLC==8） | — |
| 10 | 0x125 = 0xFFFF | 回读小端还原 == 0xFFFF | — |

轮末额外补发一次 `0x123 level=0`（**安全收尾**，绝不把车留在制动态）；单步失败不中断本轮，
失败出现在 `AEBtest=FAIL step=.. exp=.. act=..`，全过则打印 `AEBtest=PASS (10/10) round=N`。

> 自测组帧只存在于自测镜像：`Can_SendBrakeCmdTest` / `Can_SendObstacleDistTest` 包在
> `#if CAN_LOOPBACK_SELFTEST` 里，**Normal 镜像校验过符号引用数 = 0**（不可能误发 0x123/0x124
> 到真实总线）。自测步进与调度周期由 `#error` 守卫绑定：`AEB_LB_STEP_MS == TASK_100MS_MS*2`。

### 切 Normal（步骤②③）
`make clean && make`（默认即 Normal）→ `make flash`，串口打印
`[CAN] init OK: BTR=0x004E0003 ... 500kbps`。**此时才需要** TJA1050（VCC=5V）、显式共地、
总线两端各 120Ω、CANable 抓包（500kbps）；0x125 从此刻起每 20ms 上报。
步骤③ 才与板端联调 0x123（制动等级）/0x124（距离）（板端代码不由本工程修改）

### AEB 协议行为（与板端 clinerules §5 一一对应，禁止单边改动）
| ID | 方向 | STM32 端行为 |
|---|---|---|
| 0x123 | 板端→STM32 | `byte0` = 等级：0 解除（**唯一解锁路径**）/ 1 预警（LED 闪烁 + 日志，不制动）/ 2 部分制动（20ms 拍按 `RAMP_STEP` 斜坡降到 0 后滑行）/ 3 紧急制动（立即 `Motor_EmergencyBrake()` + 锁存） |
| 0x124 | 板端→STM32 | `byte0-1` 小端 cm，`0xFFFF` = 无有效目标；**只记录并打印**，不驱动动作 |
| 0x125 | STM32→板端 | `byte0-1` 小端 km/h，`0xFFFF` = 无效；周期 20ms（< 板端 recv_timeout 100ms）；左右轮 cm/s 取平均 ×0.036；Bus-Off 期间上报 0xFFFF |

**失效保护（刹车优先、可恢复）**：收到过非 0 等级后，0x123 超时 500ms 只打印
`[AEB] 0x123 LOST -> keep brake latched (no auto release)` 并**保持制动**，仅收到等级 0 才解除。

**左右轮一致性判据**：`|vL-vR| > max(30%, 0.5km/h=13.9cm/s)` 时打印
`[AEB] WHEEL-DIFF warn: vL=.. vR=..`，0x125 仍照发融合值（打滑/单轮卡死只告警，不丢速度）。

> ⚠ 旧的运动 demo（`MotionDemo_*`）已 `#if 0` 留档：内含 27 处 `delay_ms` 阻塞，
> 会挤掉 10ms 轮询导致 CAN 丢帧（规则 §3 历史事故）。改造为非阻塞任务前不要启用。

### 步骤① 真机验证方法（SWD 读回，**不需要串口**）
> ⚠ **优先用 §③ 的 `bash tools/hw_verify.sh`**（真机已跑通，地址 `nm` 现读现用；下表旧地址只作历史记录）。
> 容器里 `openocd` 连 ST-Link 需 `run_toolchain.sh` 带 `--privileged -v /dev/bus/usb`，但该镜像内无 `sudo`/`bash`，
> 真机验证实际在宿主 WSL 跑（见 §③）。
注意 openocd `init` **不会** halt 目标（保持运行），要读冻结态须自己 `halt`，收尾用 `reset run`：
```bash
# ⚠ WSL2 + usbipd-win：/dev/bus/usb/* 属 root(0600)，普通用户会报 LIBUSB_ERROR_ACCESS
#   → 用 sudo 跑烧写/调试（或 usbipd attach + udev 放权）；串口 CH340 也要 attach 才有 /dev/ttyUSB*
make flash CAN_LOOPBACK=1                             # program + verify + reset（自测镜像）
arm-none-eabi-nm -n build/car.elf | grep -E 'g_lb|g_can_test'   # 现读现用：每加一个全局地址都会平移
sleep 3
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c 'init' -c 'sleep 3000' -c 'halt' -c 'reg pc' \
  -c 'mdw 0x20000000 32' -c 'mdw 0x40006400 8' -c 'mdw 0x40023840 1' \
  -c 'resume' -c 'sleep 4000' -c 'halt' -c 'mdw 0x20000000 32' \
  -c 'reset run' -c 'shutdown'
```
判据（地址按**本版回环 elf** 对齐；`mdw 0x20000000 32` = 128B，覆盖 `.data+.bss`
= 0x20000000..0x20000073 全部 `g_*`/文件级 static）：

| 观测点（新地址，`arm-none-eabi-nm -n build/car.elf` 可现读现用） | 期望 | 实测/备注 |
|---|---|---|
| `CAN1->BTR` @0x4000641C | `0xC04E0003`（LBKM\|SILM，即 Can_InitLoopback 生效） | `0xC04E0003` ✅ |
| `CAN1->ESR` @0x40006418 | `0`（TEC/REC/LEC/BOFF 全 0 = **错误计数为 0**） | `0` ✅ |
| `CAN1->TSR` @0x40006408 | `TME0/1/2=1`（TX 邮箱空）+ `TXOK0=1` | `0x1C000003` ✅ |
| `CAN1->MCR` @0x40006400 | `ABOM\|TXFP\|DBF`，`INRQ=0`（运行态） | `0x00010044` ✅ |
| `CAN1->MSR` @0x40006404 | `INAK=0, SLAK=0`，`RX=1`（收到过帧） | `0x00000C08` ✅ |
| `RCC->APB1ENR` @0x40023840 | `CAN1EN(bit25)=1`（+TIM3/4EN=1） | `0x02000006` ✅ |
| `g_can_test_tx_counter` @0x20000034 | 每秒 +1 | 26 → 28 ✅（地址因新增全局平移） |
| `g_can_test_rx_count` @0x20000030 | **与 tx 相等**（自己发的帧回到自己） | 26 → 28 ✅ |
| `g_can_test_pass` @0x2000002C | 窗口内收到过 = 1 | `1` ✅ |
| `g_can_test_echo_bad` @0x20000028 | **恒为 0**（回读帧内容+计数全对） | `0` ✅ |
| `g_can_test_tx_last` @0x20000035 | = 最近一次真正发出的 counter | ✅ |
| `g_ticks` @0x2000006C | 与 tx 自洽（1 帧/秒） | `26193 ms` vs `tx=26` ✅（本轮随平台层拆分移到 `platform.o` 尾部） |
| `g_speed_valid` @0x20000056 | 编码器自检过 = 1 | `1` ✅ |
| `g_brake_level` @0x2000001D / `g_brake_latched` @0x2000001C | 轮间空闲 = 0/0；轮内随步进变化 | 上版为 0x15/0x14（已平移） |
| `g_lb_round` @0x20000044 | ≥1（开机 ~0.5s 起第一轮，之后每 5s +1） | **本版新增** |
| `g_lb_last_round_fail` @0x20000008 | 0（上一轮 10 步全过）；`0xFF` = 还没跑完一轮 | **本版新增** |
| `g_lb_fail` @0x2000003C | 累计失败步数 = 0（对应串口 `AEBtest=PASS`） | **本版新增** |
| `g_lb_step` @0x20000053 | 0 = 轮间空闲；1..10 = 正在跑该步 | **本版新增** |
| `g_lb_speed_echo` @0x20000050 | 最近 0x125 回读还原值（123 或 65535） | **本版新增** |
| `g_lb_speed_echo_ok` @0x20000052 | 1（DLC==8 且已收到） | **本版新增** |
| `pc` | 落在 `main` 内（本轮分层补全后 `main` @**0x08001020**） | `0x08000D3A`/`0x08000C96` 为旧值，代码搬迁后地址已变，以 `nm -n` 现读为准 |

> 同一手法可做步骤②③的离线判据：切 Normal 后读 `g_brake_level`/`g_brake_latched` 即可判定
> 0x123 是否到位（无需串口），读 `g_speed_kmh`/`g_wheel_*_cmps` 可看 0x125 数据源。

## 离线时序审计（不参与构建；改时钟/波特率/定时器/任务后必跑）
```bash
cd 04_stm32 && python3 tools/timing_audit.py    # 退出码 0=全过 / 1=有 FAIL / 2=解析失败
```
覆盖项：SYSCLK / APB1 / APB2 分频（含 **PPRE 编码陷阱**：`0b101` 才是 ÷4）、
CAN 位速率 + 采样点（与 `BTR` 表达式交叉核对，**与绝对波特率无关的回环自测不能替代它**）、
USART BRR 误差、TIM3 PWM / TIM1 舵机频率、SysTick 1ms 内核时钟、
任务函数内禁 `delay_ms`（阻塞）、ISR 写入的共享变量必须 `volatile`、
编码器自由计数（`PSC=0/ARR=0xFFFF`）与 20ms 测速窗口契约、两份头文件波特率声明一致性。
读取落点（本轮改造）：时钟/USART/TIM/SysTick 等**跨文件常量**统一在 `Src` 下全部 `.c` 的
合并正文里查找（`glob` + `all_src`），只有"`Task_*` 内禁 `delay_ms`"仍只读 `main.c`
→ 平台层/驱动层搬文件不会再误报 `[解析失败]`（退出码 2）。

## 已知待修（需用户确认后再动，勿在联调镜像里混入第二个变量）
- `platform.h` `gpio_af()` 的 `pin == 0` 边界（移位宽度/循环）；`can_driver.c` 用成员指针
  算术写 `TDTR/TDLR/TDHR`（寄存器布局已核对正确，但形式上是 UB）。
- ~~Bus-Off 目前只有查询计数与诊断打印（`BusOff=`/`Ovf=`），缺"事件级日志"；
  自动恢复依赖 `MCR` 里已使能的 ABOM（硬件行为），规则 §6.4 要求的可观测性仍需补。~~
  → **本轮已补（真机发现驱动）**：驱动层 `Can_RecoverIfStuck()`（有界 INRQ 往返自愈 +
  `g_recover_cnt` 计数）+ 1s 诊断行 `Rec=` + 事件日志 `[CAN] back on bus after init-mode stall`；
  现象/根因/三条实测证据见 §③「Bus-Off 自愈」。
  （仍待办：`BusOff=` 只反映 `ESR.BOFF` 瞬时值，"进入 bus-off 事件次数"尚未单独计数。）
- ~~平台层实现（SystemClock/SysTick/UART）仍在 `main.c`，未拆出 `Src/platform.c`~~
  → **本轮已补层**：`Src/platform.c` 已建立（见下"本轮新增（分层补全…）"节）。
  仍待补（下一轮，改动会挪动 TIM 启动时机，需单独验证）：`MX_TIM3_PWM_Init` /
  `MX_TIM1_PWM_Init` 仍留在主流程，并入 `Motor_Init()`/`Servo_Init()` 才能让驱动层自足。
- CAN 过滤器仍是"全通"（`F0R1/F0R2 = 0`），未按 0x123/0x124 收紧；
  联调期保留全通便于排查"收不到帧"，联调通过后再评估收紧。
- 帧类型校验（RTR/IDE）未加：`Core/Inc/stm32f407_regs.h` 尚无 `RI0R` 的 RTR/IDE 位宏，
  按规则 §3 不臆造位域，需连同头文件一起补（RM0090 30.9.3）。
- 等级 2 部分制动的减速速率仍沿用驱动层内部斜坡（`RAMP_STEP=80`/20ms 拍 ≈ 4000 单位/s，
  ±1000 满速约 0.25s 降到 0 后滑行）—— **本轮经确认（A1）保持默认**；若要"按时长/按距离"
  限速，属应用层新策略，须先出阈值方案再动（规则 §4.4）。

## 已修复（本轮 CAN 联调准备）
- `motor_driver.c` 速度域 ±1000 直写 `CCR1/2`（`ARR=49` ⇒ ≥50 即满占空比、`RAMP_STEP` 失效）
  → 新增 `speed_to_ccr()` 按 `TIM3->ARR` 现场换算；审计契约项 6 **FAIL → PASS**。
- `Motor_ReleaseBrake()` 只改 `g_state`、不清 IN1/IN2（"假解除"）→ 真正回到滑行态。
- `can_protocol.h` 仍为旧脚手架 ID → 已对齐 §5 的 0x123/0x124/0x125；
  旧 ID 段与 `Can_SendStatus/Can_SendHeartbeat` 一并 `#if 0` 留档停用。
- `Encoder_Init()` 从未被调用（0x125 无数据源）→ 已加入启动流程并打印自检；
  新增右轮 `Encoder_GetSpeedCmPerSRight()`（两轮平均的第二路）。
- `Can_ProcessRx()` 每轮只取 1 帧 → 改为 `while(FMP0)` 排空 FIFO0（深度 3，带 8 次兜底）
  + `Ovf=` 溢出计数观测。
- 调度层新增 `Task_20ms`（两轮测速 → 0x125 / 部分制动服务）与 `Task_100ms`
  （0x123 失效保护观测 / 轮差告警 / 0x124 距离打印 / 等级 1 LED 闪烁）；
  时间基准改累加式（`last += 周期`）以钉住 20ms 测速窗口，落后超一周期则跳拍。

## 本轮新增（⑤ AEB 回环自测 + 日志治理 M1~M3）
- **⑤-A 解析器自测** `Aeb_SelfTestParser()`（应用层，`main.c`）：两种镜像开机各跑一次，8 个契约
  用例（等级 0~3 接受 / 等级 4 拒绝 / 0x124 小端 / 0x123·0x124 的 `DLC≠8` 拒绝）；纯软件，不发帧不动电机。
  先把"字节契约"钉死，避免回环/联调时把解析问题误判成硬件问题。
- **⑤-B AEB 回环自测** `Aeb_LbSelfTest_Service()` / `Aeb_LbBeginStep()` / `Aeb_LbCheckStep()`
  （应用层，仅自测镜像）：`Task_100ms` 每 2 拍推进 1 步（200ms）、10 步/轮、5s 重跑；驱动层新增
  `Can_SendBrakeCmdTest()` / `Can_SendObstacleDistTest()`（包在 `#if CAN_LOOPBACK_SELFTEST`，
  Normal 镜像符号引用数实测 = 0）；轮末补发 `0x123 level=0` 安全收尾。**会真实驱动电机**
  （等级 3 短路制动、等级 2 减速），实测请架空车轮。
- **M1 制动日志改边沿触发**：只有等级变化才打印。板端若 20ms 发一帧同等级，旧写法 ≈20% 调度
  时间耗在串口上；持续观测改由每秒 `[AEB] lvl=.. latched=..` 状态行承担。
- **M2 非法帧日志限流**：`Aeb_LogMalformedFrame()` 每 1s 最多一条（首次必打）。
- **M3 等级 1 丢失单独告警**：`[AEB] 0x123 LOST (warn only, no brake)`；锁存/解锁语义不变
  （仍只有 `level 0` 能解除，超时永不自动解除）。
- **等级 2 补 LED 常亮**：与 README "等级 2/3 常亮" 对齐（原实现只有等级 3 常亮）。
- **编译期守卫**：`main.c` 里 `#if (TASK_100MS_MS * 2u) != AEB_LB_STEP_MS` → `#error`，
  防止任务周期与自测步进静默失配（已实测：故意把 `AEB_LB_STEP_MS` 改成 100 即报错）。
- 新全局（`g_lb_*`）都在 `#if CAN_LOOPBACK_SELFTEST` 内 → **Normal 镜像布局只受解析器自测的
  `static uint32 last_ms` 影响**；两镜像均 `make clean && make` 0 警告 0 错误。

## 本轮新增（分层补全 · AUTOSAR CP 四层核对，方案 B）
目标：按 `.clinerules` §4 四层框架"缺层补层、错层纠正"，**不碰引脚/外设初始化/协议/驱动接口签名**。

**改动**
- **新建 `Src/platform.c`（平台层实现，此前缺失）**：把原先内联在 `main.c`（应用层文件）里的
  平台服务与启动期寄存器操作**逐字**搬入 —— `Platform_EarlyInit()`（收拢 `main()` 里内联的
  SYSCFG 时钟 + 关 JTAG 释放 PB3/PB4 + VTOR 定位，属越层操作）、`SystemClock_Config`、
  `SysTick_Init`、`SysTick_Handler`、`g_ticks`、`millis`、`delay_ms`、`UART1_Init`、
  `uart_putc/puts/printf/put_uint/put_sint`、`Led_Init`（原 `static MX_LED_Init`）、
  `_sbrk`（newlib 堆桩）。
- `Inc/platform.h`：新增 `Platform_EarlyInit()` / `Led_Init()` 声明，注释由"定义在 main.c"
  改为"定义在 `Src/platform.c`"；GPIO 助手原样保留 → **上层调用点零改动**。
  （`LED_ON/LED_OFF` 寄存器宏本轮先原样保留，② 轮已替换为 `Led_Set()`，见下一节。）
- `Src/main.c`：平台层搬迁后 1211 → 1034 行；`main()` 开头改调 `Platform_EarlyInit();`，
  `MX_LED_Init()` → `Led_Init()`；`<stddef.h>`（只服务已迁走的 `_sbrk`）一并收回。
- **本轮按方案 B（零时序变化）**：`MX_TIM3_PWM_Init` / `MX_TIM1_PWM_Init` 仍留在主流程。
  并入 `Motor_Init()`/`Servo_Init()`（方案 A）会挪动 TIM3/TIM1 的启动时机，留待下轮单独验证；
  本轮不改，分层正确性不受影响。
- `tools/timing_audit.py`：新增 `SRC` 目录常量与 `all_src`（用 `glob` 合并 `Src` 下全部 `.c`），
  时钟 / USART BRR / TIM3 / TIM1 / SysTick 常量查找与 ISR `volatile` 扫描改走 `all_src`；
  `Task_*` 内禁 `delay_ms` 的项目仍只读 `main.c`。**理由**：原先 6 项硬编码读 `main.c`，
  一搬文件就 `[解析失败]`（退出码 2），现在搬文件不会再误报。

**验收（本轮实测）**
| 项 | 结果 |
|---|---|
| `make clean && make`（Normal） | **0 警告 0 错误**；text 7756 / data 8 / bss 60（① 轮；② 轮后 7768） |
| `make clean && make CAN_LOOPBACK=1` | **0 警告 0 错误**；text 9316 / data 16 / bss 100（① 轮；② 轮后 9340） |
| `python3 tools/timing_audit.py` | **PASS 17 / WARN 0 / FAIL 0** |
| 符号归属（`nm --defined-only build/Src/*.o`） | 14 个平台符号**只在 `platform.o`**；`main.o` 与各驱动 `.o` 均为 0 个 |
| `main.c` 残留寄存器访问 | 仅 `MX_TIM3/MX_TIM1` 两段（方案 B 预期），其余为 0 |
| `.data+.bss` | 仍为 0x20000000..0x20000073（`mdw 0x20000000 32` 覆盖不变），仅 `g_ticks` 移到 0x2000006C |

> 分层收益：应用段/调度段不再出现任何平台寄存器操作；将来迁 AUTOSAR CP 的 MCAL 只改
> `platform.c` 内部实现，`platform.h` 接口不变，上层代码零改动。

## 本轮新增（② LED 抽象化 + 死代码清理）
### LED：寄存器宏 → 平台层接口
- **改前**：`platform.h` 里 `LED_ON()` / `LED_OFF()` 宏直接写 `GPIOF->BSRR`，宏在应用层/调度层
  展开 ⇒ 严格按 §4 看，**业务代码里仍出现了寄存器写**（只是被宏包住），换板子还得改头文件。
- **改后**：`void Led_Set(boolean on)` 实现在 `Src/platform.c`；引脚映射 `LED_PORT/LED_PIN` 与
  两处 BSRR 写**全部下沉**到平台层，`platform.h` 只留 `Led_Init()` / `Led_Set()` 两个接口，
  不再暴露任何 LED 寄存器宏。7 处调用点（等级 2/3 常亮、等级 0 熄灭、等级 1 闪烁、`#if 0`
  留档 demo）全部改 `Led_Set(TRUE/FALSE)`，BSRR 写法逐位一致（低电平点亮，语义不变）。
- 顺带清理：`platform.h` 里注释掉的旧版 `gpio_mode`（单 bit 版，已被多 bit 版取代）删除。

### 死代码清理（`main.c`，删除的都是**不参与编译**的注释块）
1. `main()` 内被注释的实验块：`// 加这段测试` 的 LED 闪烁死循环、`// MotionDemo_Run()`、
   两段 `// while(1)` 的舵机/电机手动测试；
2. 文件尾整段被注释的**旧版 `main()`**：含"PA6 走普通 GPIO 恒高"实验
   （`GPIOA->BSRR = 1U<<6` 绕过 TIM3 直推 PWMA），与现行 TIM3 20kHz PWM 方案冲突，留着极易误改。
`#if 0` 的 `MotionDemo_*` **保留**（README 明确留档，改非阻塞后才能启用），其 LED 调用已同步
改成 `Led_Set()`，将来放开 `#if 0` 即可编译。

### 验收（② 轮实测）
| 项 | 结果 |
|---|---|
| `make clean && make`（Normal） | **0 警告 0 错误**；text **7768** / data 8 / bss 60 |
| `make clean && make CAN_LOOPBACK=1` | **0 警告 0 错误**；text **9340** / data 16 / bss 100 |
| `python3 tools/timing_audit.py` | **PASS 17 / WARN 0 / FAIL 0** |
| `LED_ON` / `LED_OFF` 残留（源码级） | **0**（仅注释里提到名字） |
| `LED_PORT/LED_PIN` / BSRR 写归属 | 仅 `Src/platform.c`；`platform.h` 已无寄存器宏 |
| `Led_Set` 符号（`nm`） | `platform.o` 定义；`main.o` 引用 1 次；各驱动 `.o` 0 次 |
| `Src/main.c` 行数 | **1211 →（①拆平台层）1034 →（②清死码）916** |
| 文本增量 | +12 B（Normal）/ +24 B（LB）：`Led_Set` 由宏展开改为函数调用 |

> LED 语义未变（等级 1 闪烁 / 2、3 常亮 / 0 熄灭）；闪烁在 `Task_100ms` 内每拍最多一次
> 函数调用，对时间片无影响。

## ③ 真机验收（**本轮已执行 ✅**：ST-Link 已 attach 进 WSL，全流程真机跑通）
```bash
# Windows 管理员 PowerShell（一次性）
usbipd list                                  # 找到 ST-Link 与 CH340 的 BUSID
usbipd bind   --busid <STLINK_BUSID>         # 首次需 bind
usbipd attach --wsl --busid <STLINK_BUSID>   # SWD 烧写/读回
usbipd attach --wsl --busid <CH340_BUSID>    # 想看 115200 串口日志时再 attach
```
```bash
# WSL 内（宿主 openocd 已在 PATH，不需要 sudo；容器内则无 sudo/bash，见下）
cd /home/fantasy/workspace2/edge_ai_car/04_stm32
make clean && make CAN_LOOPBACK=1     # ① 回环自测镜像
bash tools/hw_verify.sh               # 烧写 + SWD 读回 + 自动判定
# make flash CAN_LOOPBACK=1           # 等价的手动烧写
```
#### 非交互式容器烧写（当前 Linux 工作机**已实测通过**，2026-10-01）
`run_toolchain.sh` 是**交互式**入口；无人值守 / CI / Agent 场景改用一次性 `docker run`：
```bash
cd /home/fantasy/workspace2/edge_ai_car/04_stm32
docker run --rm --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/fantasy/workspace2/edge_ai_car:/workspace2 \
  --entrypoint /bin/bash autoedge-toolchain:v2 -c \
  'cd /workspace2/04_stm32 && make clean && make && \
   openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
     -c "program build/car.elf verify reset" -c shutdown'
# 实测：Programming Finished / Verified OK / Resetting Target
#       （device id 0x101f6413，flash 1024 KiB，Target voltage 3.09V）
```
> 三个关键点：
> ① **容器内工程路径是 `/workspace2/04_stm32`**（`tools/timing_audit.py` 已不再硬编码该路径：默认按脚本
>    自身位置推导工程根，任意 cwd / 任意克隆路径都能直接跑；仍可用 `PROJ_ROOT=/path/to/04_stm32` 覆盖）；
>    `make`/`arm-none-eabi-gcc`/`openocd`/`bash` 都在容器 `PATH`（`/bin/*`）；
> ② 必须 `--privileged` **且** `-v /dev/bus/usb:/dev/bus/usb`，否则 openocd 报 `LIBUSB_ERROR_ACCESS`
>    （宿主用户不在 docker 组时还要 `sudo`）；
> ③ 脚本较长的调试序列，用 `docker run -i ... --entrypoint /bin/bash <IMG> -s < script.sh` 从 stdin 喂，
>    再配 `setsid nohup ... > log 2>&1 &` 做后台采样（单条命令别超过 30s，否则被工具超时杀掉）。

真机实测结果（2026-09-27）：

| 项 | 结果 |
|---|---|
| `make clean && make`（Normal） | **0 警告 0 错误**；text **7980** / data 8 / bss 64 |
| `make clean && make CAN_LOOPBACK=1` | **0 警告 0 错误**；text **9552** / data 16 / bss 104 |
| `PROJ_ROOT=$PWD python3 tools/timing_audit.py` | **PASS 17 / WARN 0 / FAIL 0** |
| `bash tools/hw_verify.sh`（回环镜像真机） | **✅ 回环自测通过**：`g_can_test_echo_bad=0`、`g_lb_fail=0`、`g_lb_last_round_fail=0`、`rx=8/tx=8` 自洽、`g_lb_speed_echo=65535` + `echo_ok=1`（0x125 车速链路字节契约已验）；`CAN1->BTR=0xC04E0003`、`MSR=0x00000C08`、`TSR=0x1C000003`、`ESR=0` |
| 分层/裸机约束 | 新代码只落在驱动层（`Can_RecoverIfStuck`）+ 调度层调用；`main.c` 应用段不碰寄存器；两镜像均 0 警告 |

> 环境说明：容器镜像内**没有可用的 `sudo`/`bash`**（此为当初容器里 openocd `Error: open failed`
> 的真实原因），故真机验证改在**宿主 WSL** 跑（`openocd`/`arm-none-eabi-*` 已在 PATH）；
> `tools/hw_verify.sh` 本轮已改造：地址由 `nm -n build/car.elf` **现读现用**（不再硬编码旧地址，
> 代码一改就自动跟上），`--serial` 开串口日志改用 `stdio` 探测 `/dev/ttyUSB*`。
>
> ⚠ **2026-10-01 补正**：上述"无 bash/sudo"只对早期镜像成立。当前 `autoedge-toolchain:v2` 内
> `/bin/bash`、`/bin/make`、`/usr/local/bin/openocd` 齐备，配 `--privileged -v /dev/bus/usb`
> 后 openocd 可直连 ST-Link（`SWD DPIDR 0x2ba01477`）并成功烧写 → 见上文
> **「非交互式容器烧写」**。宿主机侧反而因 `/dev/bus/usb/*` 属 `root:root 0600` 而报 LIBUSB_ERROR_ACCESS。

### Bus-Off 自愈（本轮真机发现并补全，规则 §6.4）
**改前现象**：只烧 Normal 镜像、**板端未接/无 120Ω 终端**时，CAN1 停在
`MCR=0x00010044(INRQ=0)`、`MSR=0x00000009(INAK=1)`、`TSR=0x82000000`（3 个 TX 邮箱**永久占满** + `Terr`）、
`ESR=0`，且 1.2s/1.5s 周期连续 21 次采样**完全不变** ⇒ `Can_Send()` 每次都走满 100000 次有界等待
后返回 FALSE（**不死锁**，PC 一直在 `main` 调度循环里）⇒ 0x125 永远发不出去、0x123 也收不到，
表现为"**车在总线上假死**"。

**根因（三条互相印证的真机证据）**：bxCAN **离开初始化模式需要总线上出现连续隐性位**。
1. 让板子**脱离调试器**自由跑 5s 后首次 attach：`MSR` 仍是 `0x09`、自愈计数 = 0 →
   与调试器/`DBF` 冻结无关；
2. 只清 `MCR.DBF`(bit16) 后 1.5s 仍是 `0x09` → **排除 DBF**；
3. 用调试器把 `BTR` 临时改成 `LBKM|SILM`（环回，不依赖外部总线）后 `INAK` 立刻清 0、
   `TSR` 变 `0x1C030303`（三邮箱全空 + 三个 TXOK）→ **外设与驱动本身完好，帧能发能收**。

⇒ 无对端 + 缺 120Ω 端接 / 未共地时差动线悬空，收回的 `RXD` 不构成连续隐性位，
外设无法同步，bus-off 后连 `ABOM` 也退不出来。**这不是固件 bug，而是规则 §1 的 CAN 硬件前提未具备。**

**修复（兜底，1s 内拉回总线，不用重新上电）**：驱动层新增 `Can_RecoverIfStuck()`
（`Inc/can_driver.h` 声明）：检测到"硬件自己在 init 态"（`MSR.INAK=1` 且 `MCR.INRQ=0`）时，
复用 `can_init_internal()` 做一次**有界** INRQ 往返（含 BTR 重写 + 过滤器重装，全部幂等，
所有等待都有界 → 任务内不阻塞）；调度层 `Task_1000ms` 每秒调用一次。
观测：串口 1s 诊断行新增 `Rec=`，恢复成功会打 `[CAN] back on bus after init-mode stall, total=N`
（限流：第 1 次 + 每 10 次）；SWD 可读 `g_recover_cnt`。
**判据**：`g_recover_cnt=0` 且 `MSR.INAK=1` ⇒ 总线仍不合格（先查 CANH/CANL/GND 三线、两端 120Ω、
TJA1050 VCC=5V）；总线合格后 `INAK` 应自动清 0，最迟 1s 内被自愈拉回。

### 步骤②（Normal + CANable 抓包）前置条件与观测
> ✅ 具体操作命令、slcan 适配器/`sudo` 通道说明与**真机实测结果**见下方 **§步骤③ 真机联调**（本轮已跑通）。
1. **硬件前提**：CANable（CANH/CANL/GND 三线 + 显式共地）、**总线两端各 120Ω**、
   TJA1050 VCC=5V；用 CAN1 时**禁插**板载 USB SLAVE 口（PA11/12 与 OTG_FS 冲突）。
2. **上线判据（不看串口也能判）**：SWD 读 `CAN1->MSR` @0x40006404，`INAK=bit0` 应为 **0**；
   `TSR` @0x40006408 的 `TME0/1/2` 应在 0x123/0x125 收发时周期性出现 1；
   `ESR` @0x40006418 的 `TEC/REC` 应保持在低值（不再出现 `0x09` 卡死）。
3. **收发内容判据**：CANable 侧应能看到 **0x125 每 20ms 一帧**（byte0-1 = `uint16` 小端 km/h，
   0xFFFF = 无效）；向车上发 0x123（byte0 = 0~3）后 SWD 读 `g_brake_level`/`g_brake_latched`
   即知是否到位（无需串口）；发 0x124 可看 `g_obstacle_cm`/`g_dist_rx_count`。
4. **抓包/发帧工具**（**本轮已就绪**）：宿主 WSL 已装 `can-utils`（`/usr/bin/candump`、`cansend`、`slcand`）。
   本机 CANable2（`lsusb` = `16d0:117e MCS CANable2`）枚举成 **CDC-ACM**（`/dev/ttyACM0`，**不是** `gs_usb`
   网卡），所以走 **`slcand` → SocketCAN**，而不是"直插成 `can0`"。命令与实测见下方 **§步骤③ 真机联调**。

`tools/hw_verify.sh` 做的事：① 用 `nm -n` **现读现用**打印本版 elf 的 `g_*` 地址（改代码就会
平移，禁止照抄文档）；② `openocd program ... verify reset`；③ 第二段会话 `sleep 1500 → halt →
mdw 0x20000000 32`（开机态）→ `resume → sleep 6000 → halt → mdw`（跑完一轮）→ `reset run`；
④ 解析最后一次 dump，按符号地址逐项判定并给 ✅/❌ 结论（`g_can_test_echo_bad` / `g_lb_fail` /
`g_lb_last_round_fail` 必须为 0；`rx` 与 `tx` 自洽 = 相等或恰有一帧在途 —— `tx` 是 `uint8`，
1 帧/s 下 256s 回绕一次，判定按 mod 256 处理；`g_lb_last_round_fail=0xFF` 只给 ⚠ 不给 ❌）。
**判定逻辑已离线自测**：用合成的 openocd dump 跑"全通过 / 未跑完一轮 / 失败"三用例（分别输出
✅ / ⚠ / ❌），且位宽严格按源码声明（`uint8/boolean=1B`、`uint16=2B`、`uint32=4B` —— 读宽了会
串到相邻字段：`g_lb_last_round_fail` 挨着 `g_obstacle_cm`、`g_lb_step` 挨着 `g_speed_valid`）。

**通过标准**（任一途径满足即可）：
- 串口：`[AEB-TEST] parser PASS (8/8)` → `[CAN-LB] init OK` → `[CAN-LB] PASS` 且
  `AEBtest=PASS (10/10)`、`[ENC] self-test = 1`、`vL/vR` 手转轮子时变化；
- 无串口（SWD）：判定项全 0，且 `g_can_test_rx_count == g_can_test_tx_counter`、
  `g_ticks` 每秒 +1000、`g_lb_round` 每 5s +1。

**失败排查**（规则 §6.3 顺序，别跳步）：`init FAIL` → 看 `Can_GetTxErrorCount/RxErrorCount/BusOff`
→ 检查 BTR（回环须 `0xC04E0003`）→ 再查硬件（RXD→PA11、共地、两端 120Ω、TJA1050 VCC=5V）。
**真机跑自测镜像会真实驱动电机**（等级 3 短路急刹、等级 2 减速），请**架空车轮**。

## 步骤③ 真机联调（Normal + CANable，**本轮已执行 ✅** 2026-09-27）

联调拓扑：STM32 `PA11/PA12` ↔ TJA1050（**VCC=5V**、显式共地）↔ CANable2（`CANH/CANL/GND` 三线、
**总线两端各 120Ω**）；调试串口 = 板载 CH340 ↔ USART1（115200 8N1）。
⚠ 用 CAN1 时**禁插**板载 USB SLAVE 口（PA11/12 与 OTG_FS 冲突）。

### ① 适配器：CANable2 在本机走 **slcan**（不是 gs_usb）
`lsusb` = `16d0:117e MCS CANable2`，内核用 `cdc_acm` 枚举成 **`/dev/ttyACM0`**（major 166），
**没有** `gs_usb` 网卡 ⇒ 必须用 `slcand` 把它变成 SocketCAN 接口：
```bash
ls -l /dev/ttyACM*
报 No such file → 先跑 sudo modprobe cdc_acm，然后拔了重插（WSL 的话在 Windows 的 PowerShell 里 usbipd detach --busid X 再 usbipd attach --wsl ...），再看一次

sudo apt install can-utils                       # candump / cansend / slcand（宿主已装）
sudo slcand -o -c -s6 /dev/ttyACM0 can0          # -o=发 O\r 进正常模式  -c=退出发 C\r  -s6=500kbps
sudo ip link set can0 up
ip -d link show can0                             # 期望 UP,LOWER_UP + state ERROR-ACTIVE + type 280
```
> ⚠ **`slcand` 的第二个参数才是接口名**：`slcand [options] <tty> [canif-name]`。省掉它就按内核默认规则
> 命名为 **`can0`** —— 此时 `ip link show slcan0` 报"找不到设备"**属正常现象、不是故障**；想叫 `slcan0`
> 就显式写第二个参数。`slcand` 默认后台常驻（本版前台开关是 `-F`，`-f` 不是前台）。
> ⚠ `slcand` 一停/重启，接口即消失，必须重新 `ip link set can0 up`。

### ② WSL 里免密拿 root（`sudo` 要密码时）
```bash
timeout 25 wsl.exe -u root -e bash -c 'candump -ta -n 20 can0'   # WSL interop 免密 root
```
> 本机 `/usr/bin/wsl.exe` 存在（`/mnt/c/Windows/system32/wsl.exe`），但 `sudo` 需密码；而 `/dev/ttyACM0`
> 属 `root:root 0600`（普通用户打不开）。故 `slcand` / `ip` / `candump` / `cansend` 一律走上面这条通道。

### ③ 抓包 / 发帧（主机侧命令）
```bash
candump -ta -n 20 can0                  # 带时间戳抓 20 帧（-n 收满即退）
candump -ta can0 | stdbuf -oL head -20  # 等价写法：强制行缓冲
cansend can0 123#0100000000000000       # byte0=1 预警（LED 闪烁 + 日志，不制动）
cansend can0 123#0200000000000000       # byte0=2 部分制动（斜坡降速 + 锁存）
cansend can0 123#0300000000000000       # byte0=3 紧急制动（立即短路急刹 + 锁存）
cansend can0 123#0000000000000000       # byte0=0 解除（**唯一解锁路径**）
cansend can0 124#6400000000000000       # byte0-1 小端 = 0x0064 = 100cm
```
> ⚠ **别用 `timeout 5 candump can0` 判定"收不到帧"**：进程被信号杀死时 stdio 缓冲没刷盘 → **假阴性**
> （本机曾因此误判一次）。用 `-n N` 收满干净退出，或 `stdbuf -oL`。

### ④ 串口日志（115200 8N1）
本机设备对应关系（`usbipd list` 现读现用）：**CH340 = `1a86:7523`、BUSID `1-3`、Windows 侧 `COM5`**；
CANable2 = `16d0:117e`、BUSID `1-1`（已 attach，即 WSL 里的 `/dev/ttyACM0`）。
> ⚠ **COM5 同一时刻只能被一个程序打开**：XCOM 占着时别的程序打开会报"对端口 COM5 的访问被拒绝"
> （本轮实测），反之亦然 —— 要把串口交给 WSL/脚本抓，得**先关 XCOM**。
1. **Windows 侧 XCOM**（最省事，推荐）：端口 **COM5**、115200 8N1 → 直接看
   `[AEB] level=3 EMERGENCY (latched, only level0 releases)`、`[AEB] level=0 RELEASE` 等行
   （等级 1/2/3/0 各一行，等级 2/3 带 `latched`；另每 1s 一行诊断，含 `Rec=` 自愈计数）。
2. **Windows 侧 .NET 采集**（不提权、不碰 usbipd，**先关 XCOM**）：
   ```bash
   timeout 40 powershell.exe -NoProfile -Command '
     $p=New-Object System.IO.Ports.SerialPort("COM5",115200,"None",8,"One"); $p.ReadTimeout=300;
     $p.Open(); $p.DiscardInBuffer();
     $t=[DateTime]::Now.AddSeconds(13);
     while([DateTime]::Now -lt $t){ try{ $s=$p.ReadLine(); if($s){$s} }catch{} }
     $p.Close()' > /tmp/aeb_serial.txt 2>&1
   ```
   一边跑上面这条，一边发帧（见 ③），完事看 `/tmp/aeb_serial.txt`。
   （本轮因 XCOM 独占 COM5 未取到数据，打开/读循环路径已跑通。）
3. **WSL 侧**（把 CH340 整块 USB 转进 WSL；`bind` 需管理员，attach 后 Windows 侧 XCOM 会掉线）：
   ```powershell
   # Windows 管理员 PowerShell
   usbipd bind   --busid 1-3            # CH340，首次需 bind
   usbipd attach --wsl --busid 1-3
   ```
   ```bash
   timeout 15 wsl.exe -u root -e bash -c \
     'stty -F /dev/ttyUSB0 115200 raw -echo; timeout 10 cat /dev/ttyUSB0'
   ```
> ⚠ 未 attach 时 WSL 里**没有** `/dev/ttyUSB*`（`/dev/ttyACM0` 是 **CANable2 的 slcan 口，别接错**）。
> ✅ **没有串口也能完成联调验收**：SWD 读应用层 `g_*` 是等价判据（见 ⑥），本轮即如此完成。

### ⑤ 本轮实测结果（2026-09-27，Normal 镜像 + CANable2 走 slcan）
| # | 主机动作 | 观测（原始值） | 结论 |
|---|---|---|---|
| 0 | `ip link set can0 up` | 主机 `can0 <NOARP,UP,LOWER_UP>`、`state ERROR-ACTIVE`；STM32 `MSR=0x00000C08`(**INAK=0**)、`TSR=0x1C000003`、`ESR=0`、`BTR=0x004E0003`(500k/Normal) | 节点上线 ✅ |
| 1 | `candump -ta -n 20 can0` | 帧间隔 **20.0ms**、5s 内 **50.2 帧/s**；`125 [8] 00 00 00 00 00 00 00 00` | **0x125 @500kbps** ✅ |
| 2 | `cansend …123#01…` | `g_brake_latched/level` = `0x00000100` → latched=0 / level=1；`g_brake_rx_ms` 0→81089ms；`g_brake_rx_seen=1` | 等级 1 预警不制动 ✅ |
| 3 | `cansend …123#02…` | `0x00000201` → latched=1 / level=2 | 等级 2 部分制动 + 锁存 ✅ |
| 4 | `cansend …123#03…` | `0x00000301` → latched=1 / level=3；**`GPIOB->ODR=0x3B`（PB0/1/4/5=1，四路 IN 全高）+ `TIM3->CCR1=CCR2=0`** | 等级 3 **真短路急刹（落到硬件）** ✅ |
| 5 | 静默 1.5s 不发 0x123 | `0x00000301` 与 `g_brake_rx_ms` **完全冻结不变** | **刹车优先、不自动解除（失效保护）** ✅ |
| 6 | `cansend …123#00…` | `0x00000000`；**`GPIOB->ODR=0x08`（PB0/1/4/5 清 0；PB3=STBY 恒 1）** | 唯一解锁路径 + 真解除 ✅ |
| 7 | `cansend …124#6400…` | `g_obstacle_cm=0x0064=100`、`g_dist_rx_count 0→1` | 0x124 小端距离链路 ✅ |
| 8 | 收官 | 0x125 仍 **48.6 帧/s**；`bus-off`/`error-warn`/`error-pass`/`arbit-lost`/`bus-errors` **全 0**；`RX 6699 pkt / 0 err / 0 drop`、`TX 7 pkt / 0 err`（= 发出的 7 帧，**零重传 ⇒ 每帧都被 STM32 ACK**）；`g_rx_overrun=0`、`g_recover_cnt=0` | 全程零错误、未触发自愈 ✅ |

> `0x125` 载荷 `00 00` = **有效速度 0 km/h**（车静止、`g_speed_valid=1`），**不是**无效占位 `0xFFFF`；
> 手转轮子时 `byte0-1` 应随 `g_speed_kmh` 变化（可边 `candump` 边转轮子复核小端换算）。
> 主机 `TX 7 pkt / 0 err` 与发帧次数逐一对应 ⇒ 这就是"发帧 → 收帧 → 解析 → 执行 → 硬件动作"全链路凭证。
> 上表 #2~#7 的 `g_*` 值用 SWD 读出（`mdw`，方法见 ⑥），**未改任何代码**；下面的串口原文与它逐项互证。

#### 串口原文（COM5 @115200 实测片段，与上表逐项互证）
采集方法 = ④-2（**先关 XCOM**，Windows 侧 .NET 打开 COM5），同期在 WSL 发帧：
```text
[AEB] lvl=0 latched=0 kmh=0 vL=0 vR=0 dist=100 dist_rx=2      ← 发帧前基线（每秒诊断行）
[AEB] level=3 EMERGENCY (latched, only level0 releases)        ← cansend 123#03
[AEB] 0x123 LOST -> keep brake latched (no auto release)       ← 500ms 无 0x123：保持制动、不自动解除
[AEB] lvl=3 latched=1 kmh=0 vL=0 vR=0 dist=100 dist_rx=2
[AEB] level=0 RELEASE                                          ← cansend 123#00
[AEB] lvl=0 latched=0 kmh=0 vL=0 vR=0 dist=100 dist_rx=2
[AEB] lvl=0 latched=0 kmh=0 vL=0 vR=0 dist=100 dist_rx=3       ← cansend 124#6400 后 0x124 计数 +1
[CAN]   TxErr=0  RxErr=0  BusOff=0  Ovf=0  Rec=0               ← 每秒一行，全程 0
```
第二轮（等级 1 → 2 → 解除 + 距离变化；注：`dist=` 只在**值变化**时打印）：
```text
[AEB] level=1 WARN (no brake)                                  ← cansend 123#01：不制动、不锁存
[AEB] lvl=1 latched=0 kmh=0 vL=0 vR=0 dist=100 dist_rx=3
[AEB] 0x123 LOST (warn only, no brake)                         ← 等级1 超时：无锁存可保持，只告警
[AEB] level=2 PARTIAL (decelerate->coast, latched)             ← cansend 123#02
[AEB] lvl=2 latched=1 kmh=0 vL=0 vR=0 dist=100 dist_rx=3
[AEB] 0x123 LOST -> keep brake latched (no auto release)       ← 等级2 超时：保持不解除
[AEB] dist=200cm                                               ← cansend 124#c800（0xC8 = 200cm）
[AEB] lvl=2 latched=1 kmh=0 vL=0 vR=0 dist=200 dist_rx=4
[AEB] level=0 RELEASE                                          ← cansend 123#00
[AEB] lvl=0 latched=0 kmh=0 vL=0 vR=0 dist=200 dist_rx=4
```
> ✅ **串口 ↔ SWD 完全一致**：`lvl=3 latched=1` → `lvl=0 latched=0`、`dist 100→200`、
> `dist_rx 2→3→4` 与实际发出的 0x124 帧数（累计 4 帧）逐一对齐；硬件侧另有 `GPIOB->ODR`
> `0x3B → 0x08` 佐证；`Rec=0` = 全程未触发自愈。`kmh=0` 是**有效 0**（车静止），无效时会打 `inv`。
> ⚠ 复现时若 `dist_rx` 对不上，先核 `cansend` 侧 `TX pkt` 计数（见 ③ 与上表 #8），别先怀疑固件。
> ⚠ 串口采集与 CAN 采集**互不干扰**：CH340（COM5）与 CANable2（`/dev/ttyACM0`）是两个独立 USB 设备。

### ⑥ 无串口时的 SWD 等价判据（本轮实际用法）
```bash
cd 04_stm32
arm-none-eabi-nm -n build/car.elf | grep -E 'g_brake|g_obstacle|g_dist|g_speed'  # 地址现读现用（改代码会平移）
# 解码速查（本版实测地址，仅作对照，必须用 nm 复核）：
#   0x20000018  低字节 = g_brake_latched(1B)  次字节 = g_brake_level(1B)  次字(0x1c) = g_brake_rx_ms(4B)
#   0x20000020  低字节 = g_brake_rx_seen(1B)                           次字(0x24) = g_dist_rx_count(4B)
#   0x20000000  低16位 = g_obstacle_cm(2B)   高16位 = g_speed_kmh(2B)
#   0x40020414  GPIOB->ODR（急刹 PB0/1/4/5=1；解除后清 0，PB3=STBY 恒 1）
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'gdb_port 0' \
  -c 'init' -c 'halt' \
  -c 'mdw 0x20000018 2' -c 'mdw 0x20000020 2' -c 'mdw 0x20000000 1' -c 'mdw 0x40020414 1' \
  -c 'resume' -c 'shutdown'
```
> ⚠ **联调期读回必须用 `resume`，不要用 `reset run`**：`reset run` 会重启 MCU，把 `g_brake_latched` /
> `g_brake_rx_ms` 等运行态清零，下一步判据随之失真（回环自测收尾用 `reset run` 是对的，语义不同，别混用）。
> ⚠ 位宽严格按源码声明：`latched`/`level`/`rx_seen`/`speed_valid` = **1B**、`obstacle_cm`/`speed_kmh` = **2B**、
> `rx_ms`/`dist_rx_count` = **4B**；读宽了会串到相邻字段。
> 判据优先看**应用层 `g_*`**（呼应规则 §4"应用层不碰寄存器"），寄存器值只作交叉印证。

### ⑦ "收不到帧"排查顺序（本环境血泪版，按序执行、别跳步）
1. `ls /sys/class/net/` 先确认真实接口名：`ip link show slcan0` 找不到 ⇒ **大概率是 `slcand` 没给第二个
   参数**（接口叫 `can0`），**不是故障**；
2. `ip -d link show can0`：`type 280` + slcan 速率表 = slcan 路线 ✓；`state UP` 才说明 `ip link set can0 up` 生效；
3. `candump -n 20 can0`（**别用 `timeout` 杀**）→ 仍无帧则转 STM32 侧读 `CAN1->MSR`(INAK)/`TSR`/`ESR`；
4. 硬件三件套：`CANH/CANL/GND` 三线 + **显式共地**、**两端各 120Ω**、TJA1050 **VCC=5V**、`RXD→PA11`；
5. `MSR.INAK=1` 且 `g_recover_cnt=0` ⇒ 总线不合格（详见上节 §Bus-Off 自愈）。

## RDK 行车链路（0x100 / 0x101 / 0x102）—— **真机打通 ✅**（2026-09-28；**协议 2026-10-02 修订**）

> ⚠ **本节已按 2026-10-02 修订版 heartbeat_v1 重写**：判活**只看 0x100**（0x102 不参与任何判活/恢复/超时）、
> 取消 STOPPING 态与“0x100+0x102 任一在收”判活、HOLD 唯一出口 = “收到有效 0x100”（byte3 必须变化）。
> 带【历史】标记的小节保留 2026-09-28 / 10-01 的实测证据，**语义一律以本节为准**。
> 已删除的符号（**别再用它们写断言**）：`g_rdk_hs_fail`、`g_rdk_stop_reason`、`g_rdk_led_ovr`、
> `g_rdk_coast_cm`、`RDK_ST_STOPPING`、`Rdk_HoldReconnectOn`、`Rdk_HsLedService`。

与 §AEB 协议**并存**：ID 不重叠、语义互不覆盖；两条链路的制动请求在应用层取"或"
（刹车优先 —— RDK 巡航期间若 AEB 已锁存制动，巡航输出被强制 0，打 `[RDK] AEB latched -> cruise output 0`）。
契约的**唯一权威**是 `Inc/can_protocol.h` 的「RDK 行车链路协议」块；调参面板是 `Inc/rdk_config.h`（改参数只动 `Src/rdk_config.c`）。

### 协议（标准帧 / 500kbps / 数据段固定 8 字节 / 小端 / 未用字节填 0）
| ID | 方向 | 内容 |
|---|---|---|
| 0x100 | RDK→STM32 | `byte0` 命令：**00** 停车（PWM=0，斜坡收敛滑行）/ **01** 巡航（软启动到 `cruise_pwm`=300）/ **02** 预警（`warn_pwm`=150 + 蜂鸣）/ **03** 制动（PWM=0 + 短路制动）；`byte1` 距离 cm；`byte2` 置信度；`byte3` 计数（**与上帧相同 = 重复帧：整帧丢弃、不执行、不刷新判活**）；`byte4-7`=0。DLC **必须 = 8** |
| 0x101 | STM32→RDK | 握手请求 + STM 心跳 `AA seq 00 00 00 00 00 00`；**STATIC / CRUISE / HOLD 全程每 200ms 恒发**（`hs_period_ms`；无次数上限、无阶段停发）；**只有 LOCK 停发** |
| 0x102 | RDK→STM32 | `55 01 seq 00×5` = 握手应答（`byte1`=0x01、`byte2`=seq 递增）/ `55 02 seq 00×5` = 心跳（`byte1`=0x02）；DLC **必须 = 8**（如 `102#5501` 短帧整帧被拒）。**只在 STATIC 阶段参与握手**：连续 2 帧有效应答（byte2 递增）→ CRUISE；CRUISE/HOLD 里只记时刻作日志，**不参与判活/恢复/超时** |

> ⚠ `0x102` 的 DLC 已**收紧为 8**：早期“接受 DLC≥2”的例外**已取消**（`can_driver.c` / `can_protocol.h` 一律 `len != CAN_DATA_LEN` 整帧拒绝）。

### 状态机（`Src/main.c` 应用层；`g_rdk_state` 只有一个写者 `Rdk_SetState()`）
```
STATIC(0) --上电/复位--> [PWM=0 + 短路制动, 忽略 0x100, 0x101@200ms 恒发] --2×有效 0x102 应答--> CRUISE(1)
   |                                                                                              |
   | 60s 未握手                                                              有效 0x100 静默 ≥200ms
   v                                                                                              v
 LOCK(3) <------------------- 60s 未恢复 ------------------- HOLD(2) <-- 收到有效 0x100(byte3 变化) --+
   ^                                                                                              |
   +-- PWM=0 + LED 常亮 + 停发 0x101（需复位）          HOLD 里仍每 200ms 发 0x101（0x102 不能恢复）+

CRUISE 里四条命令都不改状态：00 停车 / 01 巡航 / 02 预警 / 03 刹停 —— **没有 STOPPING 态，也没有命令型锁死**。
```

| 状态 | 行为 | 出口 |
|---|---|---|
| **0 STATIC** | 上电/复位初态：PWM=0 + 短路制动（**未握手不许动**）；**忽略 0x100**（只记账、不执行、不判活）；每 200ms 发 0x101；**LED1 快闪 5Hz** | 累计 2 帧有效应答 → 1 CRUISE；`lock_timeout_ms`(60s) 未握手 → 3 LOCK |
| **1 CRUISE** | **唯一可驱动状态**。按有效 0x100 执行：00→PWM=0（斜坡收敛滑行）/ 01→`cruise_pwm`=300 软启动 / 02→`warn_pwm`=150 + 蜂鸣 / 03→PWM=0 + 短路制动；每 200ms 发 0x101；**LED1 慢闪 1Hz** | 判活**只看 0x100**：静默 ≥ `hb_timeout_ms`(300) → 2 HOLD |
| **2 HOLD** | PWM=0 + 短路制动保持；仍每 200ms 发 0x101；LED0/LED1 **常亮**；**忽略 0x102**（不计数、不恢复） | **收到有效 0x100**（byte3 变化）→ 回 1 CRUISE 并按该帧执行；`lock_timeout_ms`(60s) 未恢复 → 3 LOCK |
| **3 LOCK** | PWM=0 + **停发 0x101** + LED0/LED1 常亮；串口 `[RDK] LOCK ... reset required` | 只能复位 |

**判活铁律（0x100 唯一）**：CRUISE 里 `g_rdk_hb_ms` = 最近一帧**有效** 0x100 的时刻，静默 ≥ `hb_timeout_ms`(300)
即本地刹停进 HOLD（`[RDK] 0x100 TIMEOUT -> local stop (HOLD)`）——**0x102 在不在都一样**；重复帧（byte3 未变）
**不刷新**该时间戳。判定在 20ms 服务里做（分辨率高于“每 200ms 查一次”），全程非阻塞、完全不依赖上位机。

**命令语义 vs 状态（安全语义分离）**：`cmd=03` 只是**命令**，不改状态码（仍是 CRUISE，输出保持 0 + 短路制动）——
板端要“锁死”必须**停发 0x100**（→ HOLD → 60s → LOCK）或复位。这样“刹停”由命令表达、“链路失联”才进 HOLD/LOCK，
两者语义不混。

### 串口特征（115200 8N1，与 SWD 读回互证）
```
[RDK] boot: PWM=0 static lock, handshake (200ms x0x101, need 2x0x102-ack, lock after 60s) pwm=300
[RDK] link: live only if 0x100 within 300ms -> HOLD; 0x102 = handshake only
[RDK] STATE 0 -> 1 / [RDK] HS OK (ack=2) -> CRUISE pwm=300
[RDK] HB cmd=1 cnt=177 dist=20cm conf=90            ← 命令/计数变化才打（0x100 本身是周期帧）
──── 停发 0x100（0x102 照发也没用）────
[RDK] 0x100 TIMEOUT -> local stop (HOLD) / [RDK] HOLD (pwm=0, wait valid 0x100) / [RDK] STATE 1 -> 2
──── HOLD 收到新 byte3 的有效 0x100 ────
[RDK] STATE 2 -> 1（按该帧 byte0 执行；继续无 0x100 则再回 HOLD）
──── 60s 未握手 / 60s 未恢复 ────
[RDK] LOCK (pwm=0, LED on, 0x101 stopped) - reset required / [RDK] STATE x -> 3
```
（原文以 `Src/main.c` 的 `uart_puts` 为准；`uart_printf` 不做格式化，数字都是固件自写 itoa。）

### LED 指示（LED0=PF9 命令 / LED1=PF10 链路；均低电平点亮；`Rdk_LedService()` 每 20ms 相位生成，非阻塞）
| 状态 | LED0（PF9） | LED1（PF10） | 制造方法 | 串口互证 |
|---|---|---|---|---|
| STATIC | 灭 | **快闪 5Hz**（半周期 100ms） | 复位后不发 0x102 | `[RDK] STATE 0` + 总线上 0x101 每 200ms |
| CRUISE（00/01） | 灭 | **慢闪 1Hz**（半周期 500ms） | 收满 2 帧有效 `102#5501xx0000000000`（DLC=8） | `[RDK] HS OK (ack=2)` / `[RDK] STATE 0 -> 1` |
| CRUISE（02 预警） | **慢闪 1Hz** | 慢闪 1Hz | 发 `100#02…` | `[RDK] HB cmd=2 …`（状态码仍是 1） |
| CRUISE（03 制动） | **快闪 5Hz** | 慢闪 1Hz | 发 `100#03…` | `[RDK] HB cmd=3 …`（状态码仍是 1） |
| HOLD | **常亮** | **常亮** | 停发 0x100 ≥300ms（0x102 继续发也行） | `[RDK] 0x100 TIMEOUT …` / `[RDK] HOLD …` |
| LOCK | **常亮** | **常亮** | 复位后 60s 不握手（或 HOLD 60s 不恢复） | `[RDK] LOCK … reset required` |

> ⚠ LED0/LED1 都只由 `Rdk_LedService()` 在 20ms 服务里写（`millis()` 相位、无共享计数器、无 `delay_ms`）。
> ⚠ **LED 不是判据**：所有断言都走 SWD 读回 + 总线抓包，LED 只作现场肉眼互证。

> ⚠ **【历史】以下为 2026-09-28 旧协议（HANDSHAKE / STOPPING / HOLD=4 / `reason=` / `coast=`）的实测记录**，保留作证据链；
> 状态码与语义已按 2026-10-02 修订（0=STATIC / 1=CRUISE / 2=HOLD / 3=LOCK，判活只看 0x100）。现行脚本是
> `tools/regress/04_rdk_link_normal.sh`（T1~T8，`--lock` 验 60s→LOCK）与 `tools/hw_verify_rdk_recovery.sh`（无对端 SWD 注入 + 静默环回）。

### 本轮实测（`tools/regress/04_rdk_link_normal.sh`，SWD 读回 + 总线抓包，**50 通过 + 1 提示**，共 51 项判定）
| 步骤 | 刺激 | 实测 |
|---|---|---|
| 初态 | 复位后 | `st=1`、`pwm=0`、`g_state=3(BRAKE)`、`req=1`；总线抓到 `101 [8] AA 01 00×6` |
| 巡航 | 2×0x102（**第 1 帧故意 DLC=2**） | `st=2`、`pwm=300`、`g_current_speed=300`、`g_state=1(FORWARD)`、`GPIOB_ODR=0x19`(STBY+AIN1+BIN1)、`TIM3_ARR=49`、`CCR1=CCR2=15`（30% 占空**真写进定时器**） |
| 编码器（T2b） | 同上（**负载已上电**） | `g_rdk_v_cmps=42 cm/s`、左 **+45** / 右 **−40** cm/s、`TIM4_CNT=890` / `TIM8_CNT=64724` → **电机真转、编码器真反馈** |
| 维持 | 每 100ms 一帧 cmd=01，持续 1s+ | `st=2`/`pwm=300` 不变；`hb_rx +12`、`dup=0`、`bad=0`；`g_rdk_hb` 低 3 字节 = `01 14 5A`（cmd/dist/conf 字节序正确） |
| 预警 | cmd=02 周期帧 | `pwm=150`、`buzz=1`、`st=2`（预警不退出巡航态） |
| 制动 | 单帧 cmd=03 | `pwm=0`、`st=4`、`g_state=3(BRAKE)`、`reason=1(cmd03)`、`elapsed=150ms`、**滑行 `coast≈0.96~1.18cm`、残速回落 0 cm/s**（多次复跑区间） |
| 复位锁 | HOLD 下再发 cmd=01 | `st=4`、`pwm=0`、非 FORWARD（**锁死生效**） |
| 失效保护 | 复位→巡航→**0x100 与 0x102 都停发** | `reason=2(hb_timeout)`、`pwm=0`、`st=4`；距末帧 652ms 才判超时、判超时到停稳 200ms |
| **掉线自愈（本轮新增 T8）** | 承接上一步的掉线型 HOLD → 恢复 2×0x102 | `st=4` 不锁死（`hs_fail=0`）→ 每 2s `req` 递增 → 收满 2×0x102 → `st=2`、`pwm=300`（自动回巡航） |
| 全程 | — | `ESR.TEC/REC/BOFF=0`、`g_rx_overrun=0`（整场刺激不丢帧不报错） |

> 上表 51 项（T1~T7）为 2026-09-28 真机实测全通过；**T8 掉线自愈**于本轮新增（10 项判定），
> 合并后脚本共 61 项。T8 需要 `cansend` 控制刺激（板端/CANable 在线），本轮工作机**无 CANable**，
> 故 T8 尚未执行 —— 本轮改用 SWD 观测 + 故障注入在同一条真实链路上取证，见下一节。

⚠ **教训更正**：更早一轮"车轮未转"的结论（`TIM4_CNT=0`、速度恒 0）是**负载未上电**造成的，
不是固件问题 —— 电机 `VM`/编码器上电后 T2b/T5 立刻有读数。当时"软件链路已自证"的判断依然成立：
`sint16` 读回 PWM 真写进 `TIM3_CCR1/CCR2` + `GPIOB_ODR` 方向脚。

## 本轮修订（判活阈值 200→300ms + 单一配置点）—— 2026-10-10

把 0x100 判活阈值从 **200ms 放宽到 300ms**（实测 0x100 有效间隔约 110ms，200ms 余量过小、
存在抖动误刹风险），并把该数值收敛为**单一配置点**：全仓只有 `Inc/can_protocol.h` 的
`RDK_HB_TIMEOUT_MS_DEF` 一个真源，改它一处即全链生效。

| 文件 | 改动 |
|---|---|
| `Inc/can_protocol.h` | `RDK_HB_TIMEOUT_MS_DEF` 200→**300**；**判活护栏（范围 `[200,500]`）与本值同放一处**（越界直接编译失败，需人工确认） |
| `Src/rdk_config.c` | 删除原“精确值 `== 200u`”护栏（升级为 `can_protocol.h` 的范围护栏）；`g_rdk_cfg.hb_timeout_ms` 仍引用同一宏，**自动跟随** |
| `Inc/rdk_config.h`、`Src/rdk_config.c`、`Src/main.c` | 判活相关注释同步 300ms；开机回显 `[RDK] link: live only if 0x100 within <值>ms` 自动跟随 |
| `tools/regress/lib.sh` | 新增 `proto_num()`：从 `Inc/can_protocol.h` 现取协议级数值 |
| `tools/regress/04_rdk_link_normal.sh` | `HB_TIMEOUT` 改为 `proto_num` 派生（不再硬编码）；**T6 静默等待 0.5s→0.8s**；判活注释 300ms |
| `tools/regress/05_bxcan_sniff_swd.sh` | `HB_TIMEOUT_MS` 改为 `proto_num` 派生 |
| `tools/hw_verify_rdk_recovery.sh` | `HB_TIMEOUT` 改为从头部内联 grep 派生；**S8 停流 400→500ms**；判活注释 300ms |

设计要点：**判活阈值 = 单一真源 + 同文件范围护栏**。护栏范围 `[200,500]` 界定可接受区间
（下限防 0x100 抖动误刹、上限防放松失效保护）；`[200,500]` 之外的改动会**编译失败**，
必须人工确认。改动只落在 `04_stm32` 内（未动时钟/外设初始化，未改 ID/字节布局/周期）。

> ⚠ 判活阈值是**协议级安全参数**：改值需与板端 `can.protocol` / `.clinerules §5` 同步；
> 验收判据由“断 0x100 200ms 内刹停”改为 **≤300ms（检出 ≤320ms）**。


## 本轮修订（heartbeat_v1：判活只看 0x100 + 取消 STOPPING + HOLD 单出口）—— 2026-10-02

与板端语义收敛：**0x100 是唯一判活源**。只动 `04_stm32` 内文件（未动时钟/外设初始化，未改 ID/字节布局）。

| 文件 | 改动 |
|---|---|
| `Src/main.c` | ① 状态机重写为 STATIC/CRUISE/HOLD/LOCK 四态（删 STOPPING）；② 判活**只看 0x100**（`g_rdk_hb_ms`，删除双源判定）；③ **0x102 只在 STATIC 计数**（`byte1=0x01` 应答且 `byte2` 递增才计；`0x02` 心跳只记时刻）；④ HOLD 唯一出口 = 有效 0x100（byte3 变化）；⑤ 0x101 改 **200ms 恒发**（STATIC/CRUISE/HOLD），仅 LOCK 停发；⑥ 删 `stop_reason/coast/hs_fail/led_ovr/hold_reconnect` 等历史符号 |
| `Inc/can_protocol.h` | `0x102` **DLC 收紧为 8**（取消“≥2”例外）；文件头同步判活/握手说明 |
| `Inc/rdk_config.h`、`Src/rdk_config.c` | 参数按新语义收敛：`hs_period_ms`=200（恒发周期）、`hs_ack_need`=2、`hb_timeout_ms`=200、`lock_timeout_ms`=60000；删 `hs_retry_max / hs_timeout_ms / cm_per_pulse / stop_pulse_th` |
| `tools/regress/04_rdk_link_normal.sh` | 按新契约重写：T1~T8（含 `--lock` 验 60s→LOCK 停发 0x101） |
| `tools/regress/05_bxcan_sniff_swd.sh` | **新增**：不接 CANable 的总线抓包（改全通的过滤组 0 路由到 FIFO1 抢 RX，抓完还原）—— F1~F6 帧契约 + R1~R4 还原/安全；`--mode lb` 用静默环回自证 `0x101` 线上字节 + 注入 6 帧已知刺激证通路 |
| `tools/hw_verify_rdk_recovery.sh` | 重写为**无对端**验收：SWD 预装邮箱 + 单写触发 + 静默环回注入，5 段快照，开场 `verify_image` 预检（退出码 0/1/2/3/4/5） |

设计要点（都可追溯到 .clinerules §5）：**刹停是命令（cmd=03，状态仍 CRUISE）；失联才进 HOLD/LOCK**；
`0x102` 仅用于上电握手，不参与任何判活/恢复/超时。**上文“协议 / 状态机 / LED”三节即本节权威语义。**

### 真机验收（同一天第二段：真机烧写 + 免对端自证）—— 2026-10-02

板上已烧写本版（`openocd … -c "program build/car.elf verify reset exit"` → `** Verified OK **`），
用 **ST-Link + SWD** 完成自证（本轮工作机无 CANable；期间 RDK 端已在线，三个脚本都不依赖它）：

| 命令 | 结果 |
|---|---|
| `bash tools/hw_verify_rdk_recovery.sh` | **PASS=94 / FAIL=0**（① 判活铁律/制动语义/HOLD 恢复 ② 停车语义；快照 S0~S9 + B0~B5；退出码 0） |
| `bash tools/hw_verify_rdk_recovery.sh --lock` | **PASS=113 / FAIL=0**（＋③ LOCK：断总线 61s → `st=3`、`pwm=0`、`motor=3`、LED0 常亮、`req_count` 冻结在 301（0x101 停发）、LOCK 里再喂 0x102/0x100 也不响应；复位后 `st` 回 0/1、`req` 重新计数） |
| `bash tools/regress/05_bxcan_sniff_swd.sh --mode lb --dur 3000` | **21 ✅ / 0 ❌（1 提示〔lb 会摘出总线〕，退出码 0）**，**本机 openocd 0.11.0 与工具链容器 0.12.0+dev 各跑一遍，结果一致**：抓 21 帧 = `0x101`×15（`AA seq` 递增、byte2~7=0，周期 **200.0 / 200.1ms**）+ 注入 `0x100`×3 + `0x102`×3（应答 2 + 心跳 1）**全部抓到** → 环回下"固件发帧 → 线上 → 嗅探 → 解码"闭环成立（`req` 递增证明发在继续） |
| `bash tools/regress/05_bxcan_sniff_swd.sh --dur 6000`（Normal，RDK 在线；容器里用 `--dur 8000` 复跑） | **20 ✅ / 0 ❌（1 提示〔normal 抓不到自发 0x101，属 CAN 语义〕，退出码 0）**：抓 121 帧（= `0x100`×67 + `0x102`×54）/ 162 帧（= `0x100`×88〔cmd=0×60 + cmd=3×28〕+ `0x102`×74），**帧格式/DLC/未用字节/ID 白名单全过**；`0x100.byte3` 逐帧 +1、cmd∈{0,1,2,3}；`0x102` = `0x55` + byte1∈{1,2} + **应答(30/40)/心跳(24/34)共用 seq 且逐帧 +1**；R1 抓包窗口内 `g_rdk_hb_rx` 不动、R2 还原后 1s 内重新累加、`ESR=0`/`FIFO1` 无溢出、`BTR/FFA1R/INRQ/INAK` 按位还原、收尾 `st=1`(CRUISE) 且 pwm 与 cmd 自洽 |

**顺带发现并修掉的三处"脚本自身"问题**（固件无问题，都是注入手法/断言口径的问题）：

| 现象 | 根因 | 修法 |
|---|---|---|
| 偶发假 FAIL：`0x100 cmd=00` 那帧像"没生效"（同一序列里 S3 正常、B2 却显示 HOLD） | 注入原本"预装邮箱 + 只写 TIR"。固件自己的 0x101 由驱动层 `Can_Send()` 发，它取 **TME0 最低空邮箱 → 平时就是邮箱 0**，会用 0x101 的 DLC=2 / `AA seq` 覆盖预装内容 → 此后只写 TIR 就把**被覆盖的载荷**当 0x100 发出去（byte0=0xAA → 固件按 cmd 越界整帧丢弃 → 判活不刷新）。SWD 侧实测：预装不重写 **1/2** 命中，每次重写 **8/8** | `fire()` 每次**完整重写** DLC + TDLR + TDHR + TIR |
| 上电快照假 FAIL：RDK 在线时 `S0` 读到 `st=1 / ack=2`（复位 0.5s 就握上手） | `main()` 里 `Servo_SelfTest()` 等**阻塞初始化排在 `Can_Init` 之前** —— "复位后等 N ms 再切环回"要么写到尚未初始化的 CAN 寄存器（随后被 `Can_Init` 覆盖，实测 BTR 仍是 Normal），要么已经迟到 | 新增 `boot_iso()`：`reset halt` → 在 `Can_SendRdkHsReq`（**首帧 0x101 入口**）下**2 字节硬件断点**停住 → 切静默环回 → 放行。S0/B0 恢复确定性（`st=0 / ack=0 / BTR=0xC04E0003`） |
| 快照后半段依赖对端安静（S8/B4 的"0x100 静默 → HOLD"原本在 Normal 里做） | 刺激段在静默环回里、但"停流"段已 `lb_exit` 退回 Normal，对端一帧 0x100 就能改变结果 | 把"停流 → HOLD"搬回环回内（S8/B4），退出环回后只断言 `BTR/STBY`（S9/B5） |

> ⚠ 脚本会把 STM32 **摘出总线约 30s**（`--lock` 约 95s）。RDK 若没有 ABOM，可能因无人 ACK 进 bus-off ——
> 跑完把板端复位一次再联调（脚本只读/只注入，不碰板端）。

#### 现场联调实测（RDK 在线，ST-Link 只读两次、间隔 5s，未复位/未注入）

| 项 | 实测 | 说明 |
|---|---|---|
| `g_rdk_req_count` | 437 → 462（Δ25 / 5066ms） | **0x101 周期 = 202.6ms** ✓（规格 200ms） |
| `g_rdk_hb_rx` | 940 → 995（Δ55） | 板端 0x100 当前约 **92ms** 一帧（≥规格 50ms；板端 WIP 细节，可自查） |
| `age(0x100)` / `age(0x102)` | 30/50ms → 92/12ms | 两路都新鲜；0x102 对每帧 0x101 约 12ms 即回 ✓ 握手通路在 |
| `g_rdk_hb_dup` / `g_rdk_hb_bad` | 0 / 0 | 板端 byte3 逐帧递增、cmd 全合法 → 没有重复帧/越界帧 |
| `g_rdk_ack_count` | 2（CRUISE 期间不再变） | **0x102 只在 STATIC 计数**的直接现场证明 ✓ |
| `CAN_ESR` | 0 | 无错误帧/无 bus-off；`CAN_BTR=0x004E0003`（Normal）→ TJA1050 ↔ 板端物理通路健康 ✓ |
| `g_rdk_state` / `pwm` / `motor` | 1 CRUISE / 0 / 0(STOP) | 板端当前命令是 **cmd=00 停车**（coast），车静止 |

## 【历史·协议已被 2026-10-02 修订取代】本轮新增（heartbeat_v1 掉线恢复 + 判活"任一在收"）—— 2026-10-01

### 改了什么（只动本目录；协议 ID / 字节布局 / 周期 **零改动**）
| 文件 | 改动 |
|---|---|
| `Src/main.c` | ① 判活改"任一在收"：新增 `g_rdk_last_rx_100/102`（`volatile uint32`），CRUISE 看门狗改为"**两者都**静默 ≥200ms 才刹停"；② **掉线型 HOLD 自动重连**（立即发 0x101 → 每 2s 一帧 ≤30 帧 → 收满 2×0x102 自动回 CRUISE → 30 帧无应答锁死）；③ 新增 `Rdk_HoldReconnectOn()`（HOLD 子型判定）；④ `Task_1000ms` 状态行加 `age100=/age102=`；⑤ `Rdk_Init()` 复位 `stop_reason/last_rx_*/req_ms` |
| `Inc/can_protocol.h` | 文件头补"本端判活与掉线恢复"说明（**无宏/ID/字节改动**） |
| `Inc/rdk_config.h`、`Src/rdk_config.c` | 注释同步（`hb_timeout_ms`=两者都静默；`hs_timeout_ms/hs_retry_max/hs_ack_need` 与上电握手**共用**）。**没有新增参数** |
| `tools/regress/04_rdk_link_normal.sh` | 新增 **T8 掉线自愈**（10 项判定） |
| `tools/hw_verify_rdk_recovery.sh` | **新增**：免 CANable 的 SWD 真机验收（① 命令型 HOLD 锁死 ② 掉线型 HOLD 轮发重连 ③ 判活"任一在收"），刺激用 bxCAN **静默环回**自造；开场先 `verify_image` 校验板上 flash == 本版 elf（防旧固件符号错位） |
| `Makefile` | 新增**构建指纹** `build/.buildflags`（所有 `.o` 都依赖它）→ 换 `PROTOCOL`/`CAN_LOOPBACK` **不 `clean` 也整树重编**（根治"混合镜像"）；`clean` 一并删指纹；`.PHONY` 加 `FORCE` |
| `README.md` | 同步本轮：验收证据（PASS=36）、预检回显与反证配方/两个坑、构建指纹说明 |

设计要点（3 条，都可追溯到 .clinerules §5 的安全语义）：
1. **命令型 HOLD 仍然锁死**：板端主动 `cmd=03` 刹停后**不**自动恢复 —— 自动恢复前进等于擅自解除
   制动，属安全红线。自动重连**只**针对"本地判定掉线"这一种 HOLD（`reason=hb_lost`）。
2. **未握手不许动**：重连期间 PWM 恒 0（与 HANDSHAKE 同），只有收满 2×0x102 才允许输出动力。
3. **既有的"两者都静默才刹停"不放松**：单看 0x102 也能判活，但 0x100 一停就刹停的老行为被放宽为
   "任一在收即在线"（这是本轮要求；`0x102` 只在我们发 0x101 时来，同样是链路活的证据）。

`make clean && make`（默认 heartbeat_v1 / `PROTOCOL=brake_v2` / `CAN_LOOPBACK=1` 各组合）**0 警告 0 错误**。

### 本轮真机证据（SWD 观测，2026-10-01）
工作机**无 CANable**（`/dev/bus/usb` 只有 ST-Link），无法 `cansend` 造刺激；但**总线上有真实板端在网**
（`verify_image` 确认 flash == 当轮 elf（该轮构建 text 10324 B）；`TSR.TXOK0=1` 说明我们发的 0x101 **被对端 ACK**）。

| 观测点（SWD 读回） | 数值 | 说明 |
|---|---|---|
| `g_rdk_state` | **2 CRUISE** | 本次上电内处于巡航 |
| `g_rdk_stop_reason` | **2 hb_lost** | **本次上电内发生过"心跳超时→本地刹停"** |
| `g_rdk_ack_count` / `g_rdk_req_count` | 2 / 2 | 收满 2×0x102；0x101 发到第 2 帧（重连轮从头计） |
| `g_rdk_pwm_out` / `g_current_speed` | 300 / 300 | 巡航输出真的写进了电机 |
| `g_rdk_last_rx_100` / `g_rdk_last_rx_102` | 170ms 前 / **33.0s 前** | 两个判活时间戳**各自独立**在走（0x102 只在握手/重连时来） |
| `g_rdk_hb_rx` | 672 帧 | 板端 0x100 周期帧持续在收 |
| `g_rdk_v_cmps` | 41 cm/s | 编码器真反馈（与 T2b 的自由轮 42cm/s 一致） |
| `ESR` | 0 | 全程无错、无 Bus-Off |

**这一组数值的组合只有新代码能产生**：`Rdk_Init()` 已把 `stop_reason` 清零，所以
`state=CRUISE` 且 `stop_reason=2(hb_lost)` ⇒ 本次上电里确实走过
「心跳超时 → `STOPPING` → HOLD(掉线型) → 重连轮发 0x101（`req` 从 1 起）→ 板端应答 2×0x102（`ack=2`）
→ **自动回 CRUISE**」这条**新增**路径；旧代码在 HOLD 一旦进入就只会停在 `st=4` 等复位。

> ⚠ 上述是**在网板端**驱动的观测（真实链路、真实帧、真实 0x102 应答），但刺激不完全受控：
> "0x100 与 0x102 都静默 ≥200ms"这一初始条件由**板端侧**行为决定。

⚙ **受控复现（免 CANable）**：工作机 `/dev/bus/usb` 只有 ST-Link，本轮改用
**bxCAN 静默环回（`LBKM|SILM` → `BTR=0xC04E0003`，过滤器仍全通）自造刺激** —— 板端帧进不来、
自己发的帧被当收到，链路层面等价于"线上只有我"。要点：

1. **预装邮箱 + 单写触发**：先把 `TDTR/TDLR/TDHR` 预装进 3 个邮箱（T0=`0x102` DLC=2、
   T1=`0x100 cmd=03`、T2=`0x102` DLC=8），之后**每发一帧只写一次 `TIR`**。
   原因：本机每次 SWD `mww` 经 usbip 要 10~50ms，逐字段组装一帧（4 写）可能 >200ms，
   会把"注入太慢"误判成"固件超时"（本轮第一版就踩过：`cmd=01` 保活迟到 → 看门狗真的刹停）。
2. **BTR 必须在 `INAK=1` 的初始化窗口内写**（MCR→INIT→写 BTR→出），否则被忽略。
3. **`0x102` 只在 HANDSHAKE/HOLD 计数** → 注入测试必须先把状态推进到 HANDSHAKE/HOLD 再打，
   才能凑满 2 帧进 CRUISE。
4. 安全：全程压 `STBY(PB3)=0` → TB6612 高阻，**车轮不可能转**；每次 `reset run` 后 500ms
   （`Motor_Init` 之后、握手之前）重新压低。

```bash
cd 04_stm32 && make                          # heartbeat_v1（默认）
sudo bash tools/hw_verify_rdk_recovery.sh    # ~15s，末行 PASS/FAIL；FAIL>0 → 退出码 1
```

本轮实测 **PASS=36 / FAIL=0**（本脚本把"单次 openocd 会话 + 事后 awk 解析"贯彻到底：
usbip 下反复开会话 halt/读回**偶发空读**，成功率约五成）。其中 3 项是**关键反证**、1 项是**关键正证**：

| 反证项 | 实测（SWD 读回） | 只有新代码能成立 |
|---|---|---|
| 命令型 HOLD **不**自动重连 | `st=4`、`reason=1(cmd03)`、`pwm=0`、4.4s 内 `req` **不涨**、再连发 5 帧 `cmd=01` 仍 `st=4`/`pwm=0` | 若误把"重连"用到命令型 HOLD，`req` 会递增、且可能被 `cmd=01` 拉回巡航 —— **安全红线** |
| **只喂 `0x102` 仍巡航** | `age100=565ms`（0x100 已静默 >阈值）而 `age102=15ms` → `st=2`、`pwm=300` | 旧逻辑「只看 `0x100`」此刻必已 200ms 刹停 |
| 两路都静默才刹停 | `age100=967ms` **且** `age102=417ms` → `st=4`、`reason=2(hb_lost)`、`pwm=0` | 判活仍是"两路 OR"，没被放松成"任一断即停" |
| （正证）掉线型 HOLD **会**轮发重连 | 静默刹停后 `st=4(reason=hb_lost)`，再等 2.3s（>2s 重发周期）：`req` **1→2**，而 `st` 仍=4、`pwm=0` | 掉线型确实起重连轮发；但环回里没有对端回 `0x102`，所以**不自愈**（自愈需真对端，见上文在网证据 + `regress` T8） |

> ⚠ 该脚本需要 ST-Link。本机 SWD 链路**偶发** `Error: init mode failed (unable to connect to the target)`：
> 本轮复现过两次（openocd 只报到 `Target voltage: 2.88V`），间隔几秒再探即恢复
> （正常读回时 `Target voltage: 3.03~3.12V`）—— 属 ST-Link/usbip 重枚举，不是固件问题。
> 脚本已内置"整段重跑 ×3，仍读不到就判**环境问题**、打印 openocd 尾部并退出 4"，
> 避免把 SWD 链路故障误报成整片固件 FAIL。

> 📌 **本轮踩到的坑 → 已加护栏**：上一轮为验四种构建组合反复 `make`，`build/` 里是**增量叠出来的**
> 产物（Makefile 的 `.o` 规则只跟踪头文件、**不跟踪 `CFLAGS`**，所以切 `PROTOCOL`/`CAN_LOOPBACK`
> 之后不 `clean` 会残留旧宏的目标文件），而板上烧的是其中另一版 —— 于是 `make` 出来的 elf 与板上
> flash **不是同一份字节流**（`verify_image` 报 `checksum mismatch`，连 reset 向量都对不上）。
> 此时脚本拿 elf 的符号地址去读 RAM，读到的是**别的变量**：现象是 `st=0 / ack=21 / hsfail=7`
> 这类乱数，而固定地址的 `TIM3_CCR1=15` 反而正常 —— 会伪装成一堆固件 FAIL。
> 现在脚本在**同一会话开头**先 `verify_image`（10 KB ≈ 0.3 s）：通过才继续，并打印
> `✅ 预检：板上 flash == build/car.elf（openocd: verified 10280 bytes）`；不一致则**退出 5**、提示先烧写本版、
> 并回显 openocd 的 `diff` 行，同时**不再白重试 SWD**（镜像不匹配时 openocd 必中止会话）。
> 护栏本身也反证过：本工程 linker script 把 flash 镜像放在文件偏移 `0x10000`（`.isr_vector` file off `0x10000` ↔ LMA `0x08000000`），
> 所以取 `.text` 内 LMA `0x08000200`（file off `0x10200`）把原字节**取反**后重跑：
> `off=$((0x10000+0x200)); cp build/car.elf /tmp/bad.elf; b=$(od -An -tu1 -j$off -N1 build/car.elf|tr -d ' ');`
> `printf "\x$(printf %02x $((b^255)))" | dd of=/tmp/bad.elf bs=1 seek=$off conv=notrunc status=none;`
> `ELF=/tmp/bad.elf bash tools/hw_verify_rdk_recovery.sh` → 实测量到退出码 **5**、打印
> `diff 0 address 0x08000200. Was 0x7f instead of 0x80`，并且**不再白重试 SWD**。
> ⚠ 两个坑（本轮都踩了，记下以免复现时误判）：① 别写死"改成 `0x7f`"——当前镜像该处**本来就是 `0x7f`**，
> 空写 ⇒ `cmp -l` 差异 0 字节 ⇒ 护栏正确地报"一致"（不是护栏失效）。② 别改到**文件空洞**（如偏移 `4096` 处
> 8 字节全 0、不被任何 section 覆盖）：`verify_image` 按 section 比对，改空洞不会被发现，同属正常现象。
> 本节 PASS=36 是在"`make clean && make` → 烧写本版 → `verified 10280 bytes`"之后测的，复跑仍 PASS=36/FAIL=0（退出码 0），
> 所以**先 `clean` 再烧再验**这条顺序不能省（不过现在换宏忘 `clean` 已不会产出"混合"镜像，见下）。

### Makefile 构建指纹（本轮）：换宏不 `clean` 也不会再"混合"镜像
| 用例 | 操作 | 实测 |
|---|---|---|
| 空跑 | `make`（参数不变） | gcc 调用 **0 次**（指纹 mtime 不动）✅ |
| 切协议不 clean | `make PROTOCOL=brake_v2` | 整树重编 **7 单元** + `[build] build flags changed -> full rebuild`；`g_rdk_state` 符号消失 ✅ |
| 与洁净对照 | 上面产物 vs `make clean && make PROTOCOL=brake_v2` | `car.bin` sha256 **两侧相同** `6206a3f6…`（无旧 `.o` 残留）✅ |
| 切回默认不 clean | `make` | 整树重编 7 单元；sha256 **== 默认** `bba68734…`（T-A 洁净值）✅ |
| 组合宏不 clean | `make CAN_LOOPBACK=1` | 整树重编；sha256 与洁净构建相同 `b95cfde5…` ✅ |
| `clean` | `make clean` | `build/.buildflags` 一并删除 ✅ |
| 真机 | 默认构建 → `verify_image` | 板上 == 本版 elf（`verified 10280 bytes`）→ `PASS=36` ✅ |


### ⚠ 现存的**负载侧**问题：左右轮反馈符号相反（历史轮唯一 1 条提示）
`GPIOB_ODR=0x19` 是"A/B 通道都正转"（AIN1=1,AIN2=0,BIN1=1,BIN2=0），但编码器实测
**左 +45 / 右 −45 cm/s**（幅值一模一样，`encoder.c` 左右轮同一套换算、固件没有对右轮取反）——
即两路里有一路的**电机线极性或编码器 A/B 相序**与另一路相反。后果：

| 现象 | 原因 |
|---|---|
| 两轮同向指令却反着转 → **车原地打转而不是直行** | 某一路电机两根线接反（该路 `MOUT` 极性反）**或**该路编码器 A/B 相接反 |
| `g_speed_kmh=0`（0x125 报 0 km/h）、`g_wheel_diff_warn=1` | 0x125 走**带符号**两轮平均 `(vl+vr)/2 = +45−45 = 0 cm/s` → 0 km/h；`g_wheel_diff_warn` 按 30% 一致性判据置 1 |

**先看轮子**（上电自转一次）：
- 若某轮**物理倒转** ⇒ 反接的是**该轮电机线**（对调该路 `MOUT`/`BIN` 两根）；
- 若两轮物理都正转、只是读数一正一负 ⇒ 反接的是**该轮编码器 A/B 相**（左轮 TIM4_CH1/CH2 = PD12/PD13，
  右轮 TIM8_CH1/CH2 = PC6/PC7，对调该路两根或对应插针）。

⚠ 不要"为了让读数变正"就在固件里给某轮乘 −1 —— 先定因（电机线 or 相序），否则会把真故障掩盖成假正常。
修好后复跑 `tools/regress/04_rdk_link_normal.sh`：T2b 应变成 `✅ 左右轮同向为正`，`g_speed_kmh` 也应 >0。

### 一键复现（真机，~50s）
```bash
cd 04_stm32 && make clean && make            # Normal 镜像（0 警告 0 错误）
bash tools/regress/04_rdk_link_normal.sh     # 61 项判定：会驱动电机 ~1s，车轮请架空
CAN_IF=slcan0 bash tools/regress/04_rdk_link_normal.sh --no-reset   # 不复位（初态判定放宽）
sudo bash tools/hw_verify_rdk_recovery.sh    # 免 CANable：HOLD 两种子型（锁死/重连）+ 判活"任一在收"（36 项判定）
```

脚本流程：复位到初态 → 验 0x101 帧形态 → 2×0x102 握手 → 读回 `TIM3_CCR`/`GPIOB_ODR` 证"PWM 真到引脚" →
cmd=01 维持巡航 1s → cmd=02 预警 → cmd=03 刹停 → 再发 01 验**命令型 HOLD 锁死** → 复位后停发
0x100+0x102 验 200ms 失效保护 → **T8：掉线型 HOLD 每 2s 重发 0x101、恢复 2×0x102 自动回巡航**
→ 复查 `ESR`/`g_rx_overrun` 并停回 HOLD(PWM=0)。

> ⚠ **读回期间必须持续喂 0x100**（脚本用后台喂狗进程，独立计数域避免与主流程撞号被当重复帧）。
> 否则 200ms 看门狗会把刚起来的巡航掐掉 —— 那是**正确的固件行为**，不是失败（脚本第一版没喂，
> 误判了 18 条：`st=4/pwm=0/reason=hb_timeout`）。
> ⚠ 退出两种 HOLD 的方式不同：**掉线型**收到 2×0x102 会自动回巡航（本轮新增）；
> **命令型（cmd=03）**只能复位重新握手（这是设计，不是卡死）。`do_reset()` 用 openocd `reset run`，不重烧。
> ⚠ `--serial` 在本环境会被跳过（CH340 未 `usbipd attach` / 被 XCOM 独占），不影响 SWD 判定。


## 易错点
- STBY(PB3) 不拉高 -> 电机完全不动
- 高级定时器 TIM1 没开 MOE -> 舵机无输出（代码已处理）
- 用 CAN1 时别插 USB SLAVE 口（PA11/PA12 冲突）
- 接 TFTLCD 会占 PD12/PD13(FSMC_A11/A12)，接摄像头模块会占 PC6/PC7(DCMI) —— 小车不接没事
- 舵机不能从 STM32 3.3V 取电
- TB6612 VM 最高 13.5V，别上 4S 电池


openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "gdb_port disabled" \
  -c "init" \
  -c "reset halt" \
  -c "reg sp 0x20020000" \
  -c "reg pc 0x08000191" \
  -c "resume" \
  -c "shutdown"

项目侧操作（两种场景：手写寄存器 Makefile / CubeMX Make 工程）
⚠️ 硬性要求：代码放在 WSL 家目录 ~/xxx，不要放在 /mnt/c/ Windows 挂载目录，索引巨慢且容易出问题。
在 WSL 终端进入你的项目文件夹，用 code . 打开工程（必须 WSL 终端执行 code .）
# 场景 1：手写寄存器项目（自带 Makefile）
bash
# 清理旧编译产物 + 旧compile_commands.json
make clean
rm -f compile_commands.json

# bear捕获编译命令，自动生成compile_commands.json
bear make -j4
执行成功后，项目根目录会生成 compile_commands.json。
clangd 插件会自动识别这个文件，开始后台索引代码，右下角会出现 clangd 进度图标。

## 编译完，在 WSL 终端跑这个 (路径修正)

```
sed -i 's|"/workspace2/|"/home/fantasy/workspace2/edge_ai_car/|g' ~/workspace2/edge_ai_car/04_stm32/compile_commands.json

```

# 场景 2：CubeMX 生成 Makefile 工程
CubeMX 生成工程时，Toolchain 选：Makefile
将 CubeMX 输出的工程放到 WSL 家目录
WSL 终端进入项目目录执行：
bash
make clean
rm -f compile_commands.json
bear -- make -j4
每次在 CubeMX 修改外设、重新生成代码后，必须重新执行上面这两行，更新 compile_commands.json。
四、验证跳转功能
等待右下角 clangd 索引完成（进度条消失）
鼠标放到函数上，Ctrl + 鼠标左键，测试跳转定义；F12 跳转；Shift+F12 查找所有引用。



## 改代码后要不要重新跑？

表格

| 改了什么 | 要不要重新 bear make | 要不要跑 sed |
| --- | --- | --- |
| 只改 .c/.h 代码内容 | ❌ 不用 | ❌ 不用，clangd 自动更新 |
| 新增 / 删除 .c/.h 文件 | ✅ 要 | ✅ 要 |
| 改了 Makefile | ✅ 要 | ✅ 要 |

## 日常完整流程

```
# 终端1(容器):
cd /workspace2/04_stm32
make clean
rm -f compile_commands.json
bear make -j4

cd /workspace2/04_stm32
make clean
bear make -j4 && make flash


# 终端2(WSL):跳转失败
sed -i 's|"/workspace2/|"/home/fantasy/workspace2/edge_ai_car/|g' ~/workspace2/edge_ai_car/04_stm32/compile_commands.json
```
sed -i 's|/workspace2|/home/fantasy/workspace2/edge_ai_car|g' ~/workspace2/edge_ai_car/04_stm32/compile_commands.json

# 不折腾了，一行命令搞定，以后永远不用管，跳转成功

```
sudo ln -s /home/fantasy/workspace2/edge_ai_car /workspace2
```

- WSL 里 `/workspace2` 直接指向真实代码目录
- clangd 找 `/workspace2/04_stm32/Inc/platform.h` → 通过符号链接直接找到
- **不需要 sed, 不需要 path-mappings, 不需要清缓存，不需要重启任何东西**
- 以后容器里 `bear make` 完，直接跳转，什么都不用做

## 本轮调查（0x100 判活反复超时误刹 → 定位为板端周期性停发）—— 2026-10-11

**现象**：车架空时电机周期性"刹一下又走"，`0x100` 判活反复超时（阈值已放宽到 300ms）。

**结论（已定量，STM32 侧全程只读观测）**：问题**在板端 `0x100` 的发送节律**；STM32 与 CAN 总线均已排除。

| 观测项 | 实测 |
|---|---|
| 总判活超阈 | **277 次 / 1817s（30.3 分钟）= 9.1 次/分钟** |
| 四窗口恶化 | `0.4 → 1.0 → 6.3 → 17.8` 次/分钟（30 分钟内约 44 倍） |
| 最长 `0x100` **真实空档** | **30.47 s**（逐秒 `age100` 由 1179ms 单调涨到 30469ms）；另有 12.95 s 一例 |
| 晚期新增形态 | 连续 **50–58 s** 的"低速率期"（`age100` 始终 ≥300ms，等效速率 <1–2 Hz） |
| `0x100` 周期 | 实测 **94–111ms**（规格 50ms）；风暴期平均投放 **254ms**（≈60% 帧没到） |
| 停发的两族分布 | **67% 连 `0x102` 一起停**（全局停顿）/ **33% 只有 `0x100` 停**（主循环绑死） |
| `0x102` 实际节律 | **≈100ms 且与 `0x100` 恒差约 20ms**（并非独立 200ms 定时器） |
| 总线健康度 | `TxErr/RxErr/BusOff/Ovf/Rec` 全 **0**，`dup = bad = 0` ⇒ 总线干净 |
| 对照组自证 | `0x101`（STM32→板端）实测 **195ms**（规格 200ms）**合规** |

**关键判据**：`Ovf = 0` ⇒ 静默期间**总线上确实一帧都没有**（若 STM32 来不及收，3 级 RX FIFO 在 1.2s 内应溢出约 9 次），**排除 STM32 丢帧**；对照实验（停 SWD 采样 → 杀调试器 → 只留串口）现象不变，**排除观测仪器干扰**。

**为何调阈值无用**：安静期最长间隔仅 109ms（远小于阈值），而病态停发 **0.3–30.5 s**（远超阈值）⇒ 200 / 300 / 500ms 判定结果完全相同。
**安全红线**：30.47 s 已达 `lock_timeout_ms(60s)` 的 **51%** —— 一旦某次停发 >60s，STM32 进 LOCK（停发 `0x101`、需复位）。

**方法**：非停机 SWD 内存直读（3ms 分辨率，不停 CPU/不复位）+ 串口 1s 状态行双通道交叉验证。

**交付**：完整报告 → [`docs/板端问题报告.md`](docs/板端问题报告.md)（自包含，可直接转给板端）。

> ⚠ 给板端的请求（详见报告 §6/§7）：① 确认 `0x100` 是"无条件周期发"还是"有新决策才发"（30.47s 空档是否属正常语义）；
> ② 查 `0x100`/`0x102` 是否共用锁或共用同一个 CAN 写通道；③ 拆出**独立 50ms 发送线程**与推理耗时解耦；
> ④ 明确 `0x102` 的实际周期与相位（实测 ≈100ms，不是 200ms）。

> 本节**未改动任何源码/外设初始化**，仅新增文档 `docs/板端问题报告.md` 与本节记录。

