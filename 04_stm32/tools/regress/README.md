# tools/regress —— 可复现验收脚本（CAN 管线干跑 / 真机回环 / AEB 协议 / 回环自测 / RDK 行车链路 / bxCAN 嗅探）

对应 `04_stm32/README.md` 的 **§步骤① 回环自测** 与 **§步骤③ 真机联调**。
脚本都是"可运行 + 可复现"的：自动逐项判定（✅/❌/⚠）、失败返回非 0、
原始输出（openocd / candump / 串口）全部留在 `DUMP_DIR` 便于事后复查。
`01/02/03/04` 需要真机（`01/02/04` 还要 CANable）；`05` 只要 ST-Link；`00` 不需要硬件，用来先把"脚本这一侧"验干净。

| 脚本 | 复现什么 | 需要的硬件 | 典型耗时 |
|---|---|---|---|
| `00_canpipe_dryrun.sh` | **CAN 管线干跑**（vcan 虚拟 CAN，验抓包/统计/断言/发帧字节本身） | **不需要任何硬件** | ~5s |
| `01_hw_accept_normal.sh` | **真机回环验收**（Normal + CANable，500kbps「真」验证） | ST-Link + CANable + 120Ω×2 + 板子供电 | ~25s |
| `02_aeb_protocol_test.sh` | **AEB 协议契约测试**（0x123/0x124/0x125 逐条） | 同上 | ~20s |
| `03_loopback_selftest.sh` | **回环自测**（静默回环镜像，不需要 TJA1050/对端/终端电阻） | 只要 ST-Link + 板子供电 | ~15s + 编译 |
| `04_rdk_link_normal.sh` | **RDK 行车链路契约**（0x100/0x101/0x102：STATIC → 2×0x102 握手 → CRUISE → 00/01/02/03 命令 → **判活只看 0x100** 的 hb_timeout_ms(300) 失效保护 → HOLD → byte3 变化的 0x100 恢复；`--lock` 另验 60s→LOCK） | ST-Link + CANable + 120Ω×2 + 板子供电 | ~40s（`--lock` +65s；会驱动电机 ~1s） |
| `05_bxcan_sniff_swd.sh` | **不接 CANable 的总线抓包**：用 STM32 自己的 bxCAN 当嗅探器（临时把全通的过滤组 0 改路由到 FIFO1 抢 RX）→ RDK 真实帧的**原始字节/周期**；`--mode lb` 另用静默环回自证 0x101 线上字节 + 注入 6 帧已知刺激证通路 | 只要 ST-Link + 板子供电 | ~12s（`--dur 10000`） |


> `00` 是可选的"体检件"：它证明 **脚本这一侧**（candump 抓包、0x125 统计、
> DLC/保留字节/周期断言、`cansend` 线上字节）是好的。真机的位速率/BTR/锁存语义
> 仍必须由 `01/02/03` 判；但**先跑 00**，就不会把"脚本自己的 bug"误判成"板子不发帧"。

公共库：`lib.sh`（判定/抓包/串口/烧写/构建），`swd_read.py`（用 `nm -S` 现读符号，
生成 `mdw` 读内存计划并把结果解析成 `名字=值`；脚本里**没有硬编码地址**）。

## 0. 与"三条要求"的对应关系

| 你要求的三件事 | 对应脚本 | 它验的东西 | 依赖 |
|---|---|---|---|
| ① 真机正常验收：Normal 镜像下 CAN 能收发、500kbps 对得上、0x125 周期上报、发 0x123 能制动 | **`01_hw_accept_normal.sh`** | 前置（工具链/镜像类型/接口）→ 接口链路 → ①`ip -d link`/`slcand -s6` → ②代码侧位速率常量 → ③反向核对 `cansend` 能否真进总线 → ④**从真硅片寄存器反推位速率**（`RCC_CFGR.PPRE1`→PCLK1→`CAN1_BTR`，slcan 显示 `bitrate 0` 也不算没过）→ ⑤总线统计 → ⑥`0x125` 周期/信号 → ⑦`0x123`→`g_brake_level` 链路 | ST-Link + CANable + 终端电阻 |
| ② AEB 协议契约：`0x123/0x124/0x125` 的 ID、DLC、字节布局、周期、失效保护、Bus-Off 恢复 | **`02_aeb_protocol_test.sh`** | T1~T13 逐条：帧格式（标准帧/DLC=8/保留字节=0）、`0x123` 四级映射 + 只有 level0 才解锁存（T2/T3）、DLC≠8 **整帧拒绝**（T4/T5）、`0x124` 距离字节序/`0xFFFF` 占位（T6）、`0x125` 周期 <100ms + 小端车速（T7~T9）、`0xFFFF` 车速无效占位（T10）、Bus-Off 恢复（T11）、错误计数（T12）、末态安全（T13） | 同上（会发帧，车轮请架空） |
| ③ 回环自测：不接 TJA1050/对端也能自证 CAN 外设与收发路径 | **`03_loopback_selftest.sh`** | 用 `CAN_LOOPBACK=1` 构建**静默回环镜像**（`CAN_BTR.SILM=1`）烧进去，固件自身跑 10 步自测，脚本再经 SWD 复核 `g_lb_board_rounds`/`g_lb_tx_frames`/`g_lb_fail`/`g_lb_last_round_fail`/`g_lb_speed_echo`，末尾可用 `--restore` 自动还原 Normal 镜像 | 只要 ST-Link + 板子供电 |
| ④ RDK 行车链路：`0x100/0x101/0x102` 握手放行、命令状态机、断 0x100 300ms 内本地刹停 | **`04_rdk_link_normal.sh`** | T1~T8：初态 STATIC(`st=0`) 静止不许动 + `0x101` 帧形态（`AA seq 00×6`/DLC=8/周期 200ms）、`0x102` **DLC=2 短帧必须被拒**（T1c）、STATIC 忽略 0x100 只记账 + byte3 去重（T1d）、2×`0x102`（byte2 递增）→ CRUISE（`st=1`/`pwm=300`/`FORWARD`/`GPIOB_ODR=0x11`/`TIM3 CCR=15`）、cmd=01 维持（`hb_rx` 累加、dup/bad=0）、cmd=02 预警（`pwm=150`+`buzz=1`）、cmd=00 停车（`pwm=0` 但**仍 CRUISE**）、cmd=03 刹停（`pwm=0`+短路制动+**仍 CRUISE**）、**判活铁律**（只停 0x100 而 0x102 照发 → 仍 ≤300ms 进 HOLD）、HOLD 恢复三连（0x102 不能恢复 / 同 byte3 的 0x100 不能恢复 / **新 byte3 的有效 0x100** 才恢复）、收尾 `ESR`/`g_rx_overrun`=0；`--lock` 另验 60s→LOCK(3)+停发 0x101 | 同上（会驱动电机 ~1s，车轮请架空） |
| （额外送的一件）脚本自身的"体检件" | **`00_canpipe_dryrun.sh`** | 用内核 `vcan` 虚拟总线证明**脚本这一侧**是好的：`candump -n` 抓包、DLC/保留字节/周期断言、`0x125` 统计函数、`cansend` 的线上字节布局。**不需要任何硬件**、~5s。它不属于三条要求中的任何一条，目的是：万一真机判失败，先跑它就能区分"脚本自己写错"还是"板子/接线出问题" | 无（只要有 can-utils + vcan 模块） |
| （额外送的第二件）**没有 CANable 也能看线上真相** | **`05_bxcan_sniff_swd.sh`** | 借 SWD 临时把 CAN1 的**验收滤波器组 0 路由改到 FIFO1**（`FFA1R.FFA0=1`，收发器仍照常硬件 ACK，对端零副作用），由 openocd 掏空 FIFO1 → 把**总线上真实出现的整帧**dump 出来再解码断言：帧格式/DLC/未用字节/ID 白名单、`0x100.byte3` 跳号、`0x102` 应答与心跳共用 seq、`0x101` 周期，外加 `--mode lb`（静默环回）注入已知帧自证"注入→线上→嗅探→解码"通路。跑完把寄存器按位还原并复核固件恢复对话。代价：抓包窗口内固件收不到 `0x100` 会进 HOLD（`pwm=0` 安全态），还原后自动回 CRUISE | ST-Link + **heartbeat_v1** 镜像（**不需要 CANable / can-utils**） |

> 一句话：**①= 真机能不能跑通（链路）、②= 协议细节对不对（契约）、③= 不靠对端也能自证（回环）**；
> `00` 是把这三件事的"测量工具"先校准一遍，避免用坏尺子量板子；
> `05` 是"连尺子（CANable）都没有"时的替代测量手段 —— 但要注意它是**旁观者视角**（看线上真帧），
> 不能取代 ②/④ 里"发帧→看固件反应"的因果验证，两者互补才行。


## 1. 前置条件（配一次即可）

1. **接线**（照 `.clinerules §1`）：CAN1 = PA11(RX)/PA12(TX) → TJA1050（**VCC 5V**）；
   CANH/CANL/GND 三线接 CANable2.0，**GND 必须共地**，总线两端各 120Ω。
   ⚠ 用 CAN1 时**不要插板载 USB SLAVE**（PA11/12 与 OTG_FS 冲突）。
2. **ST-Link 进 WSL**（脚本靠它读运行态变量/寄存器）：
   `usbipd list` → `usbipd attach --wsl --busid <ST-Link 的 busid>`（VID:PID 0483:3748）。
   串口可选：`usbipd attach --wsl --busid <CH340 的 busid>`（VID:PID 1a86:7523）。
3. **CANable 起 slcan**（本环境实际在跑的就是这条）：
   `slcand -o -c -s6 /dev/ttyACM0` → `ip link set can0 up`。
   （`-s6` = 500kbps 速率档位，自定义速率可用 `-S 500000`；`-t hw` 用硬件时间戳；
   **第二参数缺省 → 内核接口名就叫 `can0`，不是 `slcan0`**。）
   本环境下 `ip -d link show can0` 会显示 `bitrate 0` + 一段 bittiming 常量表，
   这是 slcan 的正常表现（定时由适配器管），**不代表波特率没配**（见 §6 第 7 条）。
4. **root 通道**：`openocd` 要 `/dev/bus/usb`、`slcand` 要 `/dev/ttyACM*`。
   脚本检测到非 root 时会自动 `wsl.exe -u root -e bash ...` 提权；也可以直接 `sudo bash ...`。
   本机是**原生 Linux**（没有 `wsl.exe`）、且 `sudo` 需要密码时（脚本用的是**免密** `sudo -n`），
   就直接 `sudo bash tools/regress/05_bxcan_sniff_swd.sh …`（其它脚本同理）跑（已是 root 时脚本不再提权）。
   懒得每次 sudo 可加一条 udev 规则让 ST-Link 对登录用户可写（可选）：
   `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"' | sudo tee /etc/udev/rules.d/49-stlink.rules`。
5. **`05` 额外要求**：`openocd`（`interface/stlink.cfg` + `target/stm32f4x.cfg`）、
   `arm-none-eabi-nm`（现读符号）、`make` 产出的 `build/car.elf` 且是 **heartbeat_v1 镜像**
   （缺 `g_rdk_*` 符号时直接报错退出 3）。**不需要 CANable / can-utils / vcan。**
6. **`wsl.exe` 与免密 `sudo` 都没有时，用工具链容器当 root 通道**（本仓库现成镜像，`run_toolchain.sh` 同源；
   已实测通过 openocd **0.12.0+dev**）：

   ```bash
   cd /home/fantasy/workspace2/edge_ai_car
   docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb -v "$PWD":/workspace2 \
     -w /workspace2/04_stm32 -e REGRESS_OWNER="$(id -u):$(id -g)" \
     -e DUMP_DIR=/workspace2/04_stm32/build/regress-05 \
     --entrypoint /bin/bash autoedge-toolchain:v2 \
     -c 'bash tools/regress/05_bxcan_sniff_swd.sh --mode lb --dur 3000'
   ```
   ⚠ 一定要带 `-e REGRESS_OWNER="$(id -u):$(id -g)"`：否则容器里的 root 会把 `build/` 与日志目录
   的属主改掉（脚本的收尾钩子按这个变量还属主）。`DUMP_DIR` 指到 `build/` 下才不会被容器一起删掉。
   ⚠ 容器里的 openocd 是 0.12.0+dev，`echo` 整数**打十六进制**（`C0 0x1 0x0 …`）；本机 0.11.0 打十进制。
   脚本已用 `n2d()` 归一化，两种版本都能跑（见 §6 第 27 条）。

## 2. 快速开始

> **协议版本编译期二选一（默认 heartbeat_v1，见 `.clinerules §5`）**：
> `make` = heartbeat_v1（0x100/0x101/0x102）；`make PROTOCOL=brake_v2` = brake_v2（0x123/0x124/0x125）。
> 切换协议**必须先 `make clean`**（宏直接编进 .o，增量编译会残留上一协议的目标文件）。
> 脚本对镜像协议有要求：**①/②/③ 要 brake_v2，④ 要 heartbeat_v1**；`elf_check` 会用符号名
> 把不匹配的镜像挡下来并提示对应的 `make` 命令。

```bash
cd /home/fantasy/workspace2/edge_ai_car/04_stm32
bash tools/regress/00_canpipe_dryrun.sh              # ⓪ CAN 管线干跑（无需硬件，先确认脚本自己没毛病）

# ①/②/③ 验 brake_v2（AEB 契约 + 回环自测）：
make clean && make PROTOCOL=brake_v2                 # brake_v2 Normal 镜像（0 警告 0 错误）
bash tools/regress/01_hw_accept_normal.sh            # ① 真机回环验收（会烧写）
bash tools/regress/02_aeb_protocol_test.sh           # ② AEB 协议契约（会烧写；车轮请架空）
bash tools/regress/03_loopback_selftest.sh           # ③ 回环自测（自建 brake_v2 回环镜像+烧写）
bash tools/regress/03_loopback_selftest.sh --restore # ③ 跑完自动还原 brake_v2 Normal

# ④ 验 heartbeat_v1（行车链路）：
make clean && make                                   # 默认 heartbeat_v1 Normal 镜像
bash tools/regress/04_rdk_link_normal.sh             # ④ heartbeat_v1 行车链路（会驱动电机，车轮请架空）

# ⑤ 只有 ST-Link 也要抓总线（不需要 CANable；RDK 挂在总线上时最有用）：
bash tools/regress/05_bxcan_sniff_swd.sh             # ⑤ 抓真实总线 10s（RDK 的 0x100/0x102 原始字节）
bash tools/regress/05_bxcan_sniff_swd.sh --dur 5000  # ⑤ 抓 5s
bash tools/regress/05_bxcan_sniff_swd.sh --mode lb   # ⑤ 环回自证（验 0x101 线上字节 + 注入通路）
#   ⚠ 车轮请架空：抓包期间固件进 HOLD（PWM=0），还原后按 RDK 当前 cmd 恢复（cmd=01 会真的走）
#   ⚠ 需要 heartbeat_v1 镜像（符号 g_rdk_* 缺失即报错退出 3）
```

## 3. 覆盖参数

```bash
--no-flash           # 不烧写，直接验当前在跑的镜像（01/02/03）
--no-build           # 不编译，复用现有 elf（03）
--restore            # 结束后重建并烧写 Normal 镜像（03）
--serial             # 整场后台抓串口，末尾打印 [AEB]/[CAN-LB]/[AEB-TEST] 原文（01/02/03）
--frames=N           # 0x125 抓帧数，默认 60（01）
ELF=build/car.elf    # 覆盖 elf 路径
CAN_IF=slcan0        # 覆盖接口名（默认 can0）
SERIAL_DEV=/dev/ttyUSB0   # 覆盖串口设备（默认按 VID:PID 自动找）
--keep               # 保留临时 vcan0（00）
--no-reset           # 不复位，直接验当前运行态（04；初态判定放宽为 握手/巡航）
--mode normal|lb     # 05：normal=抓真实总线（默认 10s），lb=静默环回自证 0x101 线上字节 + 注入 6 帧
--dur 毫秒           # 05：抓包窗口，默认 10000，最小 2000（越大越能看周期/跳号）
DUMP_DIR=/tmp/xxx    # 覆盖日志目录（默认 /tmp/regress-$USER；被占用时自动换 mktemp -d）
REGRESS_AS_ROOT=1    # 已是 root 但想跳过提权检测（调试用）
```

## 4. 判定表（脚本到底在验什么）

### 00 CAN 管线干跑（vcan，无需硬件）

| 步骤 | 判定 | 期望 |
|---|---|---|
| ①② | can-utils / vcan0 可用（没有就自动 `modprobe vcan` + `ip link add`，收尾自动删） | 建成并 UP |
| ② | 接口 state / 族 / bitrate 呈现 | UP（`state UNKNOWN` 但 flags 含 UP 也算过）/ link-can / bitrate 0 只提示 |
| ③④ | 62 帧合成 0x125（含 1 帧 byte2≠0、1 帧 DLC=4） | count=62、dlc_bad=1、rsvd_bad=1、other=0 |
| ④ | 周期 | 20~30ms（20ms 定时 + cansend 进程开销）、max<100ms |
| ④ | 末帧载荷 vs "SWD 侧变量" 小端互证 | 一致（本干跑用 km/h=20 → `14 00`） |
| ⑤ | `can_send 123#03…/124#64…` 的线上字节（candump 反查） | `123 [8] 03 00…` / `124 [8] 64 00…` |

⚠ 干跑**不验**：真实位速率/`CAN1_BTR`、AEB 锁存语义、GPIOB 制动电平——那些只能真机读。

### 01 真机回环验收（Normal 镜像）

| 步骤 | 判定 | 期望 | 不达标先查 |
|---|---|---|---|
| ④ | `RCC_CR` HSERDY / `PLLRDY` | 1 / 1 | 外部 8MHz 晶振起振（`platform.c`） |
| ④ | `PLLSRC` / `SWS` | 1(HSE) / 2(PLL) | 时钟树配置 |
| ④ | SYSCLK / PCLK1 | 168000000 / 42000000 | `PLLM/N/P`、`PPRE1` |
| ④ | `CAN1_BTR` 字段 BRP/TS1/TS2/SJW | 3 / 14 / 4 / 0 | `can_driver.c` 的 `BTR=0x004E0003` |
| ④ | **反推位速率** | 500000 bps | 若≈1Mbps → `PPRE1` 误配（APB1=84MHz）历史坑 |
| ④ | `BTR.LBKM` / `BTR.SILM` | 0 / 0 | 板上还烧着回环镜像（跑完 03 记得还原） |
| ④ | `MSR.INAK` / `ESR.TEC/REC/BOFF` | 全 0 | INAK=1 卡初始化；TEC/REC≠0 → 波特率/终端电阻/共地 |
| ⑤ | 0x125 帧数（≥90% 目标帧数） | 54/60 | 板子是否在跑、是否插了 USB SLAVE、120Ω |
| ⑤ | DLC≠8 数 / byte2..7≠0 数 | 0 / 0 | `can_protocol.h` 打包 |
| ⑤ | 平均周期 / 最大周期 | 15..25ms / <100ms | `Task_20ms` 里的发送周期 vs 板端 `recv_timeout` |
| ⑤ | 载荷 = `g_speed_kmh` 小端（无效=FF FF） | 一致 | `AEB_SpeedUpdate` 换算与有效位 |
| ⑥ | level3 → `g_brake_level` / `g_brake_latched` | 3 / 1 | `main.c` AEB 0x123 分支 |
| ⑥ | `GPIOB_ODR & 0x3B`（急刹） | 0x3B=59 | `motor_driver` 短路制动电平 |
| ⑥ | level0 → 等级/锁存 | 0 / 0 | 唯一解锁路径 |
| ⑥ | `GPIOB_ODR & 0x3B`（滑行） | 0x08=8（仅 STBY 高） | 松刹车后应为滑行态 |
| ⑦ | TEC/REC/BOFF/INAK/Ovf/Rec | 全 0 | `g_rx_overrun≠0` → 接收轮询太慢 |
| ⑧ | 收尾 0x125 存活帧数 | ≥1 | 整轮刺激后链路仍通 |

### 02 AEB 协议契约（0x123 / 0x124 / 0x125）

| 用例 | 刺激 | 期望 |
|---|---|---|
| T1 | `123#0300000000000000` | `g_brake_level=3`、`g_brake_latched=1` |
| T2 | T1 后静默 1.0s（> `AEB_BRAKE_TIMEOUT_MS`=500ms） | **仍 3 / 1（刹车优先、不自动解除）** |
| T3 | `123#0100000000000000` | 等级=1，锁存**仍 1**（预警不制动也不解锁） |
| T4 | `123#0400000000000000`（level>3 非法） | 整帧被拒，状态不变（仍 1/1） |
| T5 | `123#03`（DLC=1） | 整帧被拒（数据段固定 8 字节） |
| T6 | `123#0200000000000000` | 等级=2、锁存=1 |
| T7 | `123#0000000000000000` | 等级=0、锁存=0（唯一解锁路径） |
| T8 | 解锁后 `123#0500000000000000` | 仍 0/0（非法帧不能搞乱已解锁状态） |
| T9 | `124#6400000000000000`（100cm 小端 64 00） | `g_dist_rx_count` +1 |
| T10 | `124#FFFF000000000000` | `g_obstacle_cm=65535`，计数 +1（共 +2） |
| T11 | `124#64`（DLC=1） | 计数不变（被拒） |
| T12 | `1FF#0000000000000000`（协议外 ID） | 等级/锁存/计数全不变 |
| T13 | 抓 20 帧 0x125 | DLC 全 8、byte2..7 全 0、周期 15..25ms、max<100ms、载荷与 `g_speed_kmh` 互证 |
| 收尾 | 全程 `ESR.TEC/REC/BOFF`、`g_rx_overrun` | 全 0（21 帧刺激不丢帧不报错） |

### 03 回环自测（静默回环镜像）

| 步骤 | 判定 | 期望 |
|---|---|---|
| ② | elf 含 `g_lb_*` | 必须是 `CAN_LOOPBACK=1` 构建的镜像 |
| ④ | `BTR.LBKM` / `BTR.SILM` | 1 / 1（真·静默回环，SILM 不驱动 CANTX） |
| ④ | `BTR & 0x3FFFFFFF` | `0x004E0003`（时序与 Normal 完全一致=500kbps） |
| ④ | `g_can_test_echo_bad` / `g_lb_fail` | 0 / 0（0x111 回读逐字节校验） |
| ⑤ | `g_lb_round` | ≥1（跑完整轮 10 步） |
| ⑤ | `g_lb_last_round_fail` / `g_lb_fail` | 0 / 0 |
| ⑤ | `g_lb_speed_echo` / `g_lb_speed_echo_ok` | 123 **或** 65535（0x125 小端回读 + DLC=8）<br>步 9=123、步 10=0xFFFF 都合法，以固件步断言 `g_lb_last_round_fail=0` 为准 |
| ⑤ | `g_can_test_rx_count` 与 `tx_counter` | `(rx-tx) mod 256 ∈ {0,255}`，rx ≥ 6 |
| ⑦ | 收尾 | 提示 / `--restore` 还原 Normal 镜像 |

### 04 RDK 行车链路契约（0x100 / 0x101 / 0x102）

| 用例 | 刺激 | 期望（SWD 读回 + 总线抓包） |
|---|---|---|
| T1 | 复位后（`do_reset` = openocd `reset run`，**不重烧**） | `st=0`(STATIC)、`pwm=0`（**未握手不许动**）、`g_state=0(STOP)`、`req=1..5`（0x101 已在发） |
| T1b | 抓 0x101（过滤 `101:7FF`） | `101 [8] AA xx 00 00 00 00 00 00`、DLC 全 8、周期 ≈200ms（`hs_period_ms`），无次数上限 |
| T1c | `102#5501`（**DLC=2 短帧**） | 整帧被拒：`g_rdk_ack_count` **不涨**（§5 数据段固定 8 字节，0x102 无短帧例外） |
| T1d | STATIC 里发 1 帧 cmd=01，再发**同 byte3** 的第二帧 | 忽略 0x100（`pwm=0`、`cmd` 不变）但 `g_rdk_hb_rx` 记账；第二帧 `g_rdk_hb_dup` +1（去重、不刷新判活） |
| T2 | `102#5501000000000000` + `102#5502000000000000`（DLC=8，**byte2 递增**） | `st=1`(CRUISE)、`pwm=300`、`g_current_speed=300`、`g_state=1(FORWARD)`、`GPIOB_ODR` 含 STBY/AIN1/BIN1 且不含 AIN2/BIN2、`TIM3_CCR1==CCR2==cruise_pwm×(ARR+1)/1000`（占空真进定时器）<br>⚠ 进 CRUISE 后 hb_timeout_ms(300) 内必须有有效 0x100 → 脚本用 `feed_start` 后台喂狗 |
| **T2b** | 同上（**负载已上电**：电机 VM + 编码器 5V） | `sval g_wheel_l_cmps` / `sval g_wheel_r_cmps` / `g_rdk_v_cmps` ≠ 0（`TIM4_CNT`/`TIM8_CNT` 有计数）→ 编码器闭环成立；**两轮异号 → 提示负载侧相序反了**<br>⚠ 负载未上电时恒 0 → 只出提示不判失败 |
| T3 | 持续喂 cmd=01 满 1s（100ms/帧） | 仍 `st=1`/`pwm=300`；`g_rdk_hb_rx` 累加 ≥5；`g_rdk_hb` 低 3 字节 = cmd/dist/conf 打包序；`dup` 不涨、`bad=0` |
| T4 | 持续喂 cmd=02 | `st=1`（预警只减速，不掉状态）、`pwm=150`、`g_rdk_buzz_on=1`、仍 FORWARD |
| T4b | 持续喂 cmd=00 | `pwm=0`、`g_state=0(STOP)`（滑行停车，不短路制动）、`st=1`（**停车 ≠ HOLD**） |
| T5 | 持续喂 cmd=03 | `st=1`（**已无 STOPPING 态**）、`pwm=0`、`g_state=3(BRAKE)`（短路制动）、`GPIOB_ODR&0x33=0x33`；负载上电时另判 **残速 ≤2 cm/s** |
| T6 | **判活铁律（核心回归）**：先巡航，再**只停 0x100**而 0x102 应答照发（80ms/帧），等 0.8s | `st=2`(HOLD)、`pwm=0`、`g_state=3(BRAKE)`；`age100 ≥300ms` 且 **`age102 <300ms`**（0x102 在流也挡不住判活） |
| T7 | HOLD 恢复三连：① 只发 `102#5501/5502/5508` ② 重发**同 byte3** 的 0x100 ③ 发**新 byte3** 的有效 0x100 | ① 仍 `st=2`/`pwm=0` 且 `ack_count` 不变（HOLD 里 0x102 不计数）② 仍 `st=2`/`pwm=0`，`hb_dup` +1 ③ `st=1`、`pwm=300`、`cmd=01`（**唯一恢复条件**） |
| T8 | （`--lock`）复位后全程不喂 0x102，静默等满 60s | `st=3`(LOCK)、`pwm=0`；2s 抓包 **0 帧 0x101**（LOCK 停发，需复位恢复）。默认跳过，约 +65s |
| 收尾 | 全程 `ESR.TEC/REC/BOFF`、`g_rx_overrun`；末态 `pwm` | 全 0；`pwm=0`（**不把车留在动状态**）；`EXIT` 钩子还会补发一次 cmd=03 兜底（CRUISE 下立即刹停） |

> ⚠ **T2/T3/T4 必须在读回期间持续喂帧**：脚本用 `feed_start/feed_stop` 在后台喂狗
> （独立计数域 `0xA0..`，避免与主流程 `CNT` 撞号被当重复帧丢弃）。进巡航后 > hb_timeout_ms(300) 没有有效 0x100
> 就会被看门狗本地刹停 —— 那是**固件的正确行为**；不喂狗会把 `st=2` 读成 `st=4/PWM=0/reason=2`
> （脚本第一版因此误判 18 条）。
> ⚠ HOLD 的**唯一出口**是收到一帧**有效 0x100**（DLC=8 && byte0∈0x00..0x03 && byte3 变化）：0x102 应答、
> 心跳、byte3 未变的重复 0x100 都**不能**解除 HOLD（T7 三连就是钉这三个反例）。只有 LOCK 必须复位。
> ⚠ `g_rdk_hb` 是 `Rdk_HbFrame_t`（**最近一帧打包内容**，低 3 字节 = cmd/dist/conf），
> 累计有效帧数叫 `g_rdk_hb_rx`；写断言别把两者搞混（会算出 `0x0D000000` 这种吓人的"心跳数"）。
> ⚠ **有符号变量要换取值函数**：轮速是 `sint32` → 用 `sval`（`.s32` 有符号解释）；
> `g_rdk_coast_cm` 已随 STOPPING 态删除（本版固件无 `float32` 全局）；直接用 `val` 会印出
> `4294967257`（= −39）、`g_rdk_v_cmps` 这类**位模式**，看着吓人且会把断言带偏。

### 05 bxCAN 嗅探（SWD，无 CANable）

**原理**：bxCAN 只把帧存进"命中的**最小索引**过滤组"对应的 FIFO。固件占着组 0（掩码全 0 = 全通 → FIFO0），
所以新加一组（如 14）**抢不到帧**（实测 0 帧）。本脚本反过来：抓包前把**组 0 路由改到 FIFO1**
（`CAN_FFA1R.FFA0=1`，一位），固件读 FIFO0 读空、帧全落 FIFO1 给脚本读；抓完把该位清回。
收发器仍由硬件 ACK ⇒ **对端（RDK）看到的是完全健康的总线**（不像静默环回会把对端顶进 error-passive）。

| 判定 | 验什么 |
|---|---|
| F1 | 帧格式：全部 `IDE=0`（标准帧）/ `RTR=0`（数据帧）/ `DLC=8` |
| F2 | `0x100`：byte0 ∈{0,1,2,3}、byte4~7=0、**byte3 逐帧 +1**（看得到跳号 ⇒ 能报丢帧） |
| F3 | `0x101`：byte0=`0xAA` + byte1 逐帧 +1 + byte2~7=0（**仅 `--mode lb` 可判**；normal 下报"不适用，属 CAN 语义"） |
| F4 | `0x102`：byte0=`0x55`、byte1 ∈{1(应答),2(心跳)}、**byte2 应答与心跳共用同一计数器且逐帧 +1**、byte3~7=0 |
| F5 | 总线上没有别的 ID（只有 `0x100/0x101/0x102`） |
| F6 | 周期 min/avg/max：`0x100` 只观测（RDK 侧节拍，实测 89.4ms，脚本按 ≤2.5×规格 放行）、`0x101` 环回下断 200ms（实测 200.0ms，间隔 195~205ms）、`0x102` 心跳 ≈200ms（实测 253ms，脚本按 0.75~1.6× 带宽判） |
| 自证 | `--mode lb` 另注入 6 帧已知刺激（3×`0x100` cmd=0/dist=137/conf=95/byte3=0x11~0x13 + 2×`0x102` 应答 seq 0x21/0x22 + 1×心跳 seq 0x23），**全抓到**才算"注入→线上→嗅探→解码"通路 OK |
| R1 | 抓包窗口内 `g_rdk_hb_rx` 原地不动（**证明确实把 RX 抢走了**，否则抓的是空气；允许 ≤2 帧切换瞬间在途帧） |
| R2 | 还原后 1s 内 `g_rdk_hb_rx` 重新累加（**证明 FFA1R 已交还**，固件从 HOLD 自动回 CRUISE） |
| R3 | 寄存器还原：`BTR`=Normal(004E0003)、`MCR.INRQ=0`、`MSR.INAK=0`、`FFA1R.FFA0=0`、`ESR`=0（按位判，不看整值） |
| R4 | 收尾安全：`st≠LOCK(3)`，且 `g_rdk_pwm_out` 与 RDK 当前 `cmd` 自洽（cmd=0/3 ⇒ pwm=0） |

> ⚠ **抓包期间固件一定进 HOLD**：它的 0x100 被抢走 ⇒ 判活超时(hb_timeout_ms) ⇒ PWM=0 + 短路制动（安全态，`0x101` 照发）。
> 这是**预期代价**，R1/R2 就是钉这件事；还原后收到带**新 byte3** 的 0x100 即自动回 CRUISE。
> ⚠ **normal 模式抓不到 0x101 不是故障**：CAN 节点不接收自己发的帧（无回环位时）。要验 0x101 线上字节用 `--mode lb`。
> ⚠ **`--mode lb` 会把板子摘出总线几秒**（`BTR.LBKM|SILM`）：对端在此期间收不到 ACK，板端若无 ABOM 可能进 bus-off，跑完建议复位对端。

## 5. 真机实测结论（2026-09-28 首跑；2026-10-02 经工具链容器复跑复核）

> 复跑走的就是 §1 第 6 条那条容器通道（`--entrypoint /bin/bash` + `REGRESS_OWNER`），
> 一次跑 `--mode lb --dur 3000`、一次跑 `--mode normal --dur 8000`，两条都 `rc=0`，
> 结论与首跑一致（21 ✅ / 20 ✅，各 1 条提示），`build/` 属主未被容器改坏。

六个脚本全部在真机上跑通（`00` 13 项、`01` 42~43 项、`02` 37 项、`03` 21 项、`04` 51 项〔负载上电后〕、
`05` 21 项〔`--mode lb --dur 3000`〕/ 20 项〔normal `--dur 6000`/`8000`〕，各带少量提示；
`05` 在**本机 openocd 0.11.0** 与**工具链容器 openocd 0.12.0+dev** 上各复跑一次，结论一致 —— 顺带暴露并修掉
"两个版本 `echo` 整数进制不同"的脚本缺陷，见 §6 第 27 条）。
首跑抓到 **1 个固件缺陷 + 1 个脚本自身竞争**，都已修：

| # | 现象 | 根因 | 处理 |
|---|---|---|---|
| 1 | `02` T3 失败：发 `123#01…`（level=1）后 `g_brake_level` 仍是 3 | 同一段里紧跟的 `123#03`（**DLC=1**）被解析层放行（`len >= 1`），byte0=3 把等级又推回 3。与 §5「数据段固定 8 字节」、应用层日志 `(DLC!=8 ... ignored)` 矛盾 —— **驱动层缺陷**（`can_driver.h` 本就写着"DLC 由本层负责"） | 驱动层收紧：`Can_ParseBrakeCmd`/`Can_ParseObstacleDist` 要求 `len == CAN_DATA_LEN(8)`，DLC≠8 **整帧拒绝**；`can_protocol.h` 新增 `CAN_DATA_LEN` 常量；固件解析器自测补 **case 8**（0x123 DLC=1 必须被拒，此前只测了 0x124 的短帧）→ `parser PASS (8/8)`。修后 `02` 真机 37/37 |
| 2 | `03` 报 `g_lb_speed_echo = 65535，期望 123` | **脚本竞争，不是固件问题**：`g_lb_speed_echo` 是"最近一次回读载荷"，随自测步滚动（步 9=123、步 10=0xFFFF），固定 `t+8.5s` 读到步 10 的值完全合法 | 03 接受 `123` **或** `65535` 两个合法载荷；"步 9 必须回读 123"交给固件自身步断言（`g_lb_last_round_fail=0` 覆盖整轮 10 步）。不要再把它改成单一期望值 |
| 3 | `04` 首跑 18 条失败：T2 读到 `st=4/PWM=0`，T3 心跳增量是乱码 | **脚本侧两个错，都不是固件问题**：① 握手成功后隔 0.35s 才发第一帧 cmd=01，超过 200ms 心跳看门狗 → 固件**正确地**本地刹停进 HOLD；② 把 `g_rdk_hb`（最近一帧打包内容 `Rdk_HbFrame_t`）当成了心跳计数 | ① 加后台喂狗 `feed_start/feed_stop`（读回期间持续喂，计数域 `0xA0..` 防撞号）；② 计数改用 `g_rdk_hb_rx`，`g_rdk_hb` 只用于验字节序（低 3 字节 = cmd/dist/conf）→ 复跑 47/47 |
| 4 | `04` 的 `--serial` 被跳过：`串口不可用/被 XCOM 占用` | CH340 没 `usbipd attach`（或 COM5 被 XCOM 独占）| **不算失败**：`--serial` 只是附加证据，判定全走 SWD 读回 + 总线抓包；要看串口原文先 attach 或直接看 XCOM |
| 5 | 更早一轮 `04` 结论写成"车轮未转 ⇒ 负载侧供电问题"，其实**当时负载真的没上电** | 板子电机 `VM`/编码器 5V 未供 → `TIM4_CNT=0`、速度恒 0 | 负载上电后**结论自动成立**：新增 **T2b**（编码器闭环）与 **T5 运动学判定**（残速回落 + 滑行距离），复跑 50 通过 + 1 提示。教训：这类"读数恒 0"的判定**必须写成"提示/条件判定"**（`warn`），否则同一脚本在"带载/空载"两种台面上会出现假失败/假通过 |
| 6 | `04` T2b 报"左右轮转向相反（左 +45 / 右 −45 cm/s，幅值相同）" | **负载侧**：两路里有一路的电机线极性或编码器 A/B 相序与另一路相反（`GPIOB_ODR=0x19` 双通道同向正转，`encoder.c` 左右同一套换算、固件无取反）；顺带 `g_speed_kmh=0`（0x125 带符号平均 = 0）、`g_wheel_diff_warn=1` | **不动固件**：先看轮子——某轮物理倒转 ⇒ 对调该轮电机线；两轮都正转只是读数异号 ⇒ 对调该轮编码器 A/B（左 PD12/PD13，右 PC6/PC7）。修好复跑应变成 `✅ 左右轮同向为正` 且 `g_speed_kmh>0`（脚本已把这条诊断固化，见 §7） |

`01` 的 ④（位速率真验证）在真硅片上的实测值，可作为后续排查的基准：
`RCC_CR=0x03037C83 PLLCFGR=0x07405408 CFGR=0x940A` → SYSCLK 168MHz / PCLK1 42MHz，
`CAN1_BTR=0x004E0003`（BRP=4,TS1=15,TS2=5,SJW=1）→ **反推 500000 bps**，
`MCR.INRQ=0`、`MSR.INAK=0`、`ESR.TEC=REC=BOFF=0`，`0x125` 实测均值 20~21ms、最大 59.5ms。

## 6. 实现要点（改脚本前必读，都是踩过的坑）

1. **SWD 只 `halt` + `resume`，绝不 `reset run`**：复位会清掉 `g_brake_latched`、
   `g_brake_rx_ms`、`g_obstacle_cm`，协议判定立刻失真。`lib.sh:swd_read_all()` 里
   失败还会重试 3 次（ST-Link 重枚举很常见）。
2. **符号地址/宽度不写死**：`swd_read.py` 用 `arm-none-eabi-nm -S` 现读
   （寄存器别名走内置 `REGS` 表），`plan` 把要读的符号合并成尽量少的 `mdw`，`emit`
   解析 `mdw` 输出成 `名字=值`，并给字宽 ≥4 的符号附带 `.s32`（有符号解释）。
3. **判"运行态"而不是判"日志"**：所有结论都基于 SWD 读回 + 总线抓包；
   串口（`--serial`）只是**附加证据**，抓不到不判失败（XCOM 会独占 COM5）。
4. **`candump -n N` 自然退出**：`timeout` 杀 candump 会丢管道缓冲（假"收不到帧"）；
   兜底用 `stdbuf -oL` 行缓冲。所以 `can_capture N secs` 里 `secs` 只是个上界。
5. **`slcand` 第二参数缺省时接口名是 `can0`**（不是 `slcan0`）：
   `can_if_autodetect` 会自动探测/dev/ttyACM* 对应的接口；`CAN_IF=` 可覆盖。
6. **串口设备按 VID:PID 认**（CH340 = `1a86:7523`），避免把 CANable（`16d0:117e`）
   当串口读。
7. **`slcan` 的 `bitrate` 字段常显示 0**：这不是故障，所以"500kbps 真验证"走
   **在线寄存器反推**（`RCC_CFGR.PPRE1` → PCLK1 → `CAN1_BTR` → bps），比看注释/文档可靠。
8. **`uart_printf` 不能格式化**（nano.specs 下 vsnprintf 会 HardFault），
   串口原文里的数字都是固件自写 itoa 打的；脚本只做 `grep -a` 原文提取。
9. **提权**：`maybe_reexec_as_root` 先试免密 `sudo`，再试 `wsl.exe -u root`；
   两者都不行就打印准确的手工命令（注意 root 路径可能与本环境不同）。
10. **`DUMP_DIR` 默认 `/tmp/regress-$USER`**，若被别的用户（如 root）占用会
    自动换成 `mktemp -d`，避免 Permission denied。
11. **0x125 统计只在 DLC=8 时查 byte2..7**：短帧少掉的字段由 `dlc_bad` 记账，
    不能重复算进"保留字节非 0"（否则一帧 DLC=4 会同时虚增 2 个计数器）。
12. **接口 UP 判定兼容 `state UNKNOWN`**：vcan 及部分 can_dev 驱动即使管理态已 UP
    也报 `state UNKNOWN`，此时以 flags 里的 `UP` 为准（真机 slcan/can0 仍是 `state UP`）。
13. **`CAN_SIDE/CAN_PREFIX` 在 `regress_init` 里就有默认值**：即便 `can_detect` 提前
    返回，`can_sh` 也不会炸成 `unbound variable` 这种看不懂的报错。
14. **离线自测**：`lib.sh` 里 ④ 的解码块、`can_stats_125`、`can_send` 字节布局都可
    离线复现——前者用桩值跑"正常/回环镜像/PPRE1 误配"三场景（PPRE1 场景必须报
    反推位速率 1000000），后者见 `00_canpipe_dryrun.sh`。
15. **跑完脚本后 `build/` 属主会交还给调用者**：脚本为了 openocd 会提权到 root，
    构建产物/日志目录本来会变成 `root:root`，导致调用者自己的 `make clean` 报
    `Permission denied`。`regress_init` 记下调用者（`REGRESS_OWNER`）并在 `EXIT`
    钩子 `regress_handback_build()` 里 `chown -R` 还回去 —— 所以脚本跑完可以立刻
    接着 `make clean && make`。若中途被 `kill -9`（不触发 EXIT），手动跑：
    `wsl.exe -u root -e bash -c 'chown -R $USER:$USER 04_stm32/build'`。
16. **别把 `make` 和脚本并行跑**：脚本开始时会查 `build/car.elf`，而 `make clean`
    正好在删它 —— 症状是脚本报"找不到 elf（先 make）"这种莫名其妙的 ❌/退出码 2。
    串行执行（脚本自己会构建/烧写）。

17. **测 RDK 时读回期间必须持续喂 0x100**：固件进巡航后 200ms 内没有**有效** 0x100 就本地刹停
    （这是验收要求，不是 bug）。SWD 会话本身要几百 ms，所以喂狗必须由后台进程在读取期间持续进行
    （`04` 的 `feed_start/feed_stop`），并且**喂狗帧的计数要与主流程计数错开**——固件按 `byte3` 去重，
    计数撞号的那帧会被整帧丢弃（既不执行命令也不刷新看门狗）。
18. **先分清"哪个符号是计数、哪个是载荷"**：`g_rdk_hb` = `Rdk_HbFrame_t`（最近一帧内容，低 3 字节
    cmd/dist/conf）、`g_rdk_hb_rx` = 有效帧累计、`g_rdk_hb_ms` = 最近有效帧时刻（看门狗基准）、
    `g_rdk_last_cnt` = 去重基准（上一有效帧 byte3）、`g_rdk_hb_dup`/`g_rdk_hb_bad` = 重复帧/坏帧计数、
    `g_rdk_ack_count`/`g_rdk_last_ack_seq` = STATIC 期应答计数与上一次应答 byte2、`g_rdk_cnt_seen` = 是否收到过 0x100。
    写断言前先 `nm` + 看 `Src/main.c` 声明，别按名字猜
    （踩过：把 `g_rdk_hb` 当计数器，得到 `0x0D000000` 这种值还以为是"心跳很多"）。
19. **`can_capture` 支持 ID 过滤**：第 4 个参数透传给 `candump`（如 `101:7FF` 只抓 0x101），
    用来在"板子在周期重发/总线很吵"时精确抓指定帧，避免被其它 ID 刷满 `-n` 计数提前退出。

20. **有符号/浮点符号别用 `val`**：`val` 返回无符号十进制（`-39` 会印成 `4294967257`）。轮速类（`g_wheel_l_cmps`/
    `g_wheel_r_cmps`/`g_rdk_v_cmps`）用 `sval` 取 `.s32`（本版固件已无 `float32` 全局：`coast` 随 STOPPING 态删除，
    `lib.sh:f32()` 保留仅作将来之用）。判"同向/异号"这类断言**必须先换成有符号解释**，否则符号判断全是错的。
21. **寄存器别名只加"真的用过"的**：`swd_read.py` 的 `REG` 表补 `TIM4_CNT`/`TIM8_CNT`（= `TIM4_BASE+0x24`/
    `TIM8_BASE+0x24`，基址与字段序照抄 `Core/Inc/stm32f407_regs.h`）。它们是**20ms 采样窗内的增量**
    （`Encoder_GetDelta` 读完会清零），单次读取可能是 0 → 只用于显示/佐证，别拿它写硬断言。
22. **bxCAN 抢 RX 的唯一办法是改"组 0"的路由**（`05` 的核心，实测过）：帧只进**命中的最小索引过滤组**
    对应的 FIFO；固件的组 0 掩码全 0（全通）必然先命中 ⇒ 新加一组（哪怕也全通，索引 14）**一帧都抢不到**。
    正确做法是 `CAN_FFA1R.FFA0=1` 把组 0 改路由到 FIFO1，读完再清位。改位必须在 INIT 窗口内
    （`MCR.INRQ=1` → 等 `MSR.INAK=1` → 改 `FFA1R` → `MCR.INRQ=0` → 等 `INAK=0`），
    否则位不会生效。改路由**不影响收发器 ACK** ⇒ 对端总线完全健康（不会像静默环回把对端顶进 bus-off）。
23. **`--mode normal` 抓不到自己的 0x101 是 CAN 语义，不是 bug**：无环回位时节点不接收自己发的帧。
    要验 0x101 的线上字节，必须让固件自己的帧进 RX FIFO —— 即 `05 --mode lb`（`BTR.LBKM|SILM` +
    `reset halt` 后放行，让固件重跑 `Can_Init` 写回 Normal BTR，再由脚本切环回）。
24. **openocd 跑的是 Jim Tcl，不是 Tcl 8.x**：`05` 的抓包脚本里**不要在花括号列表里用 `\` 续行**——
    实测这样写会把 `set sched {{a b c d} \` 换行 `{e f g h}}` 解析成 7 个元素（Tcl 8.6 是 6 个），
    于是 `lindex $e 3` 取到空串 → `mww <addr> ''` 报 `Invalid command argument: value option value ('') is not valid`。
    调度表一律写成**单行**，并给注入函数加空值守卫（`05` 的 `inject` 已加）。
25. **SWD 抓包能追多少帧**：实测稳态 ≈1000 次寄存器访问/s（≈1ms/次）；总线 ~20 帧/s 时余量充足。
    漏帧只发生在**开抓瞬间**（FIFO1 只有 3 格）——所以判定不靠"帧数相等"，而是靠 `0x100.byte3` /
    `0x102.byte2` 的**跳号计数** + `RF1R.FOVR1` 溢出计数（正常 0~几帧）。
26. **寄存器判定按位写，不要比整值**：`MCR`/`MSR` 会随模式位变化（如 lb 模式实测 `MSR=00000208`/`00000E08`，
    Normal 是 `00000C08`）——用 `rbit() { echo $(( (0x$1 >> $2) & 1 )); }` 取 `INRQ`(bit0)/`INAK`(bit1)/
    `FFA0`(bit0)，比 `case` 里比整值稳（`05` 的 R3 就是这么判的）。
27. **不同 openocd 版本 echo 整数的进制不同**（踩过，且是**跨环境跑**才会现形的坑）：本机 **0.11.0** 打十进制
    （`C0 1 0 345 …`），工具链容器里 **0.12.0+dev** 打十六进制（`C0 0x1 0x0 0x79e …`）——因为
    `mem2array` 的值被直接 `echo` 出去了。后果是 `case "$ST5" in 3)`、`[ "$RQ3" -gt "$RQ1" ]`
    这类比较**退化成字符串比较**，同一条其实正常的链路会冒出
    `⚠ 快照 C5 st='0x1' 未识别` / `⚠ g_rdk_cmd='0x0' 非预期` / `⚠ 抓包期间 0x101 没发出`。
    修法：`05` 里 `fld()` 统一过一遍 `n2d()`（`0x… → $(( 0x… ))` 十进制），
    `kv()` 取的是 Tcl `[hex …]` 格式化值（`%08X`，无前缀）不受影响。**写新脚本沿用这个习惯**。
    真机对照：同一条硬件链路，0.11.0 与 0.12.0+dev 都是 `21 ✅（lb）` / `20 ✅（normal，8s）` + 1 提示。

## 7. 常见失败与排查（按报错行名）

| 现象 | 大概率原因 | 处理 |
|---|---|---|
| `make flash 失败` / `unable to connect to the target` / `target voltage may be too low` | ST-Link 的 VTref 没接到板子 3.3V：SWD 线松、板子没上电 | 检查板子电源 + 排线；`usbipd list` 确认 0483:3748 已 attach |
| `脚本启动就报"找不到 build/car.elf"` | 与 `make clean`/`make` 并行跑了（构建产物被删的瞬间） | 串行执行；`make` 完再跑脚本 |
| 用户侧 `make clean` 报 `Permission denied` | 脚本提权跑过，`build/` 属主是 root（正常会被 EXIT 钩子还回；`kill -9` 时不会） | `wsl.exe -u root -e bash -c 'chown -R $USER:$USER 04_stm32/build'` |
| `符号 X 取不到` | elf 与板上固件不是同一份（拿回环 elf 读 Normal，或反之） | `make clean && make` 后重跑；用 `ELF=` 指向与板上一致的 elf |
| 0x125 一帧都收不到 | 板子没跑 / 烧的是回环镜像(LBKM=1) / 用 CAN1 时插了 USB SLAVE / 没共地 / 无 120Ω | 先跑 03 确认回环通；再查 120Ω 与 GND 共地 |
| `ESR.TEC/REC ≠ 0` | 波特率不一致、终端电阻缺失、CANH/CANL 反接 | 用 01 的 ④ 输出对比 PCLK1/BTR；量 CANH-CANL 差分 |
| `INAK=1` | 板子卡在 CAN 初始化（`Can_RecoverIfBusOff` 会在 1s 内拉回） | 再读一次；仍 1 就看串口 `[CAN]` 行 |
| 收到帧但内容不对 | 字节序/字节布局不符（多字节必须小端、未用字节填 0） | 对照 `Inc/can_protocol.h` 与 `.clinerules §5` |
| T2（静默 1s）掉锁存 | 应用层缺"刹车优先、不自动解除" | `main.c` AEB 段：只有 level0 才清 `g_brake_latched` |
| `T3/T4` 等级被 `123#03` 之类的短帧改写 | 「历史缺陷」解析层曾接受 DLC≠8（`len>=1`） | 已修：`can_driver.c` 的 `len != CAN_DATA_LEN` 必须在；固件自测应打 `parser PASS (8/8)` |
| `--serial` 无输出 | CH340 没 attach / XCOM 独占 / 权限 | Windows 侧 `usbipd attach --wsl --busid 1-3`，关掉 XCOM，脚本用 root 跑 |
| `04` T2/T3/T4 读到 `st=2(HOLD)`/`pwm=0`（旧版是 `st=4`/`reason=2`） | 读回期间没喂 0x100（或握手成功后到第一帧 >200ms）→ **固件正确地本地刹停了** | 用脚本自带的 `feed_start/feed_stop` 后台喂狗，且每帧 `byte3` 必须递增；别手写"先握手 → 再读 → 再发命令"的序列 |
| `04` T7b 卡在 HOLD（发 0x102、或重发同 byte3 的 0x100 都不动） | **这就是本轮设计**：HOLD 唯一出口 = 收到**新 byte3** 的有效 0x100 | T7c 用 `(last_cnt+1)&0xFF` 造新计数即恢复；要退出 LOCK 才必须复位。`--no-reset` 只在明确知道板子当前在 STATIC/CRUISE 时用 |
| `04` T1b 抓不到 0x101 | 板子已在 **LOCK**（LOCK 才停发 0x101）；或 CANable 没接 / 没共地 / 接口没 UP | 复位后重跑；`--no-reset` 时该判定自动放宽为提示（同 T1 的 req 区间判定） |
| `04` T4 期望 `pwm=150` 却读到 300 或 0 | 300 = cmd=02 帧间隔 >200ms 后又超时前的一瞬；0 = 已刹停 | 喂狗间隔保持 ≤100ms；改 `Src/rdk_config.c` 的 `cruise_pwm/warn_pwm` 时**必须同步**脚本顶部常量（已注释标明对应字段），否则是"假失败" |
| `04` T2b "编码器无脉冲（`g_rdk_v_cmps=0`）" | **负载侧**：TB6612 `VM` 电机电压未供 / 编码器 5V 未接 | 量 VM 与编码器供电后复跑；这条**只提示不判失败**（软件链路由 T2 的 `TIM3_CCR`/`GPIOB_ODR` 判定） |
| `04` T2b "左右轮转向相反（左 +45 / 右 −45 cm/s）" | **负载侧**：某轮电机线接反（该轮物理倒转，车原地打转）或该轮编码器 A/B 反（读数符号反） | 看轮子定因：某轮物理倒转 → 对调该轮电机线；两轮都正转 → 对调该轮编码器 A/B（左 PD12/PD13，右 PC6/PC7）。⚠ 别在固件里给某轮乘 −1 掩盖故障；修好后 T2b 变 `✅ 同向为正`、`g_speed_kmh>0` |
| `04` 运动学断言读到 `0x4xxxxxxx` 这种大数 | 本版固件已无 `float32` 全局（`coast` 随 STOPPING 态删除）；把 `sint32` 轮速（`g_rdk_v_cmps`）用 `val` 无符号读了 | 有符号量一律用 `sval`；`lib.sh:f32` 保留仅作将来之用 |
| `00` 报"建 vcan0 失败" | 内核没编 vcan 模块（`modinfo vcan` 空） | 换环境或用 `CAN_IF=can0` 直接抓真总线（但那就不是干跑了） |
| `00` 抓 0 帧 | `CAN_IF` 指向了真接口 can0（vcan0 才是回环） | 别覆盖 `CAN_IF`；或 `ip link show vcan0` 确认存在且 UP |
| `symbol` 之类的 `unbound variable` | 某处没先做 `can_detect` / `swd_build` | 报给脚本维护者（库已有安全默认值，不该再出现） |
| `05` 报"`0x101` 抓不到属正常"（normal 模式） | **不是故障**：CAN 节点不接收自己发的帧 | 要验 `0x101` 线上字节就跑 `--mode lb`；normal 下这条只当提示 |
| `05` R1 报"固件仍在收 0x100"（`g_rdk_hb_rx` 涨了 >2） | `FFA1R.FFA0` 没生效（改位不在 INIT 窗口内 / 写被覆盖）| 看日志 `C1 ... FFA1R=…`：应为 `00000001`；若为 0 说明改位失败，抓到的帧不完整（R3 也会报 `FFA1R` 未还原） |
| `05` 报 `E9 调度表项损坏` / `value option value ('') is not valid` | openocd 的 **Jim Tcl** 对花括号列表里的 `\` 续行处理与 Tcl 8.6 不同（会把一项拆成两项） | 调度表写**单行**（见 §6 第 24 条）；`inject` 的空值守卫会先报 `E9` 再返回，不会污染后续判定 |
| `05 --mode lb` 报 `环回模式下没抓到 0x101` | 环回没切进去：`BTR` 写入失败或固件随后又重写了 `BTR`（`Can_Init`） | 看日志 `C1 BTR=…` 应为 `C04E0003`；若为 `004E0003` 说明没进环回（本脚本已用 `reset halt` → 放行 → 再切环回的顺序避免） |
| `05` 报 `注入自证：… 只抓到 N 帧` | 嗅探链路本身有问题（FIFO1 没读干净 / 邮箱被固件抢占） | 先看 `--mode lb` 下 0x101 是否正常；注入用**邮箱 1**（固件 0x101 走 TME0 最低空邮箱=邮箱 0），若固件改用邮箱 1 需同步脚本 |
| `05` 退出码 3（符号缺失） | 跑的是 `brake_v2` 镜像（无 `g_rdk_*`） | `make clean && make` 出 heartbeat_v1 再跑 |
| `05` 抓到的 0x100 周期是 ≈90ms 而非规格 50ms | **RDK 侧节拍**（板端代码不在本仓库，不由本端断言） | 只看趋势：脚本给 `warn`，稳定即可；要卡死 50ms 需同步板端 |
| `05`/`04` 报 `st='0x1' 未识别`、`g_rdk_cmd='0x0' 非预期`、`req 0x6 → 0x15` 却判"没发出" | **openocd 版本差异**：0.12.0+dev 的 `mem2array` 值被 `echo` 成十六进制，字符串比较就失真（0.11.0 是十进制） | `05` 已用 `n2d()` 归一化（§6 第 27 条）；若别的脚本也冒出这种"看着对却判错"，先把值过一遍 `n2d()` 再比 |
| 容器里 `05` 报 `Permission denied` / `make clean` 后 `build/` 变 root 属主 | 忘了传 `REGRESS_OWNER`，容器 root 改了属主 | 重跑带 `-e REGRESS_OWNER="$(id -u):$(id -g)"`（§1 第 6 条）；已变 root 属主时在容器里 `chown -R <uid>:<gid> /workspace2/04_stm32/build` |
| 容器里 `docker run … bash -c '…'` 报 `/bin/bash: cannot execute binary file` | 镜像 entrypoint 已是 `/bin/bash`，再跟一个 `bash` 会被当成脚本路径 | 用 `--entrypoint /bin/bash … -c '…'`（不要写成 `… autoedge-toolchain:v2 bash -c '…'`） |

## 8. 文件清单

```
tools/regress/
├── README.md                  # 本文档
├── lib.sh                     # 公共库：判定/记账、SWD 读回、有符号/浮点取值（sval/f32）、
│                              #          candump 抓包统计、串口、烧写/构建、
│                              #          提权+EXIT 属主交还（脚本跑完 build/ 仍是调用者的）
├── swd_read.py                # nm -S 现读符号 → mdw 计划 → 结果解析（无硬编码地址；
│                              #          寄存器别名含 GPIOB_ODR / TIM3_* / TIM4_CNT / TIM8_CNT / CAN1_* / RCC_*）
├── 00_canpipe_dryrun.sh       # ⓪ CAN 管线干跑（vcan，无需硬件；先跑它排除脚本自身故障）
├── 01_hw_accept_normal.sh     # ① 真机回环验收（Normal + CANable，500kbps 真验证）
├── 02_aeb_protocol_test.sh    # ② AEB 协议契约（0x123/0x124/0x125 共 13 个用例 / 37 条判定）
├── 03_loopback_selftest.sh    # ③ 回环自测（静默回环镜像，21 条判定 + --restore 还原）
├── 04_rdk_link_normal.sh      # ④ RDK 行车链路契约（0x100/0x101/0x102，T1~T8；--lock 另验 60s→LOCK）
└── 05_bxcan_sniff_swd.sh      # ⑤ bxCAN 嗅探（不用 CANable：改过滤组 0 路由到 FIFO1 抢 RX，
                               #    F1~F6 帧契约 + R1~R4 还原/安全；--mode lb 用静默环回自证 0x101 与注入通路）
```

六个脚本都遵守 `.clinerules`：只读/只写 `04_stm32` 内文件、不改外设初始化、
不引入 HAL、不改 CAN 协议；`01/02` 跑完都会把制动状态留在 **level=0**（安全态），
`03` 可用 `--restore` 自动还原 Normal 镜像；`04` 默认跑完停在 **CRUISE + 已刹停**（cmd=03 → PWM=0 + 短路制动，
状态不锁死），`--lock` 则停在 **LOCK**（需复位），其 `EXIT` 钩子在任何退出路径都会补发一次 cmd=03；
`05` 抓完把 `CAN_FFA1R.FFA0` 清回、把 FIFO1 读干净，固件从 HOLD 自动回 CRUISE（不停发、不锁态），
**按 RDK 当前 cmd 恢复 ⇒ cmd=01 时车会真的走，车轮请架空**（要停在 `--dur` 后自己 `cansend` 一帧 cmd=03）。

> `tools/hw_verify_rdk_recovery.sh`（上一层目录）是**不接 CANable** 的版本：用 bxCAN **静默环回**
> （`BTR.LBKM|SILM`）把帧注入自己的 RX FIFO，纯 SWD 读回判定；开场先 `verify_image` 对齐板上 flash 与 elf。
> 快照 S0~S9（判活铁律 / 制动语义 / HOLD 恢复）+ B0~B5（停车语义），`--lock` 再加 L0~L3
> （断总线 61s → LOCK：停发 0x101 + LED0 常亮 + 复位恢复）。**RDK 在不在总线上都能跑**：
> `boot_iso()` 用硬件断点停在**首帧 0x101 发送入口**、先切进静默环回再放行，上电瞬间就杜绝了
> 板端 0x102 把 STM32 带进 CRUISE（否则 `S0` 会读到 `st=1` 假失败）。
> 副作用：脚本运行期间 STM32 约 30s（`--lock` 约 95s）不在总线上，板端若无 ABOM 可能进 bus-off，
> 跑完建议复位板端。两者互补：`04` 证「总线上真的这么收发」，`hw_verify` 证「只有 ST-Link 也能复现同一套状态机语义」。

## 9. 退出码约定

| 码 | 含义 |
|---|---|
| 0 | 全部判定通过（**⚠ 提示不影响退出码**：`lib.sh:finish()` 只在有 ❌ 时返回非 0） |
| 1 | 有判定失败（❌） |
| 2 | 前置不满足（工具/elf 类型/接口/vcan 创建/构建/烧写；`05` 的 `--mode`/`--dur` 非法也走这里） |
| 3 | SWD 读回失败（ST-Link 未 attach、elf 不匹配、目标掉电）；`05` 的 `g_rdk_*` 符号缺失（跑错协议镜像） |
| 4 | `tools/hw_verify_rdk_recovery.sh`：切入/退出静默环回（`BTR.LBKM/SILM`）失败 |
| 5 | `tools/hw_verify_rdk_recovery.sh`：`verify_image` 不一致 —— 板上 flash ≠ 本版 elf（先 `make && make flash`） |

`DUMP_DIR` 里会留下 `*.dump`（openocd 原始输出）、`*.vals`（符号=值）、
`*.stats`（0x125 统计）、`*.txt`（candump/串口原文），失败时可直接把这些贴出来定位。
