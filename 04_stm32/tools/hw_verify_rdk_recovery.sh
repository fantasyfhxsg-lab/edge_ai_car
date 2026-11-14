#!/bin/bash
# ============================================================================
# hw_verify_rdk_recovery.sh —— RDK 行车链路真机验收（**无对端**：SWD 注入 + CAN 静默环回）
#                              纯 SWD 读回判定（无需 CANable、无需板端在线）
#
# 与 tools/regress/04_rdk_link_normal.sh 的分工：
#   04（总线侧）用 CANable 抓包验 0x101 周期/帧格式 + 真实收发；
#   本脚本（板内侧）不接任何对端，用 BTR.LBKM|SILM 静默环回把帧"注入"进自己的 RX FIFO，
#   按固件状态机逐条核对 —— 适合手边只有 ST-Link 的场合。
#
# 固件依据：Inc/can_protocol.h「RDK 行车链路协议」+ Src/main.c 状态机（heartbeat_v1）
#   st：0 STATIC / 1 CRUISE / 2 HOLD / 3 LOCK
#   判活铁律：**只看 0x100**（静默 ≥ hb_timeout_ms(300) → 本地刹停 → HOLD）；0x102 只在 STATIC 参与握手计数
#   HOLD 唯一恢复条件 = 收到有效 0x100（DLC=8 && byte0≤3 && byte3 与上帧不同）
#   byte3 相同 = 重复帧：丢弃，且不刷新判活时间戳
#
# 覆盖（快照 S0~S9 / B0~B5；`--lock` 另加 L0~L3。判定全用 nm 现读符号地址 + SWD 读回，无硬编码）：
#   ★ 全程"静默环回"隔离：LBKM|SILM 把 CAN_RX 与总线断开 —— 无论 RDK 是否正挂在总线上
#     （它对每帧 0x101 都回 0x102，上电 0.3s 内就能把 STM32 带进 CRUISE），本脚本看到的
#     都只有自己注入的帧。因此 S0/B0 的"复位初态"、S8/B4 的"0x100 静默 → HOLD"都**与对端无关**。
#     ⚠ 代价：脚本运行期间 STM32 会被"摘出"总线几十秒；RDK 若没有 ABOM，可能因无人 ACK 进
#       bus-off —— 跑完建议把板端复位一次再联调（本脚本不碰板端）。
#   S0  复位初态（环回已隔离）：STBY(PB3)=0（TB6612 高阻 → 车轮不可能转）、st=0 STATIC、
#       PWM=0、CCR1=0、0x101 已在周期发出（req≥1）、还没有任何 0x100/0x102 记账
#   S1  环回生效（BTR=0xC04E0003 = LBKM|SILM）
#   S2  注入 2×0x102（DLC=8，byte2 递增）+ 1 帧 0x100 cmd=01 → st=1 CRUISE：
#       PWM=300、斜坡 cur=300、CCR1 真写进 TIM3、AIN1/BIN1=H（0x11）
#   S3  0x100 cmd=03 → **st 仍=1 CRUISE**（本轮已无 STOPPING/命令型锁死）：PWM=0 + CCR1=0 +
#       motor=BRAKE(3) —— 制动是"命令"，不是状态锁死
#   S4  只喂 0x102、**0x100 静默 ≥300ms** → st=2 HOLD：PWM=0 + brake；
#       同时 age102<300ms（0x102 在收也挡不住判活 ⇒ 与旧"任一在收"相反）
#   S5  HOLD 里继续发 0x102 → st 仍=2 HOLD、ack 计数不涨（0x102 只在 STATIC 计数）
#   S6  重发与上一帧**同 byte3** 的 0x100 → st 仍=2 HOLD（去重丢弃，且不刷新判活）；hb_dup 递增
#   S7  改 byte3（新计数）的 0x100 cmd=01 → st=1 CRUISE、PWM=300（HOLD 唯一恢复条件）；
#       且 req 较 S5 递增（HOLD 期间 0x101 一直在发）
#   S8  环回里停流 500ms → st=2 HOLD + PWM=0（0x100 静默 → 本地刹停；仍在环回里 → 不受对端影响）
#   S9  退出环回 + 归还 STBY → BTR 回 Normal、STBY=1（总线交还 RDK，故不断言 st/pwm）
#   ②  停车语义：0x100 cmd=00 → PWM=0 但 **st 仍=1 CRUISE**（停车 ≠ HOLD）、motor=STOP(0)；
#       随后 cmd=01 立刻回到 PWM=300；B4 = 环回里停流 500ms → HOLD；B5 = 退出环回 + 归还 STBY
#   ③  --lock（可选，+65s）断总线 61s → LOCK：st=3、PWM=0、motor=BRAKE、LED0 常亮（PF9=0）、
#       0x101 停发（5s 后 req_count 不变）、LOCK 里再喂 0x102/0x100 也不恢复；复位后 st 回 0/1
#
# 为什么"单次 openocd 会话"：本机 ST-Link 走 usbip，反复开会话时 halt/读回**偶发空读**
#   （实测成功率 ~50%）。改成"一次 init 跑完全程"，用 `echo SNAP_x` 打标记、事后 awk 解析。
#   cfg-file 里的 mdw 不打印读回值，只有 `-c` 追加的命令才打印 → 全部命令都用 -c 拼在一条会话里。
#   另注：CPU 被 halt 时 SysTick 不涨 → millis() 冻结，所以快照里的 age 是"冻结那一刻"的一致视图。
#
# 注入发帧方式：**每次发帧都把 DLC + 数据 + TIR 完整重写一遍**（4 个 `mww`）。
#   慢一点没关系：`mww` 期间内核被 halt，**目标时间只在 `sleep` 里前进**，所以墙钟变慢
#   不会把判活阈值（hb_timeout_ms）吃掉。
#   ⚠ 不能"预装一次、之后只写 TIR"：固件自己的 0x101 由驱动层 Can_Send() 发出，它取
#     **TME0 最低空邮箱 → 平时就是邮箱 0**（Normal 下它卡住时才轮到邮箱 1），会用 0x101 的
#     DLC=2 / 数据 `AA seq` 覆盖我们预装的载荷；此后只写 TIR 就会把**被覆盖的载荷**当 0x100
#     发出去（byte0=0xAA → 固件按"cmd 越界"整帧丢弃 → 判活不刷新 → 伪 FAIL，现象见 2026-10-02
#     真机：S1/S3 通过而 B2 假 HOLD）。实测：预装不重写 → 邮箱 0 注入时好时坏（1/2）；
#     每次重写 → 同一序列连续 8/8 命中。
#
# 安全（重要）：
#   * SWD 把 STBY(PB3) 拉低 → TB6612 整片高阻，**车轮不可能转**；boot_iso 停在"第 1 帧
#     0x101 发送处"（Motor_Init 已跑完）立刻拉低，全程不归还直到断言跑完。
#   * 不烧写、不改 Flash，只 reset run + 挂 SWD（Flash 仅做只读 `verify_image` 校验）；退出前归还 STBY。
#   * 结束时 lb_exit 回 Normal 并把 STBY 拉起，车停在 HOLD（PWM=0）。
#
# 用法（在工具链容器里，必须 root：/dev/bus/usb/* 属 root）：
#   bash run_toolchain.sh                        # 进容器（--privileged + mount /dev/bus/usb）
#   cd /workspace2/04_stm32 && make && make flash # 先保证板上 == 本版 elf（见下）
#   sudo bash tools/hw_verify_rdk_recovery.sh          # ①+②：约 30s，末行 PASS/FAIL，FAIL>0 → 退出码 1
#   sudo bash tools/hw_verify_rdk_recovery.sh --lock   # 再跑 ③ LOCK 段（+65s ≈ 共 95s）
#   ★ RDK 在不在总线上都能跑：刺激段全程在静默环回里做（见文件头"覆盖"）
#   可选：ELF=build/car.elf（默认）指定取符号+校验用的镜像；不用 BUILD=build_check 之类的临时目录
#
# 开场预检（重要）：本脚本用 `arm-none-eabi-nm` 从 elf 现读符号地址再去读 RAM，所以
#   **板上 flash 必须就是这份 elf**。若板上是旧构建，读到的会是别的变量（现象：st/ack 这类值
#   全是乱数，而固定地址的 TIM3_CCR1 又看着正常）→ 伪造成一堆 FAIL。故会话开头先
#   `verify_image`（只读，≈0.3s）：不一致时报 checksum mismatch/diff 并**退出 5**。
#
# 退出码：0=全 PASS；1=有 FAIL；2=不在容器/04_stm32 目录；3=符号缺失（不是 heartbeat_v1 镜像）；
#         4=SWD 链路连不上（环境问题，已自动重试 3 次）；5=板上 flash 与本版 elf 不一致。
#
# 前提：编译宏为 heartbeat_v1（默认；brake_v2 无 g_rdk_* 符号 → 脚本直接报"符号缺失"退出 3）
# ⚠ 用 CAN1 时禁止插板载 USB SLAVE 口（PA11/PA12 与 OTG_FS 冲突，见 .clinerules §1）
# ============================================================================
set -u
cd /workspace2/04_stm32 2>/dev/null || cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "$PWD" | grep -q '04_stm32$' || { echo "❌ 请在工具链容器里、于 04_stm32 目录下运行（bash run_toolchain.sh 进容器）"; exit 2; }
ELF=${ELF:-build/car.elf}
LOCK_RUN=0                                   # --lock：追加 ③ LOCK 段（+65s，需断总线 61s）
case "${1:-}" in
    --lock) LOCK_RUN=1 ;;
    "")     ;;
    *)      echo "用法：bash $0 [--lock]"; exit 2 ;;
esac
OCD=(openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'gdb_port 0')

GPIOB_BSRR=0x40020418; GPIOB_ODR=0x40020414
GPIOF_ODR=0x40021414                         # LED0=PF9（低电平点亮）：LOCK 常亮断言用
CAN_MCR=0x40006400; CAN_BTR=0x4000641C
CAN_FA1R=0x4000661C
TX0TIR=0x40006580; TX0TDTR=0x40006584; TX0TDLR=0x40006588; TX0TDHR=0x4000658C
BTR_NORMAL=0x004E0003; BTR_LOOP=0xC04E0003        # 500kbps：BRP=3 TS1=14 TS2=3 SJW=1（现役值）
STBY_LOW=0x00080000; STBY_HIGH=0x00000008         # GPIOB_BSRR：PB3 置位(复位用 0x08)/清位(0x80000)

# ---- 判活阈值从 Inc/can_protocol.h 现取（唯一真源，改头文件即跟随）；PWM 标定与 Src/rdk_config.c 同步 ----
CRUISE_PWM=300; WARN_PWM=150
HB_TIMEOUT=$(grep -oE '^[[:space:]]*#define[[:space:]]+RDK_HB_TIMEOUT_MS_DEF[[:space:]]+[0-9]+u?' Inc/can_protocol.h 2>/dev/null | grep -oE '[0-9]+' | head -1)
HB_TIMEOUT=${HB_TIMEOUT:-300}
ST_STATIC=0; ST_CRUISE=1; ST_HOLD=2; ST_LOCK=3
MOTOR_STOP=0; MOTOR_FORWARD=1; MOTOR_BRAKE=3
CMD_PARK=0; CMD_CRUISE=1; CMD_ALERT=2; CMD_STOP=3

echo "======== RDK 真机验收（无对端：SWD 注入 + 静默环回）======"
echo "[info] ELF = $ELF   （板上 flash 必须与它一致，脚本开头会 verify_image 校验）"

# ---------------------------------------------------------------- 符号（nm 现读）
sym(){ arm-none-eabi-nm -n "$ELF" | awk -v s="$1" '$3==s{print "0x"$1; exit}'; }
A_ST=$(sym g_rdk_state);      A_RQ=$(sym g_rdk_req_count);  A_AK=$(sym g_rdk_ack_count)
A_PW=$(sym g_rdk_pwm_out);    A_CS=$(sym g_current_speed);  A_MO=$(sym g_state)
A_R100=$(sym g_rdk_last_rx_100); A_R102=$(sym g_rdk_last_rx_102); A_TK=$(sym g_ticks)
A_HD=$(sym g_rdk_hb_dup);     A_HX=$(sym g_rdk_hb_rx);      A_LN=$(sym g_rdk_last_cnt)
for n in A_ST A_RQ A_AK A_PW A_CS A_MO A_R100 A_R102 A_TK A_HD A_HX A_LN; do
    eval "v=\$$n"; [ -n "$v" ] || { echo "符号缺失：$n —— 当前 elf 不是 heartbeat_v1 镜像？先 make（默认 heartbeat_v1）"; exit 3; }
done

# ---------------------------------------------------------------- 会话骨架
# 组装**一条**长命令序列（同一 openocd 会话；只有 -c 追加的命令才打印读回值）
ARGS=(-c init -c halt -c "verify_image $ELF" -c resume); SNAPS=()
snap(){ ARGS+=(-c "echo SNAP_$1"); SNAPS+=("$1"); dump; }
cmd(){ ARGS+=(-c "$1"); }
boot_reset(){ cmd "reset run"; cmd "sleep 500"; cmd "mww $GPIOB_BSRR $STBY_LOW"; }   # 复位后立刻压 STBY
# 复位 → 停在"第 1 帧 0x101 即将发出"处 → 切静默环回 → 放行（保证 STATIC 初态可复现）
#   为什么不用"复位后等 N ms 再 lb_enter"：main() 里 Servo_SelfTest() 等阻塞初始化排在
#   Can_Init 之前（复位后 CAN 寄存器还没配置，写 BTR 会被随后的 Can_Init 覆盖），而 RDK 对
#   第 1 帧 0x101 就回 0x102，两帧 200ms 就把 STM32 带进 CRUISE。2026-10-02 真机实测：
#   RDK 在线时"复位 +0.5s 再 lb_enter"完全无效（快照 BTR=Normal / st=1 / ack=2）。
#   只有停在"第 1 帧发送入口"切环回，S0/B0 的复位初态才与"RDK 在不在总线上"无关。
A_BP=$(sym Can_SendRdkHsReq)
[ -n "$A_BP" ] || { echo "符号缺失：Can_SendRdkHsReq（当前 elf 不是 heartbeat_v1 镜像？）"; exit 3; }
A_BP=$(( A_BP & ~1 ))                             # Thumb 符号可能带 bit0 → 断点用偶地址
boot_iso(){ cmd "reset halt"
            cmd "bp $A_BP 2 hw"                   # 断点=第 1 帧 0x101 的发送入口（Can_Init 已完成）
            cmd "resume"
            cmd "wait_halt 10000"                 # 命中即停（初始化含舵机自检，可能 1~2s）
            cmd "rbp $A_BP"
            cmd "mww $GPIOB_BSRR $STBY_LOW"       # 停在 halt 中压 STBY → TB6612 整片高阻
            lb_enter                              # 切静默环回：RDK 的 0x102 之后再也进不来
            cmd "resume"; }
# dump 字段顺序（解析器按位置取值，改这里必须同步改下面的 ST()/RQ()/... ）：
#   1 ST 2 RQ 3 AK 4 PW 5 CS 6 R100 7 R102 8 TK 9 HD 10 HX 11 LN 12 MO 13 CCR1 14 ODR 15 BTR
#   16 FA1R 17 GPIOF_ODR（PF9=LED0，低电平点亮）
dump(){ ARGS+=(-c halt)
    ARGS+=(-c "mdb $A_ST 1");   ARGS+=(-c "mdw $A_RQ 1");   ARGS+=(-c "mdb $A_AK 1")
    ARGS+=(-c "mdh $A_PW 1");   ARGS+=(-c "mdh $A_CS 1");   ARGS+=(-c "mdw $A_R100 1")
    ARGS+=(-c "mdw $A_R102 1"); ARGS+=(-c "mdw $A_TK 1");   ARGS+=(-c "mdw $A_HD 1")
    ARGS+=(-c "mdw $A_HX 1");   ARGS+=(-c "mdb $A_LN 1");   ARGS+=(-c "mdb $A_MO 1")
    ARGS+=(-c "mdw 0x40000434 1"); ARGS+=(-c "mdw $GPIOB_ODR 1")
    ARGS+=(-c "mdw $CAN_BTR 1");  ARGS+=(-c "mdw $CAN_FA1R 1")
    ARGS+=(-c "mdw $GPIOF_ODR 1")
    ARGS+=(-c resume); }
# 快速进/出静默环回（BTR 必须在 INRQ=1 的初始化窗口内写，否则被忽略）
lb_enter(){ cmd "mww $CAN_MCR 0x00010045"; cmd "sleep 20"
            cmd "mww $CAN_BTR $BTR_LOOP";  cmd "sleep 20"
            cmd "mww $CAN_MCR 0x00010044"; cmd "sleep 20"; }
lb_exit(){  cmd "mww $CAN_MCR 0x00010045"; cmd "sleep 20"
            cmd "mww $CAN_BTR $BTR_NORMAL";cmd "sleep 20"
            cmd "mww $CAN_MCR 0x00010044"; cmd "sleep 20"; }
# 邮箱预装 / 触发：三个发射邮箱**任一都可能被固件自身出帧覆盖**（见文件头"注入发帧方式"）
#   stage  = 4 写（TDTR/TDLR/TDHR + TIR=0 清待发），并记住该邮箱"应发"的 DLC + 数据
#   retag  = 1 写（只改 TDLR = byte0..3），同时更新"应发"数据
#   fire   = 每次都把 DLC + 数据 + TIR 完整重写再置 TXRQ —— **不依赖之前预装的内容**
TXS=0x10
tir_of(){ printf '0x%X' $(( ($1 << 21) | 1 )); }         # 标准帧 ID → TIR（TXRQ=1, IDE=0, RTR=0）
declare -A PAY_DLC=( [0]=8 [1]=8 [2]=8 )                 # 每邮箱应发 DLC
declare -A PAY_TDLR=( [0]=0 [1]=0 [2]=0 )                # 每邮箱应发数据 byte0..3
stage(){ PAY_DLC[$1]=$2; PAY_TDLR[$1]=$3
         cmd "mww $(( TX0TDTR + TXS * $1 )) $2"
         cmd "mww $(( TX0TDLR + TXS * $1 )) $3"
         cmd "mww $(( TX0TDHR + TXS * $1 )) $4"
         cmd "mww $(( TX0TIR  + TXS * $1 )) 0"; }
retag(){ PAY_TDLR[$1]=$2
         cmd "mww $(( TX0TDLR + TXS * $1 )) $2"; }
fire(){  cmd "mww $(( TX0TDTR + TXS * $1 )) ${PAY_DLC[$1]}"
         cmd "mww $(( TX0TDLR + TXS * $1 )) ${PAY_TDLR[$1]}"
         cmd "mww $(( TX0TDHR + TXS * $1 )) 0"
         cmd "mww $(( TX0TIR  + TXS * $1 )) $(tir_of "$2")"
         cmd "sleep ${3:-20}"; }
M_HB=1    # 邮箱1：0x100 当前帧（cmd/dist/conf/cnt）
M_HB2=0   # 邮箱0：0x100 备帧（换帧内容时交替用；固件 0x101 平时就用邮箱0 → fire 每次重写）
M_102=2   # 邮箱2：0x102 应答/心跳
# 帧数据（DLC 一律 8；byte4..7 = 0 → TDHR 恒 0）
#   0x102：55 01 seq 00 | 0…  → TDLR = seq<<16 | 0x0155
#   0x100：cmd dist conf cnt | 0…（dist=20=0x14, conf=90=0x5A）→ TDLR = cnt<<24 | 0x5A1400 | cmd

echo
echo "======== ① 判活铁律 / 制动语义 / HOLD 恢复（快照 S0~S9）========"
# ★ 先隔离总线再等 1.2s：boot_iso 停在"第 1 帧 0x101 发送处"切静默环回 —— RDK 若正挂在总线上，
#   它对每帧 0x101 都回 0x102，复位后 0.3s 内就能把 STM32 带进 CRUISE，那样 S0/B0 必然假失败。
boot_iso; cmd "sleep 1200"                   # 上电 1.2s：0x101 已发若干帧、仍未握手
snap S0
stage $M_102 8 0x110155 0x0                  # 0x102 应答 seq=0x11（55 01 11 00 | 0…）
stage $M_HB  8 0x215A1401 0x0                # 0x100 cmd=01 cnt=0x21
stage $M_HB2 8 0x235A1403 0x0                # 0x100 cmd=03 cnt=0x23（备用帧）
snap S1                                      # 环回生效（BTR=LBKM|SILM）
fire $M_102 0x102 20                         # 应答 1
retag $M_102 0x120155                        # 应答 2：seq 递增（0x12）—— byte2 不递增会被丢
fire $M_102 0x102 60                         # → 20ms 服务里 acks=2 → CRUISE
fire $M_HB  0x100 120                        # 进巡航后 hb_timeout_ms(300) 内必须有有效 0x100（否则立刻 HOLD）
snap S2                                      # 期望 CRUISE + PWM=300 + CCR1 写入 + 前进引脚
retag $M_HB2 0x235A1403                      # byte3 换新（0x23）+ cmd=03
fire $M_HB2 0x100 40                         # ★ 制动命令：车停，但**状态还是 CRUISE**（本轮无 STOPPING）
snap S3
retag $M_HB 0x245A1401                       # cmd=01 cnt=0x24 → 回到巡航前进
fire $M_HB 0x100 120
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do fire $M_102 0x102 40; done   # ★ 只喂 0x102：0x100 静默 >300ms
snap S4                                      # 期望 HOLD(2) + PWM=0，且 age102<300ms（0x102 挡不住判活）
fire $M_102 0x102 20                         # HOLD 里继续发 0x102 应答
retag $M_102 0x100255                        # byte1=0x02 → 心跳 kind（55 02 10 00 | 0…）
fire $M_102 0x102 20
fire $M_102 0x102 20
snap S5                                      # 期望仍 HOLD + ack 计数不涨（0x102 只在 STATIC 计数）
retag $M_HB 0x245A1401                       # 与上一帧**同 byte3**(0x24)：重复帧
fire $M_HB 0x100 60
snap S6                                      # 期望仍 HOLD（去重丢弃，且不刷新判活）+ hb_dup 递增
retag $M_HB 0x255A1401                       # 新 byte3(0x25) → HOLD **唯一**恢复条件
fire $M_HB 0x100 120
snap S7                                      # 期望回 CRUISE + PWM=300（且 req 较 S5 递增）
cmd "sleep 500"                              # ★ 仍在静默环回里停流 500ms → 0x100 静默 → 本地刹停
snap S8                                      # 期望 HOLD(2) + PWM=0（环回里无外部 0x100 → 与对端无关）
lb_exit
cmd "mww $GPIOB_BSRR $STBY_HIGH"; cmd "sleep 300"
snap S9                                      # 期望 BTR=Normal + STBY 归还（此后总线交还 RDK）

echo
echo "======== ② 停车语义：0x100 cmd=00 → PWM=0 但仍在 CRUISE（停车 ≠ HOLD）========"
boot_iso; cmd "sleep 1200"                   # 同 ①：停在首帧 0x101 处隔离（RDK 在场也进不来）
snap B0
stage $M_102 8 0x310155 0x0                  # 0x102 应答 seq=0x31
stage $M_HB  8 0x415A1401 0x0                # 0x100 cmd=01 cnt=0x41
stage $M_HB2 8 0x425A1400 0x0                # 0x100 cmd=00 cnt=0x42（停车）
fire $M_102 0x102 20
retag $M_102 0x320155                        # seq 递增 0x32
fire $M_102 0x102 60
fire $M_HB  0x100 120
snap B1                                      # 期望 CRUISE + PWM=300
fire $M_HB2 0x100 100                        # cmd=00 停车
snap B2                                      # 期望 st 仍=1、PWM=0、cur=0、motor=STOP(0)（不是 brake）
retag $M_HB 0x435A1401                       # cmd=01 cnt=0x43
fire $M_HB 0x100 120
snap B3                                      # 期望立刻回到 PWM=300（滑行→前进）
cmd "sleep 500"                              # ★ 仍在静默环回里停流 500ms → 0x100 静默 → HOLD
snap B4                                      # 期望 st=2 HOLD、PWM=0（环回里 → 与对端无关）
lb_exit
cmd "mww $GPIOB_BSRR $STBY_HIGH"; cmd "sleep 400"
snap B5                                      # 期望 STBY=1、BTR=Normal（总线交还 RDK，不断言 st）

# ---------------------------------------------------------------- ③ LOCK（--lock 可选）
if [ "$LOCK_RUN" = "1" ]; then
    echo
    echo "======== ③ LOCK：断总线 61s → LOCK（0x101 停发 + LED0 常亮）+ 复位恢复 ========"
    # 复位后立刻停在第 1 帧 0x101 处进环回：既断开 RX（0x100/0x102 都进不来）又不在总线上。
    # 入口可能是 STATIC（还没握手）也可能是 CRUISE（复位后那几百 ms 已握手）——两条路都会到
    # LOCK：STATIC 静默 60s 直接 LOCK；CRUISE 先 hb_timeout_ms 超时进 HOLD，再由 HOLD 静默 60s → LOCK。
    boot_iso
    cmd "sleep 1200"
    snap L0                                  # 入口态（读出来记一笔：0=STATIC / 1=CRUISE）
    cmd "sleep 61000"                        # 61s 目标时间（millis 只在目标运行时前进）
    snap L1                                  # 期望 st=3 LOCK + PWM=0 + motor=BRAKE + LED0 亮(PF9=0)
    stage $M_102 8 0x710155 0x0              # LOCK 里再喂 0x102 + 0x100：必须毫无反应
    stage $M_HB  8 0x815A1401 0x0
    fire $M_102 0x102 40
    fire $M_HB  0x100 200
    cmd "sleep 5000"                         # 5s 后再看：req_count 必须与 L1 完全相同（0x101 已停发）
    snap L2
    boot_reset; cmd "sleep 1200"             # 复位恢复（这一段不隔离，让板端重新握手）
    snap L3                                  # 复位恢复：st 回 0/1（不再是 3）+ req 重新计数
fi
cmd "mww $GPIOB_BSRR $STBY_HIGH"
cmd shutdown

echo "[info] 命令数 = ${#ARGS[@]}；快照 ${#SNAPS[@]} 个：${SNAPS[*]}"
# 本机 ST-Link 走 usbip：会话建立本身偶发失败（open 失败/空读，实测约五成）。
# 整个会话幂等（每段开头 boot_reset），失败就整段重跑；三次都读不到才判环境问题。
LOG=/tmp/rdk_hb_verify.log
for try in 1 2 3; do
    "${OCD[@]}" "${ARGS[@]}" > "$LOG" 2>&1
    grep -q '^0x' "$LOG" && break
    # 镜像不匹配：openocd 会中止会话（必然读不到值），重试无意义 → 立即交给下面的 exit 5
    grep -q -e 'checksum mismatch' -e '^diff ' "$LOG" && break
    echo "  ⚠ 第 $try 次 openocd 会话未取到读回值（ST-Link 重枚举/被占用），3s 后整段重跑"
    sleep 3
done
echo "[info] 会话输出 $(wc -l < "$LOG") 行 / 读回值 $(grep -c '^0x' "$LOG") 行"
if grep -q -e 'checksum mismatch' -e '^diff ' "$LOG"; then
    echo "❌ 板上 flash 与 $ELF **不一致** —— 本脚本用 elf 的符号地址读 RAM，镜像不匹配时会读到别的"
    echo "   变量并伪造成一堆 FAIL。先烧写本版再重跑："
    echo "     make flash        # 或 openocd 'program build/car.elf verify reset exit'"
    grep -m3 -e '^diff ' "$LOG" | sed 's/^/     /'
    exit 5
fi
# 预检（只读 verify_image 逐字节比对）：板上 flash == 本版 elf → 符号地址才可信，结论才有效
VB=$(grep -m1 -o 'verified [0-9]* bytes' "$LOG")
if [ -n "$VB" ]; then echo "✅ 预检：板上 flash == $ELF（openocd: $VB）"
else echo "  ⚠ 预检未见 verified 回显（openocd 版本差异）：后续读回若自洽即视为通过"; fi
if ! grep -q '^0x' "$LOG"; then
    echo "❌ 三次都没读到内存 —— 不是固件问题，是 SWD 链路问题。排查："
    echo "   * /dev/bus/usb 是否 root（本脚本需 sudo）；Windows 侧 usbipd attach ST-Link"
    echo "   * 是否别的进程/会话占着 ST-Link（上一个 openocd 没退干净 → 等几秒重来）"
    echo "   * openocd 原始输出尾部："
    tail -12 "$LOG" | sed 's/^/     /'
    exit 4
fi

# ---------------------------------------------------------------- 解析读回值
awk 'function h2d(h,  n,i,c,v){n=0; for(i=1;i<=length(h);i++){c=toupper(substr(h,i,1)); v=index("0123456789ABCDEF",c)-1; if(v>=0) n=n*16+v} return n}
     /^SNAP_/ { name=substr($1,6); next }
     /^0x[0-9a-fA-F]+:/ { printf "%s %d\n", name, h2d($2) }' "$LOG" > /tmp/rdk_hb_vals.txt
awk '{ if ($1!=prev) { if (NR>1) printf "\n"; printf "%s", $1; prev=$1 } printf " %s", $2 } END { printf "\n" }' \
    /tmp/rdk_hb_vals.txt > /tmp/rdk_hb_snap.txt
val(){ awk -v s="$1" -v i="$2" '$1==s{print $(i+1); exit}' /tmp/rdk_hb_snap.txt; }
# 字段位置 ↔ dump() 里的读取顺序（一一对应，改一处必须同步另一处）
ST(){ val "$1" 1; };  RQ(){ val "$1" 2; };  AK(){ val "$1" 3; };  PW(){ val "$1" 4; }
CS(){ val "$1" 5; };  TK(){ val "$1" 8; };  HD(){ val "$1" 9; };  HX(){ val "$1" 10; }
LN(){ val "$1" 11; }; MO(){ val "$1" 12; }; CCR(){ val "$1" 13; }
ODR(){ val "$1" 14; }; BTR(){ val "$1" 15; }
AGE100(){ echo $(( $(TK "$1") - $(val "$1" 6) )); }   # 距最近一帧有效 0x100
AGE102(){ echo $(( $(TK "$1") - $(val "$1" 7) )); }   # 距最近一帧有效 0x102（只作日志）
BD3(){ local v; v=$(ODR "$1"); echo $(( (${v:-0} >> 3) & 1 )); }   # STBY(PB3)：0=拉低(高阻)
LED0(){ local v; v=$(val "$1" 17); echo $(( (${v:-2} >> 9) & 1 )); }  # LED0(PF9)：0=亮（低电平点亮）/1=灭
INM(){ local v; v=$(ODR "$1"); echo $(( ${v:-0} & 0x33 )); }       # PB0/1/4/5（IN1/IN2）掩码

PASS=0; FAIL=0
OK(){  printf '  \033[32m✅\033[0m %s\n' "$*"; PASS=$((PASS+1)); }
BAD(){ printf '  \033[31m❌\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
chk(){ local l=$1 g=$2 w=$3; [ "$g" = "$w" ] && OK "$l = $g" || BAD "$l = ${g:-<空>}（期望 $w）"; }
chkge(){ local l=$1 g=$2 w=$3; [ "${g:-0}" -ge "$w" ] 2>/dev/null && OK "$l = $g (≥$w)" || BAD "$l = ${g:-<空>}（期望 ≥$w）"; }
chklt(){ local l=$1 g=$2 w=$3; [ "${g:-999999}" -lt "$w" ] 2>/dev/null && OK "$l = $g (<$w)" || BAD "$l = ${g:-<空>}（期望 <$w）"; }
chkrng(){ local l=$1 g=$2 lo=$3 hi=$4; { [ "${g:-x}" -ge "$lo" ] && [ "$g" -le "$hi" ]; } 2>/dev/null \
    && OK "$l = $g ($lo~$hi)" || BAD "$l = ${g:-<空>}（期望 $lo~$hi）"; }
sec(){ printf '\n\033[36m── %s\033[0m\n' "$*"; }

# 先把"每个快照是否读全 16 个字段"筛一遍：halt 失败/空读会让后续断言全是空值，必须先报出来
sec "快照完整性（每格 17 个字段）"
for s in "${SNAPS[@]}"; do [ -n "$(val "$s" 17)" ] && OK "快照 $s 字段齐全" || BAD "快照 $s 字段不全（SWD halt/读回失败 → 后面断言不可信）"; done

# ================================================================ ①
sec "S0 复位初态（已隔离：STATIC / PWM=0 / 0x101 在发 / STBY 已拉低）"
chk "st"  "$(ST S0)" "$ST_STATIC"
chk "pwm" "$(PW S0)" "0"
chk "cur" "$(CS S0)" "0"
chk "CCR1" "$(CCR S0)" "0"
chk "motor（上电=PWM0+短路制动：rdk_config.stop_short_brake=TRUE）" "$(MO S0)" "$MOTOR_BRAKE"
chk "IN1/IN2（PB0/1/4/5 全 H = 短路制动）" "$(INM S0)" "51"
chk "ack_count（0x102 一帧都没生效 → 没被带进 CRUISE）" "$(AK S0)" "0"
chk "hb_dup" "$(HD S0)" "0"
# hb_rx：隔离动作落在 Can_Init 之后、首帧 0x101 之前，这几 ms 里 RDK 仍可能发来它的周期 0x100
# （板端本来就每 50ms 发一帧）—— 那是测试装置的产物，不是固件行为。关键看 st 仍是 0：STATIC 忽略 0x100。
hx0=$(HX S0); hx0=${hx0:-0}
if [ "$hx0" = "0" ]; then OK "hb_rx = 0（隔离前也没收到 0x100）"
else OK "hb_rx = $hx0（隔离前 RDK 送来过 0x100，而 st 仍 = 0 → \"STATIC 忽略 0x100\"直接得证）"; fi
chkrng "req_count（0x101 已周期发出）" "$(RQ S0)" 1 30
chk "STBY PB3（=0 高阻，车轮不可能转）" "$(BD3 S0)" "0"

sec "S1 静默环回已生效"
chk "CAN_BTR" "$(BTR S1)" "$((BTR_LOOP))"

sec "S2 2×0x102 应答（seq 递增）+ 0x100 cmd=01 → CRUISE 前进"
chk "st" "$(ST S2)" "$ST_CRUISE"
chk "ack_count（2 帧应答）" "$(AK S2)" "2"
chk "pwm（cruise 300）" "$(PW S2)" "$CRUISE_PWM"
chk "cur（斜坡已到 300）" "$(CS S2)" "$CRUISE_PWM"
chk "CCR1（300×50/1000=15，真写进 TIM3）" "$(CCR S2)" "15"
chk "motor" "$(MO S2)" "$MOTOR_FORWARD"
chk "IN1/IN2（PB0/1/4/5 = 1,0,1,0 前进）" "$(INM S2)" "17"
chkge "hb_rx（收到 1 帧有效 0x100）" "$(HX S2)" 1
chklt "age(0x100)（判活新鲜）" "$(AGE100 S2)" "$HB_TIMEOUT"

sec "S3 ★ 0x100 cmd=03 → 刹停，但状态仍是 CRUISE（无 STOPPING/命令型锁死）"
chk "st" "$(ST S3)" "$ST_CRUISE"
chk "pwm" "$(PW S3)" "0"
chk "cur" "$(CS S3)" "0"
chk "CCR1" "$(CCR S3)" "0"
chk "motor（短路制动 3，不是滑行 0）" "$(MO S3)" "$MOTOR_BRAKE"
chk "IN1/IN2（PB0/1/4/5 全 H = 短路制动）" "$(INM S3)" "51"
chklt "age(0x100)" "$(AGE100 S3)" "$HB_TIMEOUT"

sec "S4 ★★ 只喂 0x102、0x100 静默 ≥${HB_TIMEOUT}ms → HOLD（判活只看 0x100）"
chk "st" "$(ST S4)" "$ST_HOLD"
chk "pwm" "$(PW S4)" "0"
chk "motor" "$(MO S4)" "$MOTOR_BRAKE"
chkge "age(0x100)（0x100 已静默）" "$(AGE100 S4)" "$HB_TIMEOUT"
chklt "age(0x102)（0x102 明明在收，却仍进 HOLD）" "$(AGE102 S4)" "$HB_TIMEOUT"

sec "S5 HOLD 里发 0x102（应答 + 心跳）→ 不能恢复、不计数"
chk "st" "$(ST S5)" "$ST_HOLD"
chk "pwm" "$(PW S5)" "0"
chk "ack_count（仍 2，0x102 只在 STATIC 计数）" "$(AK S5)" "$(AK S2)"
chkge "0x101 仍周期发出（req 递增）" "$(RQ S5)" "$(RQ S2)"

sec "S6 ★ 同 byte3 重发 0x100 → 去重丢弃，仍 HOLD"
chk "st" "$(ST S6)" "$ST_HOLD"
chk "pwm" "$(PW S6)" "0"
chk "hb_dup（去重计数 +1）" "$(HD S6)" "$(( $(HD S5) + 1 ))"
chk "hb_rx（重复帧不计有效帧）" "$(HX S6)" "$(HX S5)"

sec "S7 ★ 新 byte3 的 0x100 cmd=01 → HOLD 唯一恢复条件成立（回 CRUISE）"
chk "st" "$(ST S7)" "$ST_CRUISE"
chk "pwm" "$(PW S7)" "$CRUISE_PWM"
chk "cur" "$(CS S7)" "$CRUISE_PWM"
chk "last_cnt（记录新 byte3）" "$(LN S7)" "37"          # 0x25
chkge "req_count（HOLD 期间 0x101 从未停发）" "$(RQ S7)" "$(RQ S5)"

sec "S8 ★ 静默环回里停流 500ms → 0x100 静默 → 本地刹停（HOLD，与对端无关）"
chk "CAN_BTR（仍在环回）" "$(BTR S8)" "$((BTR_LOOP))"
chk "st（0x100 静默 >${HB_TIMEOUT}ms）" "$(ST S8)" "$ST_HOLD"
chk "pwm" "$(PW S8)" "0"
chk "motor" "$(MO S8)" "$MOTOR_BRAKE"
chkge "age(0x100)（已静默）" "$(AGE100 S8)" "$HB_TIMEOUT"

sec "S9 退出环回 + 归还 STBY（此后总线交还 RDK，故不断言 st/pwm）"
chk "CAN_BTR（已回 Normal）" "$(BTR S9)" "$((BTR_NORMAL))"
chk "STBY PB3（=1 已归还）" "$(BD3 S9)" "1"

# ================================================================ ②
sec "B0 复位初态"
chk "st" "$(ST B0)" "$ST_STATIC"
chk "pwm" "$(PW B0)" "0"
chk "motor（上电=PWM0+短路制动，与 S0 同源）" "$(MO B0)" "$MOTOR_BRAKE"
chk "IN1/IN2（PB0/1/4/5 全 H = 短路制动）" "$(INM B0)" "51"
chk "STBY PB3" "$(BD3 B0)" "0"

sec "B1 握手后 cmd=01 → CRUISE 前进"
chk "st" "$(ST B1)" "$ST_CRUISE"
chk "pwm" "$(PW B1)" "$CRUISE_PWM"
chk "cur" "$(CS B1)" "$CRUISE_PWM"
chk "motor" "$(MO B1)" "$MOTOR_FORWARD"

sec "B2 ★ cmd=00 停车 → PWM=0，但状态仍 CRUISE（停车 ≠ HOLD）、motor=STOP(0) 非 brake"
chk "st" "$(ST B2)" "$ST_CRUISE"
chk "pwm" "$(PW B2)" "0"
chk "cur（斜坡收到 0）" "$(CS B2)" "0"
chk "CCR1" "$(CCR B2)" "0"
chk "motor（滑行停车=0，短路制动才是 3）" "$(MO B2)" "$MOTOR_STOP"
chk "IN1/IN2（全 L = 悬空滑行）" "$(INM B2)" "0"

sec "B3 cmd=01 立刻恢复前进"
chk "st" "$(ST B3)" "$ST_CRUISE"
chk "pwm" "$(PW B3)" "$CRUISE_PWM"
chk "cur" "$(CS B3)" "$CRUISE_PWM"
chk "motor" "$(MO B3)" "$MOTOR_FORWARD"

sec "B4 ★ 静默环回里停流 500ms → HOLD（与对端无关）"
chk "st" "$(ST B4)" "$ST_HOLD"
chk "pwm" "$(PW B4)" "0"
chk "motor" "$(MO B4)" "$MOTOR_BRAKE"
chk "CAN_BTR（仍在环回）" "$(BTR B4)" "$((BTR_LOOP))"

sec "B5 回收：退出环回 + STBY 归还（总线交还 RDK）"
chk "STBY PB3" "$(BD3 B5)" "1"
chk "CAN_BTR（已回 Normal）" "$(BTR B5)" "$((BTR_NORMAL))"

# ================================================================ ③ LOCK（--lock）
if [ "$LOCK_RUN" = "1" ]; then
sec "L0 LOCK 段入口（已在环回里断总线：0x100/0x102 都进不来）"
chkrng "st（0=STATIC 未握手 / 1=CRUISE 已握手，两条路都能到 LOCK）" "$(ST L0)" "$ST_STATIC" "$ST_CRUISE"
chk "CAN_BTR（环回）" "$(BTR L0)" "$((BTR_LOOP))"

sec "L1 ★★ 断总线 61s → LOCK（0x101 停发 + LED0 常亮 + 制动保持）"
chk "st" "$(ST L1)" "$ST_LOCK"
chk "pwm" "$(PW L1)" "0"
chk "motor（制动保持）" "$(MO L1)" "$MOTOR_BRAKE"
chk "LED0（PF9=0 → 常亮）" "$(LED0 L1)" "0"

sec "L2 LOCK 里再喂 0x102 + 0x100 → 不响应、0x101 仍停发（只能复位恢复）"
hx1=$(HX L1); hx1=${hx1:-0}
chk "st（仍是 LOCK）" "$(ST L2)" "$ST_LOCK"
chk "req_count（5s 后完全不变 → 0x101 已停发）" "$(RQ L2)" "$(RQ L1)"
chk "ack_count（LOCK 里 0x102 不计数）" "$(AK L2)" "$(AK L1)"
chkge "hb_rx（LOCK 里注入的 0x100 仍被记账：\"不响应\" ≠ \"不收帧\"）" "$(HX L2)" "$((hx1 + 1))"
chk "pwm" "$(PW L2)" "0"
chk "LED0（仍常亮）" "$(LED0 L2)" "0"

sec "L3 复位恢复：LOCK → STATIC/CRUISE，0x101 重新计数"
chkrng "st（不再是 3）" "$(ST L3)" "$ST_STATIC" "$ST_CRUISE"
chkge "req_count（0x101 已恢复发送）" "$(RQ L3)" 1
chk "STBY PB3（脚本收尾前仍拉低）" "$(BD3 L3)" "0"
fi

# ================================================================ 汇总
echo
printf '结果：PASS=%d  FAIL=%d（快照 %d 格）\n' "$PASS" "$FAIL" "${#SNAPS[@]}"
if [ "$LOCK_RUN" = "1" ]; then
    echo "LOCK 段已随本次运行校验（③：0x101 停发 + LED0 常亮 + LOCK 不响应 + 复位恢复）。"
else
    echo "注意：LOCK（60s 未握手/未恢复 → 停发 0x101 + LED 常亮）要等 60s 量级，默认不跑；"
    echo "      加 --lock 跑 ③（≈+65s），无需 CANable 即可校验（见文件头「覆盖」）。"
fi
if [ "$FAIL" -eq 0 ]; then echo "✅ RDK 真机验收 PASS"; exit 0; fi
echo "❌ RDK 真机验收 FAIL（$FAIL 项）"
echo "   原始会话日志：$LOG   解析：/tmp/rdk_hb_snap.txt"
exit 1
