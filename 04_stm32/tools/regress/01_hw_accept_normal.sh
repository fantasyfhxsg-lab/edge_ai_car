#!/usr/bin/env bash
# ============================================================================
# 01_hw_accept_normal.sh —— 真机验收（Normal 镜像 + CANable/slcan，500kbps「真」验证）
#
# 完整复现 README §步骤③ 的验收链路，判定全部自动完成（不依赖人眼看串口）：
#   ① 前置：工具链 / 镜像类型（必须是 Normal：有 g_brake_*、无 g_lb_*）/ CAN 通道
#   ② 烧写（--no-flash 可跳过：验当前在跑的镜像，不复位）
#   ③ 接口自检：存在 / state UP / type can|280
#   ④ **位速率真验证**：SWD 读 RCC_CR/PLLCFGR/CFGR + CAN1_MCR/MSR/ESR/BTR，
#      从"在线寄存器"反推 HSE→PLL→SYSCLK→APB1(PCLK1)→tq→bps，必须=500000。
#      同时断言 Normal 模式位（LBKM/SILM=0）与总线健康（INAK=0/TEC=REC=BOFF=0）。
#      ⚠ 历史坑拦截：platform.c 的 PPRE1 若误配成 ÷2（APB1=84MHz），这里会直接报
#        位速率≈1Mbps —— 这正是注释/文档与实物不一致的典型事故。
#   ⑤ 0x125 遥测：抓 N 帧 → 帧数/周期(20ms)/DLC=8/byte2..7=0/小端载荷与 SWD 变量互证
#   ⑥ 0x123 上行闭环：level3 → g_brake_latched=1 + GPIOB 急刹位；level0 → 解锁滑行
#   ⑦ 链路错误计数：TEC/REC/BOFF/Ovf/Rec 全 0
#
# 用法（openocd 要 /dev/bus/usb → 脚本会自己经 wsl.exe -u root 提权，也可直接 sudo）：
#   bash tools/regress/01_hw_accept_normal.sh               # 完整验收（含烧写）
#   bash tools/regress/01_hw_accept_normal.sh --no-flash    # 不烧写
#   bash tools/regress/01_hw_accept_normal.sh --serial      # 顺带抓 3s 串口原文
#   CAN_IF=slcan0 bash ...                                  # 覆盖接口名（默认 can0）
#
# 前置条件见 tools/regress/README.md（slcand 起接口 / ST-Link attach / 板子供电）。
# ============================================================================
set -u
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
regress_init "① 真机验收（Normal + CANable，500kbps 真验证）"

FLASH=1; SERIAL=0; FRAMES=60
for a in "$@"; do
    case "$a" in
        --no-flash) FLASH=0 ;;
        --serial)   SERIAL=1 ;;
        --frames=*) FRAMES=${a#*=} ;;
        -h|--help)  sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数：$a（支持 --no-flash / --serial / --frames=N）"; exit 2 ;;
    esac
done

maybe_reexec_as_root "$@"

# ---------------------------------------------------------------- ① 前置
sec "① 前置：工具链 / 镜像类型 / CAN 通道"
need_cmd openocd "容器里先 run_toolchain.sh 进环境" || exit 2
need_cmd arm-none-eabi-nm || exit 2
need_cmd python3 || exit 2
elf_check g_brake_latched g_lb_fail "make clean && make PROTOCOL=brake_v2   # 本脚本验 AEB，需 brake_v2 Normal 镜像" || exit 2        # Normal 镜像：有 g_brake_*，无回环自测符号
can_detect || exit 2
can_if_autodetect || exit 2

# ---------------------------------------------------------------- ② 烧写
sec "② 烧写镜像（--no-flash 跳过）"
if [ "$FLASH" = 1 ]; then
    do_flash || exit 2
    info "等 1.5s：复位后 0x125 首个周期 + 应用初始化"
    sleep 1.5
else
    info "跳过烧写：验证当前正在运行的镜像（保持运行态，不复位）"
fi
if [ "$SERIAL" = 1 ]; then
    serial_start_bg "$DUMP_DIR/acc_serial.txt" || warn "--serial 忽略（串口不可用/被 XCOM 占用）"
fi

# ---------------------------------------------------------------- ③ 接口
sec "③ CAN 接口自检"
can_if_check

# ---------------------------------------------------------------- ④ 位速率真验证
sec "④ 位速率真验证：在线寄存器反推（HSE→PLL→SYSCLK→APB1→tq→bps）"

pre_of() {      # APB 预分频字段（PPREx）→ 分频比
    case "$1" in 0|1|2|3) echo 1 ;; 4) echo 2 ;; 5) echo 4 ;; 6) echo 8 ;; 7) echo 16 ;; esac
}
ahb_pre_of() {  # AHB 预分频字段（HPRE）→ 分频比
    case "$1" in
        0|1|2|3|4|5|6|7) echo 1 ;; 8) echo 2 ;; 9) echo 4 ;; 10) echo 8 ;; 11) echo 16 ;;
        12) echo 64 ;; 13) echo 128 ;; 14) echo 256 ;; 15) echo 512 ;;
    esac
}

swd_build RCC_CR RCC_PLLCFGR RCC_CFGR CAN1_MCR CAN1_MSR CAN1_ESR CAN1_BTR \
          g_recover_cnt g_rx_overrun || exit 3
swd_read_all "$DUMP_DIR/acc_regs" || exit 3

for k in RCC_CR RCC_PLLCFGR RCC_CFGR CAN1_MCR CAN1_MSR CAN1_ESR CAN1_BTR; do
    [ "$(val $k)" = "NA" ] && { bad "寄存器 $k 没读回来（SWD/elf 不匹配）"; exit 3; }
done

cr=$(val RCC_CR); pll=$(val RCC_PLLCFGR); cfgr=$(val RCC_CFGR)
mcr=$(val CAN1_MCR); msr=$(val CAN1_MSR); esr=$(val CAN1_ESR); btr=$(val CAN1_BTR)
info "RCC_CR=$(hexv RCC_CR) PLLCFGR=$(hexv RCC_PLLCFGR) CFGR=$(hexv RCC_CFGR)"
info "CAN1: MCR=$(hexv CAN1_MCR) MSR=$(hexv CAN1_MSR) ESR=$(hexv CAN1_ESR) BTR=$(hexv CAN1_BTR)"
HSE=8000000                                        # 外部 8MHz 晶振（.clinerules §1）
hserdy=$(( (cr >> 17) & 1 )); pllrdy=$(( (cr >> 25) & 1 ))
pllsrc=$(( (pll >> 22) & 1 ))                      # 1 = HSE 作 PLL 源
pllm=$(( pll & 0x3F )); plln=$(( (pll >> 6) & 0x1FF ))
case $(( (pll >> 16) & 0x3 )) in 0) pllp=2 ;; 1) pllp=4 ;; 2) pllp=6 ;; 3) pllp=8 ;; esac
sws=$(( (cfgr >> 2) & 0x3 ))                       # 2 = PLL 作 SYSCLK
hpre=$(( (cfgr >> 4) & 0xF )); ppre1=$(( (cfgr >> 10) & 0x7 )); ppre2=$(( (cfgr >> 13) & 0x7 ))

vco_in=0
[ "$pllm" -gt 0 ] && vco_in=$(( HSE / pllm ))      # F4 的 PLLM 字段 = 直接分频值（RM0090）
sysclk=$(( vco_in * plln / pllp ))
ahb_div=$(ahb_pre_of "$hpre"); apb1_div=$(pre_of "$ppre1"); apb2_div=$(pre_of "$ppre2")
hclk=$(( sysclk / ahb_div )); pclk1=$(( hclk / apb1_div )); pclk2=$(( hclk / apb2_div ))
info "HSE=$HSE PLLM=$pllm → VCOin=$vco_in Hz；PLLN=$plln，PLLP=÷$pllp → SYSCLK=$sysclk Hz"
info "AHB÷$ahb_div → HCLK=$hclk；APB1÷$apb1_div → PCLK1=$pclk1；APB2÷$apb2_div → PCLK2=$pclk2"

brp=$(( btr & 0x3FF )); ts1=$(( (btr >> 16) & 0xF )); ts2=$(( (btr >> 20) & 0x7 ))
sjw=$(( (btr >> 24) & 0x3 )); lbkm=$(( (btr >> 30) & 1 )); silm=$(( (btr >> 31) & 1 ))
ntq=$(( 3 + ts1 + ts2 ))                           # 1(Sync)+(TS1+1)+(TS2+1) 个 tq
bps_x1000=$(( pclk1 * 1000 / ( (brp + 1) * ntq ) ))
info "BTR：BRP=$brp(÷$((brp + 1))) TS1=$((ts1 + 1))tq TS2=$((ts2 + 1))tq SJW=$((sjw + 1))tq → 位时间 $ntq tq"
info "tq ≈ $(( (brp + 1) * 1000 / (pclk1 / 1000000) )) ns；反推位速率 = $((bps_x1000 / 1000)) bps（x1000=$bps_x1000）"

assert_eq "HSERDY(外部晶振就绪)" "$hserdy" 1
assert_eq "PLL 源 PLLSRC(1=HSE)" "$pllsrc" 1
assert_eq "PLLRDY" "$pllrdy" 1
assert_eq "SYSCLK 源 SWS(2=PLL)" "$sws" 2
assert_eq "SYSCLK(Hz)" "$sysclk" 168000000
assert_eq "APB1 分频字段 PPRE1(5=÷4)" "$ppre1" 5
assert_eq "PCLK1(Hz)" "$pclk1" 42000000
assert_eq "BRP 字段" "$brp" 3
assert_eq "TS1 字段" "$ts1" 14
assert_eq "TS2 字段" "$ts2" 4
assert_eq "SJW 字段" "$sjw" 0
assert_eq "反推位速率(bps)" "$((bps_x1000 / 1000))" 500000
[ $((bps_x1000 % 1000)) -eq 0 ] || warn "位速率不是整数（x1000=$bps_x1000），复核 PCLK1/BTR 组合"
assert_eq "BTR.LBKM(0=Normal 模式)" "$lbkm" 0
assert_eq "BTR.SILM(0=Normal 模式)" "$silm" 0
assert_eq "MCR.INRQ(0=已退出初始化)" "$(( mcr & 1 ))" 0
assert_eq "MSR.INAK(0=已在总线上)" "$(( msr & 1 ))" 0
assert_eq "ESR.TEC(发送错误计数)" "$(( (esr >> 16) & 0xFF ))" 0
assert_eq "ESR.REC(接收错误计数)" "$(( (esr >> 24) & 0xFF ))" 0
assert_eq "ESR.BOFF(总线关闭)" "$(( (esr >> 2) & 1 ))" 0

# ---------------------------------------------------------------- ⑤ 0x125 遥测
sec "⑤ 0x125 车速遥测（周期必须 < 板端 recv_timeout = 100ms）"
info "抓 $FRAMES 帧（@50fps ≈ $((FRAMES / 50))s）"
can_capture "$FRAMES" 8 "$DUMP_DIR/acc_125.txt"
S125="$DUMP_DIR/acc_125.stats"
can_stats_125 "$DUMP_DIR/acc_125.txt" >"$S125"
sed 's/^/     /' "$S125"
assert_range "0x125 帧数" "$(stats_get "$S125" count)" "$((FRAMES * 9 / 10))" 999999
assert_eq "DLC≠8 的 0x125 帧数" "$(stats_get "$S125" dlc_bad)" 0
assert_eq "byte2..7 非 0 的帧数" "$(stats_get "$S125" rsvd_bad)" 0
assert_range "平均周期(ms)" "$(round0 "$(stats_get "$S125" mean_ms)")" 15 25
mmax=$(round0 "$(stats_get "$S125" max_ms)")
if [ "$mmax" = "NA" ] || [ "$mmax" -lt 0 ]; then
    bad "最大周期取不到（本次没抓到 0x125，周期契约无从评估）"
elif [ "$mmax" -lt 100 ]; then
    ok "最大周期 $(stats_get "$S125" max_ms) ms < 100ms（板端 recv_timeout 契约）"
else
    bad "最大周期 $(stats_get "$S125" max_ms) ms ≥ 100ms → 板端可能判超时"
fi
[ "$(stats_get "$S125" other)" = "0" ] || \
    info "总线上另有 $(stats_get "$S125" other) 帧其它 ID（本地自发帧回显或板端其它节点）"

# 载荷与 STM32 侧变量互证：有效则 = km/h 小端，无效则 = 0xFFFF
# 用 **最后一帧** 比：它与随后的 SWD 读回时间最近（车速若在变，首帧必然对不上，会假报失败）
swd_build g_speed_kmh g_speed_valid || exit 3
swd_read_all "$DUMP_DIR/acc_speed" || exit 3
kmh=$(val g_speed_kmh); sv=$(val g_speed_valid)
if [ "$sv" = "1" ]; then
    exp=$(printf '%02X %02X' $((kmh & 0xFF)) $(( (kmh >> 8) & 0xFF )))
else
    exp="FF FF"
fi
assert_eq "0x125 末帧 byte0 byte1（小端，与 g_speed_kmh 互证）" "$(stats_get "$S125" last_val)" "$exp"
info "SWD 侧：g_speed_kmh=$kmh g_speed_valid=$sv（编码器 cm/s × 0.036）"
info "抓到的载荷多样性：$(stats_get "$S125" vals)"

# ---------------------------------------------------------------- ⑥ 0x123 上行闭环
sec "⑥ 0x123 制动上行闭环（含 GPIOB 硬件佐证）"
swd_build g_brake_level g_brake_latched g_brake_rx_seen GPIOB_ODR || exit 3
info "发 123#03…（level=3 紧急制动）"
can_send "123#0300000000000000" >/dev/null
sleep 0.4
swd_read_all "$DUMP_DIR/acc_brake" || exit 3
assert_eq "g_brake_level" "$(val g_brake_level)" 3
assert_eq "g_brake_latched" "$(val g_brake_latched)" 1
assert_eq "g_brake_rx_seen" "$(val g_brake_rx_seen)" 1
assert_eq "GPIOB.ODR&0x3B（0x3B=AIN1/AIN2/BIN1/BIN2/STBY 全高＝短路急刹）" "$(maskv GPIOB_ODR 0x3B)" 59
info "原始 GPIOB_ODR = $(hexv GPIOB_ODR)"

info "发 123#00…（level=0 解除制动）"
can_send "123#0000000000000000" >/dev/null
sleep 0.4
swd_read_all "$DUMP_DIR/acc_release" || exit 3
assert_eq "g_brake_level" "$(val g_brake_level)" 0
assert_eq "g_brake_latched" "$(val g_brake_latched)" 0
assert_eq "GPIOB.ODR&0x3B（0x08=仅 STBY 高＝滑行）" "$(maskv GPIOB_ODR 0x3B)" 8
info "原始 GPIOB_ODR = $(hexv GPIOB_ODR)"

# ---------------------------------------------------------------- ⑦ 错误计数
sec "⑦ 链路错误计数（全 0 = 位定时/终端电阻/共地/滤波器都合格）"
swd_build CAN1_ESR CAN1_MSR CAN1_BTR g_rx_overrun g_recover_cnt || exit 3
swd_read_all "$DUMP_DIR/acc_err" || exit 3
esr=$(val CAN1_ESR); msr=$(val CAN1_MSR)
assert_eq "ESR.TEC" "$(( (esr >> 16) & 0xFF ))" 0
assert_eq "ESR.REC" "$(( (esr >> 24) & 0xFF ))" 0
assert_eq "ESR.BOFF" "$(( (esr >> 2) & 1 ))" 0
assert_eq "MSR.INAK(0=在总线上)" "$(( msr & 1 ))" 0
assert_eq "FIFO0 溢出累计 g_rx_overrun" "$(val g_rx_overrun)" 0
info "自愈累计 g_recover_cnt=$(val g_recover_cnt)（0 = 从未卡在 init 态）"

# ---------------------------------------------------------------- ⑧ 存活复查
sec "⑧ 收尾：0x125 仍在发（整轮刺激后链路存活）"
can_capture 3 3 "$DUMP_DIR/acc_alive.txt"
assert_range "0x125 存活帧数" "$(grep -c ' 125 ' "$DUMP_DIR/acc_alive.txt")" 1 99

if [ "$SERIAL" = 1 ]; then
    sec "⑨ 串口原文（整场后台抓取，--serial）"
    serial_stop_bg
    if [ -s "$DUMP_DIR/acc_serial.txt" ]; then
        grep -aoE '\[(AEB|AEB-TEST|CAN-LB|CAN|ENC)\][^[:cntrl:]]*' "$DUMP_DIR/acc_serial.txt" | head -25 | sed 's/^/     /'
    fi
fi

finish
