#!/usr/bin/env bash
# ============================================================================
# 03_loopback_selftest.sh —— 回环自测（静默回环镜像，BTR.LBKM|SILM=1）
#
# 复现 README §步骤① 的"分层验证第一步"：**不需要 TJA1050、不需要对端、不需要 120Ω**，
# 用 CAN 外设自带的静默回环（TX 内部接回 RX）自发自收，断言所有链路：
#   ① 构建 make PROTOCOL=brake_v2 CAN_LOOPBACK=1（-DCAN_PROTOCOL_BRAKE_V2=1 -DCAN_LOOPBACK_SELFTEST=1）
#      ⚠ 回环 AEB 自测（0x123/0x124/0x125 的 g_lb_*）属 brake_v2 分支；heartbeat_v1 分支无这些符号
#   ② 镜像自检：含回环符号（g_lb_*）——防止把 Normal 镜像当回环镜像测
#   ③ 烧写 + 复位
#   ④ t+1.5s：BTR.LBKM=SILM=1（真·静默回环，时序字段仍 = 0x004E0003 即 500kbps），
#            0x111 回读脏帧计数 echo_bad=0
#   ⑤ t+8.5s：至少跑完一轮 10 步 AEB 自测 → g_lb_round≥1、g_lb_last_round_fail=0、
#            g_lb_fail=0、g_lb_speed_echo=123（0x125 小端回读）、speed_echo_ok=1、
#            rx/tx 自洽（0x111 每步一帧，256 回绕）
#   ⑥ 收尾：提示（或 --restore 自动）把板子还原成 Normal 镜像
#
# 用法：
#   bash tools/regress/03_loopback_selftest.sh            # 构建+烧写+断言
#   bash tools/regress/03_loopback_selftest.sh --no-build # 复用现有回环 elf
#   bash tools/regress/03_loopback_selftest.sh --serial   # 整场抓串口（[CAN-LB] PASS 行）
#   bash tools/regress/03_loopback_selftest.sh --restore  # 结束后自动重建+烧写 Normal
#
# ⚠ 回环镜像不会真的收发总线（SILM=1 不驱动 CANTX），**跑完记得还原 Normal**，
#   否则后续与板端/CANable 联调会"什么都收不到"。
# ============================================================================
set -u
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
regress_init "③ 回环自测（静默回环镜像 CAN_LOOPBACK=1）"

BUILD=1; FLASH=1; SERIAL=0; RESTORE=0
for a in "$@"; do
    case "$a" in
        --no-build)  BUILD=0 ;;
        --no-flash)  FLASH=0 ;;
        --serial)    SERIAL=1 ;;
        --restore)   RESTORE=1 ;;
        -h|--help)   sed -n '2,27p' "$0"; exit 0 ;;
        *) echo "未知参数：$a（支持 --no-build / --no-flash / --serial / --restore）"; exit 2 ;;
    esac
done

maybe_reexec_as_root "$@"

# ---------------------------------------------------------------- ① 前置
sec "① 前置：工具链"
need_cmd openocd "容器里先 run_toolchain.sh 进环境" || exit 2
need_cmd arm-none-eabi-nm || exit 2
need_cmd python3 || exit 2
need_cmd make || exit 2

# ---------------------------------------------------------------- ② 构建
sec "② 构建回环自测镜像（make PROTOCOL=brake_v2 CAN_LOOPBACK=1）"
if [ "$BUILD" = 1 ]; then
    do_build PROTOCOL=brake_v2 CAN_LOOPBACK=1 || exit 2
else
    info "跳过构建：复用现有 $ELF（必须是 PROTOCOL=brake_v2 CAN_LOOPBACK=1 构建的）"
fi
elf_check g_lb_fail "" "make clean && make PROTOCOL=brake_v2 CAN_LOOPBACK=1" || exit 2      # 必须含回环自测符号
elf_check g_brake_latched "" "make clean && make PROTOCOL=brake_v2 CAN_LOOPBACK=1" || exit 2

# ---------------------------------------------------------------- ③ 烧写
sec "③ 烧写 + 复位（自测从已知初态开始）"
if [ "$FLASH" = 1 ]; then
    do_flash || exit 2
else
    info "跳过烧写：直接读当前运行态"
fi
if [ "$SERIAL" = 1 ]; then
    serial_start_bg "$DUMP_DIR/lb_serial.txt" || warn "--serial 忽略（串口不可用/被 XCOM 占用）"
fi

swd_build CAN1_BTR CAN1_MCR CAN1_MSR CAN1_ESR \
          g_can_test_tx_counter g_can_test_tx_last g_can_test_rx_count \
          g_can_test_echo_bad g_can_test_pass g_ticks \
          g_lb_round g_lb_step g_lb_fail g_lb_last_round_fail g_lb_fail_step \
          g_lb_fail_exp g_lb_fail_act g_lb_speed_echo g_lb_speed_echo_ok || exit 3

# ---------------------------------------------------------------- ④ 开机态
sec "④ t+1.5s：回环模式位 + 0x111 回读链路"
sleep 1.5
swd_read_all "$DUMP_DIR/lb_t1" || exit 3
check_nums CAN1_BTR CAN1_MSR g_can_test_echo_bad g_lb_fail || exit 3
btr=$(val CAN1_BTR); msr=$(val CAN1_MSR)
lbkm=$(( (btr >> 30) & 1 )); silm=$(( (btr >> 31) & 1 ))
info "BTR=$(hexv CAN1_BTR)（期望 0xC04E0003 = 0x004E0003 | LBKM | SILM）"
info "MSR=$(hexv CAN1_MSR) INAK=$(( msr & 1 ))（0 = 已退出初始化）"
assert_eq "BTR.LBKM(1=内部回环)" "$lbkm" 1
assert_eq "BTR.SILM(1=静默，不驱动 CANTX)" "$silm" 1
assert_eq "BTR 时序字段（掩掉 LBKM|SILM 后仍为 0x004E0003=500kbps）" "$(( btr & 0x3FFFFFFF ))" "$(( 0x004E0003 ))"
assert_eq "MSR.INAK(0=在总线上)" "$(( msr & 1 ))" 0
assert_eq "0x111 回读脏帧计数 g_can_test_echo_bad" "$(val g_can_test_echo_bad)" 0
assert_eq "累计失败步 g_lb_fail" "$(val g_lb_fail)" 0
info "开机态：tx_counter=$(val g_can_test_tx_counter) rx_count=$(val g_can_test_rx_count) round=$(val g_lb_round) step=$(val g_lb_step) last_round_fail=$(val g_lb_last_round_fail)（0xFF=一轮还没跑完，正常）"

# ---------------------------------------------------------------- ⑤ 跑完一轮
sec "⑤ t+8.5s：至少跑完一轮 10 步 AEB 自测（10 步×200ms + 轮间 5s）"
info "等待 7s..."
sleep 7
swd_read_all "$DUMP_DIR/lb_t2" || exit 3
check_nums g_lb_round g_lb_fail g_lb_last_round_fail g_can_test_rx_count g_can_test_tx_counter \
           g_can_test_echo_bad g_lb_speed_echo g_lb_speed_echo_ok || exit 3
assert_range "已完成轮数 g_lb_round（≥1 = 跑完整轮）" "$(val g_lb_round)" 1 999999
assert_eq "上一轮失败步数 g_lb_last_round_fail（0=全过）" "$(val g_lb_last_round_fail)" 0
assert_eq "累计失败步数 g_lb_fail" "$(val g_lb_fail)" 0
assert_eq "0x111 脏帧计数 g_can_test_echo_bad" "$(val g_can_test_echo_bad)" 0

# g_lb_speed_echo 是"最近一次回读到的 0x125 载荷"，随自测步滚动（步 9=123、步 10=0xFFFF），
# 固定时刻读到的可能已滚到步 10 —— 两个都是合法回读；"步 9 必须回读 123"由固件自身的
# 步断言保证（g_lb_round≥1 且 g_lb_last_round_fail=0，已覆盖整轮 10 步含步 9）。
echo_val=$(val g_lb_speed_echo)
case "$echo_val" in
    123)   ok "0x125 小端回读值 g_lb_speed_echo = 123（自测步 9 载荷，小端还原正确）" ;;
    65535) ok "0x125 小端回读值 g_lb_speed_echo = 0xFFFF（自测步 10 占位载荷，合法；步 9 的 123 由 g_lb_last_round_fail=0 覆盖）" ;;
    *)     bad "0x125 小端回读值 g_lb_speed_echo = $echo_val，既不是 123 也不是 0xFFFF(65535)" ;;
esac
assert_eq "0x125 回读 DLC=8 标志 g_lb_speed_echo_ok" "$(val g_lb_speed_echo_ok)" 1
rx=$(val g_can_test_rx_count); tx=$(val g_can_test_tx_counter)
d=$(( (rx - tx) % 256 ))
case "$d" in
    0|255) ok "0x111 收发自洽：rx=$rx 与 tx_counter=$tx 相符（差取模 256 = $d）" ;;
    *)     bad "0x111 收发自洽失败：rx=$rx tx_counter=$tx（差取模 256 = $d）" ;;
esac
assert_range "回读帧数 g_can_test_rx_count（1 帧/秒，≥6 = 链路持续通）" "$rx" 6 999999
info "轮次信息：round=$(val g_lb_round) step=$(val g_lb_step) ticks=$(val g_ticks)"
info "失败详情（正常应为 fail_step=0 exp=-1 act=-1）：fail_step=$(val g_lb_fail_step) exp=$(sval g_lb_fail_exp) act=$(sval g_lb_fail_act)"

if [ "$SERIAL" = 1 ]; then
    sec "⑥ 串口原文（[CAN-LB] PASS/FAIL 行）"
    serial_stop_bg
    if [ -s "$DUMP_DIR/lb_serial.txt" ]; then
        grep -aoE '\[(CAN-LB|AEB-TEST|ENC|CAN)\][^[:cntrl:]]*' "$DUMP_DIR/lb_serial.txt" | head -25 | sed 's/^/     /'
    fi
fi

# ---------------------------------------------------------------- ⑦ 收尾
sec "⑦ 收尾：镜像还原"
if [ "$BUILD" = 1 ]; then
    warn "板上当前是回环自测镜像（BTR.LBKM|SILM=1，不驱动总线）——不还原会「收不到帧」"
    if [ "$RESTORE" = 1 ]; then
        do_build PROTOCOL=brake_v2 || exit 2
        do_flash || exit 2
        ok "已重建并烧写 brake_v2 Normal 镜像（可以继续 §步骤③ AEB 联调）"
    else
        info "还原命令：make clean && make PROTOCOL=brake_v2 && make flash（或本脚本加 --restore 自动做）"
    fi
else
    info "本次未构建镜像（--no-build），板上的固件未被改动"
fi

finish
