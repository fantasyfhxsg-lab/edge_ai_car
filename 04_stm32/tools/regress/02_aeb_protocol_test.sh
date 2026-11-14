#!/usr/bin/env bash
# ============================================================================
# 02_aeb_protocol_test.sh —— AEB 协议契约测试（0x123 / 0x124 / 0x125）
#
# 逐条验证 .clinerules §5 与 Inc/can_protocol.h 的线上契约（判定用 SWD 读回 + 总线抓包，
# 不依赖人眼看串口）。覆盖：
#   T1/T2 0x123 level=3 → 立即制动并锁存；静默 1s（> AEB_BRAKE_TIMEOUT_MS=500ms）
#         仍保持锁存（**刹车优先、不自动解除**）
#   T3    0x123 level=1 → 预警，只改等级、**不改锁存**（仍为 1）
#   T4    level=4（>3 非法）→ 整帧被拒，状态不变
#   T5    0x123 DLC=1（≠8）→ 整帧被拒（数据段固定 8 字节）
#   T6    0x123 level=2 → 部分制动 + 锁存
#   T7    0x123 level=0 → **唯一解锁路径**：锁存清 0
#   T8    解锁后收非法 level=5 → 状态仍为 0/0（非法帧不能把已解锁状态搞乱）
#   T9-11 0x124 100cm → g_obstacle_cm=100 且计数 +1；0xFFFF → 65535 且计数 +1；
#         DLC=1 短帧 → 计数不变（被拒）
#   T12   协议外 ID 0x1FF → 不改变任何 AEB 状态
#   T13   0x125 总线上 DLC=8、byte2..7=0、周期 < 100ms（板端 recv_timeout）
#
# 用法：
#   bash tools/regress/02_aeb_protocol_test.sh              # 完整（含烧写复位，保证初态）
#   bash tools/regress/02_aeb_protocol_test.sh --no-flash   # 不烧写（初态可能非 0/0）
#   bash tools/regress/02_aeb_protocol_test.sh --serial     # 顺带抓 3s 串口原文
#   CAN_IF=slcan0 bash ...
#
# ⚠ 安全：本脚本会真的驱动制动执行机构（level=3 短路急刹 / level=0 滑行）。
#   请把车轮架空或确认车不会冲出去；结束时脚本会发 level=0 回到解锁态。
# ============================================================================
set -u
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
regress_init "② AEB 协议契约测试（0x123/0x124/0x125）"

FLASH=1; SERIAL=0
for a in "$@"; do
    case "$a" in
        --no-flash) FLASH=0 ;;
        --serial)   SERIAL=1 ;;
        -h|--help)  sed -n '2,27p' "$0"; exit 0 ;;
        *) echo "未知参数：$a（支持 --no-flash / --serial）"; exit 2 ;;
    esac
done

maybe_reexec_as_root "$@"

# ---------------------------------------------------------------- ① 前置
sec "① 前置：工具链 / 镜像 / CAN 通道"
need_cmd openocd "容器里先 run_toolchain.sh 进环境" || exit 2
need_cmd arm-none-eabi-nm || exit 2
need_cmd python3 || exit 2
elf_check g_brake_latched g_lb_fail "make clean && make PROTOCOL=brake_v2   # 本脚本验 AEB，需 brake_v2 Normal 镜像" || exit 2       # 必须 Normal 镜像
can_detect || exit 2
can_if_autodetect || exit 2

sec "② 烧写（--no-flash 跳过；烧写=已知初态 level0/latched0）"
if [ "$FLASH" = 1 ]; then
    do_flash || exit 2
    sleep 1.5
else
    info "跳过烧写：不保证初态，T0/T1 的绝对判定会放宽为相对判定"
fi

# ---------------------------------------------------------------- SWD 读回
sec "③ T0 基线：读 AEB 状态"
swd_build g_brake_level g_brake_latched g_brake_rx_seen g_brake_rx_ms \
          g_obstacle_cm g_dist_rx_count g_speed_kmh g_speed_valid \
          CAN1_MSR CAN1_ESR || exit 3
swd_read_all "$DUMP_DIR/t0_base" || exit 3
base_level=$(val g_brake_level); base_latch=$(val g_brake_latched)
base_cm=$(val g_obstacle_cm); base_cnt=$(val g_dist_rx_count)
info "level=$base_level latched=$base_latch obstacle_cm=$base_cm dist_rx=$base_cnt"
info "MSR=$(hexv CAN1_MSR) ESR=$(hexv CAN1_ESR)（INAK/TEC/REC 应为 0）"
check_nums CAN1_MSR CAN1_ESR g_dist_rx_count || exit 3
msr=$(val CAN1_MSR); esr=$(val CAN1_ESR)
assert_eq "MSR.INAK" "$(( msr & 1 ))" 0
assert_eq "ESR.TEC" "$(( (esr >> 16) & 0xFF ))" 0
assert_eq "ESR.REC" "$(( (esr >> 24) & 0xFF ))" 0

if [ "$SERIAL" = 1 ]; then
    serial_start_bg "$DUMP_DIR/proto_serial.txt" || warn "--serial 忽略（串口不可用/被 XCOM 占用）"
fi

# ---------------------------------------------------------------- T1/T2
sec "④ T1/T2 0x123 level=3 → 立即制动并锁存；静默 1s（>500ms）不自动解除"
can_send "123#0300000000000000" >/dev/null; sleep 0.3
info "已发 123#03…，再静默 1.0s（> AEB_BRAKE_TIMEOUT_MS=500ms → 触发失效保护判定）"
sleep 1.0
check_nums g_brake_level g_brake_latched || exit 3
swd_read_all "$DUMP_DIR/t2_l3" || exit 3
assert_eq "T1 等级(发 level3 后)" "$(val g_brake_level)" 3
assert_eq "T1 锁存" "$(val g_brake_latched)" 1
assert_eq "T2 静默 1s 后仍锁存（刹车优先）" "$(val g_brake_latched)" 1
assert_eq "T2 静默 1s 后等级仍=3" "$(val g_brake_level)" 3
assert_ne "g_brake_rx_ms 已是运行时刻(≠0)" "$(val g_brake_rx_ms)" 0

# ---------------------------------------------------------------- T3/T4/T5
sec "⑤ T3 预警不改锁存 / T4 非法等级被拒 / T5 DLC≠8 被拒"
can_send "123#0100000000000000" >/dev/null; sleep 0.3    # level=1 预警
can_send "123#0400000000000000" >/dev/null; sleep 0.3    # 非法等级 4
can_send "123#03"               >/dev/null; sleep 0.3    # DLC=1 短帧
swd_read_all "$DUMP_DIR/t3_l1" || exit 3
assert_eq "T3 等级(level1 已生效)" "$(val g_brake_level)" 1
assert_eq "T3 锁存未被清（level1 不制动也不解锁）" "$(val g_brake_latched)" 1
assert_eq "T4/T5 非法帧后等级仍=1" "$(val g_brake_level)" 1
assert_eq "T4/T5 非法帧后锁存仍=1" "$(val g_brake_latched)" 1

# ---------------------------------------------------------------- T6/T7/T8
sec "⑥ T6 部分制动 / T7 level0 唯一解锁 / T8 解锁后非法帧无效"
can_send "123#0200000000000000" >/dev/null; sleep 0.3    # level=2 部分制动
swd_read_all "$DUMP_DIR/t6_l2" || exit 3
assert_eq "T6 等级" "$(val g_brake_level)" 2
assert_eq "T6 锁存" "$(val g_brake_latched)" 1
can_send "123#0000000000000000" >/dev/null; sleep 0.3    # level=0 解锁
can_send "123#0500000000000000" >/dev/null; sleep 0.3    # 非法 level=5
swd_read_all "$DUMP_DIR/t7_l0" || exit 3
assert_eq "T7 等级(level0)" "$(val g_brake_level)" 0
assert_eq "T7 锁存已清" "$(val g_brake_latched)" 0
assert_eq "T8 非法 level5 后等级仍=0" "$(val g_brake_level)" 0
assert_eq "T8 非法 level5 后锁存仍=0" "$(val g_brake_latched)" 0

# ---------------------------------------------------------------- T9/T10/T11
sec "⑦ T9-T11 0x124 障碍物距离（100cm / 0xFFFF 无效 / DLC=1 被拒）"
cnt0=$(val g_dist_rx_count)
can_send "124#6400000000000000" >/dev/null; sleep 0.2      # 100 cm = 0x0064 小端 64 00
can_send "124#FFFF000000000000" >/dev/null; sleep 0.2      # 无有效目标
can_send "124#64"               >/dev/null; sleep 0.3      # DLC=1 → 必须被拒
swd_read_all "$DUMP_DIR/t9_dist" || exit 3
check_nums g_dist_rx_count g_obstacle_cm || exit 3
assert_eq "T9/T10 合法帧计数 +2（DLC=1 的短帧不计）" "$(( $(val g_dist_rx_count) - cnt0 ))" 2
assert_eq "T10 无效距离 = 0xFFFF(65535)" "$(val g_obstacle_cm)" 65535
cnt1=$(val g_dist_rx_count)

# ---------------------------------------------------------------- T12
sec "⑧ T12 协议外 ID 0x1FF 不影响任何 AEB 状态"
can_send "1FF#0000000000000000" >/dev/null; sleep 0.3
swd_read_all "$DUMP_DIR/t12_other" || exit 3
assert_eq "T12 等级仍=0" "$(val g_brake_level)" 0
assert_eq "T12 锁存仍=0" "$(val g_brake_latched)" 0
assert_eq "T12 距离计数不变" "$(val g_dist_rx_count)" "$cnt1"

# ---------------------------------------------------------------- T13
sec "⑨ T13 0x125 线上字节契约（DLC=8 / byte2..7=0 / 周期 < 100ms）"
swd_build g_speed_kmh g_speed_valid || exit 3
swd_read_all "$DUMP_DIR/t13_speed" || exit 3
check_nums g_speed_kmh g_speed_valid || exit 3
kmh=$(val g_speed_kmh); sv=$(val g_speed_valid)
if [ "$sv" = "1" ]; then
    exp=$(printf '%02X %02X' $((kmh & 0xFF)) $(( (kmh >> 8) & 0xFF )))
else
    exp="FF FF"
fi
info "SWD：g_speed_kmh=$kmh g_speed_valid=$sv → 期望载荷 byte0 byte1 = $exp"
can_capture 20 5 "$DUMP_DIR/t13_125.txt"
S="$DUMP_DIR/t13_125.stats"; can_stats_125 "$DUMP_DIR/t13_125.txt" >"$S"
sed 's/^/     /' "$S"
assert_range "抓到 0x125 帧数" "$(stats_get "$S" count)" 18 9999
assert_eq "DLC≠8 的帧数" "$(stats_get "$S" dlc_bad)" 0
assert_eq "byte2..7 非 0 的帧数" "$(stats_get "$S" rsvd_bad)" 0
assert_range "平均周期(ms)（≈20ms，与 AEB_SPEED_PERIOD_MS 一致）" "$(round0 "$(stats_get "$S" mean_ms)")" 15 25
assert_range "最大周期(ms) < 100（板端 recv_timeout）" "$(round0 "$(stats_get "$S" max_ms)")" 0 99
case "$(stats_get "$S" vals)" in
    *"$exp"*) ok "0x125 载荷集合 [$(stats_get "$S" vals)] 含期望 $exp（与 g_speed_kmh 互证）" ;;
    *)        bad "0x125 载荷集合 [$(stats_get "$S" vals)] 不含期望 $exp" ;;
esac

# ---------------------------------------------------------------- 收尾
sec "⑩ 收尾：确认回到解锁态（安全）+ 错误计数复查"
can_send "123#0000000000000000" >/dev/null; sleep 0.4
swd_build g_brake_level g_brake_latched CAN1_ESR g_rx_overrun || exit 3
swd_read_all "$DUMP_DIR/final" || exit 3
assert_eq "末态等级" "$(val g_brake_level)" 0
assert_eq "末态锁存" "$(val g_brake_latched)" 0
check_nums CAN1_ESR || exit 3
esr=$(val CAN1_ESR)
assert_eq "全程 ESR.TEC（发送错误计数）" "$(( (esr >> 16) & 0xFF ))" 0
assert_eq "全程 ESR.REC（接收错误计数）" "$(( (esr >> 24) & 0xFF ))" 0
assert_eq "全程 ESR.BOFF" "$(( (esr >> 2) & 1 ))" 0
assert_eq "FIFO0 溢出累计 g_rx_overrun（21 帧刺激不丢帧）" "$(val g_rx_overrun)" 0

if [ "$SERIAL" = 1 ]; then
    sec "⑪ 串口原文（整场后台抓取，--serial）"
    serial_stop_bg
    if [ -s "$DUMP_DIR/proto_serial.txt" ]; then
        grep -aoE '\[(AEB|CAN)\][^[:cntrl:]]*' "$DUMP_DIR/proto_serial.txt" | head -30 | sed 's/^/     /'
    fi
fi

finish
