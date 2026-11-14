#!/usr/bin/env bash
# ============================================================================
# 00_canpipe_dryrun.sh —— CAN 管线干跑（**不需要板子 / 不需要 ST-Link / 不需要 CANable**）
#
# 用途：在动真机之前，先证明本目录三个脚本所依赖的 CAN 侧机制本身是好的：
#   can_detect / can_if_check / can_capture（candump -n + stdbuf）/ can_stats_125
#   （帧数、DLC、保留字节、周期 min/max/mean/fps、载荷多样性）/ can_send 线上字节布局（candump 反查）
# 做法：用内核自带的 vcan 虚拟 CAN（自己发自己收，报文语义与真 CAN 一致），
#       由一个"合成板子"按 20ms 发 0x125，并故意注入 1 帧脏保留字节 + 1 帧 DLC=4。
#
# 为什么值得跑：它能抓住"抓包/解析/断言"层的 bug（例如短帧把保留字节检查算脏、
# 接口 state 判定过严、CAN_PREFIX 未初始化等），这些 bug 在真机上会伪装成
# "板子不发帧"，浪费一整轮联调时间。
#
# 用法：
#   bash tools/regress/00_canpipe_dryrun.sh          # 无 vcan0 时自动提权创建，结束自动清理
#   bash tools/regress/00_canpipe_dryrun.sh --keep   # 保留 vcan0（方便自己再手抓）
#   CAN_IF=vcan0 bash tools/regress/00_canpipe_dryrun.sh
#
# 判定：全部 ✅ = CAN 管线可用；出现 ❌ 先修这里，再去折腾硬件。
# ============================================================================
set -u
CAN_IF=${CAN_IF:-vcan0}      # 必须早于 regress_init（它会把 CAN_IF 默认成 can0）
export CAN_IF
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
regress_init "⓪ CAN 管线干跑（vcan，无需硬件）"

KEEP=0
for a in "$@"; do
    case "$a" in
        --keep)    KEEP=1 ;;
        -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
        *) echo "未知参数：$a（支持 --keep）"; exit 2 ;;
    esac
done

sec "① 前置：can-utils / vcan 接口"
need_cmd candump "apt install can-utils" || exit 2
need_cmd cansend "apt install can-utils" || exit 2
need_cmd awk "gawk/mawk" || exit 2
can_detect || exit 2

CAN_IF=${CAN_IF:-vcan0}
MADE_IF=0
CAN_IF_READY=0
if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
    info "没有 $CAN_IF：需要 root 建 vcan 回环接口"
    maybe_reexec_as_root "$@"            # 不能在 repo 内乱来：只做 modprobe + ip link add
    modprobe vcan 2>/dev/null || warn "modprobe vcan 失败（内核可能没编译 vcan）"
    ip link add dev "$CAN_IF" type vcan 2>/dev/null || { bad "建 $CAN_IF 失败（内核无 vcan 模块？）"; exit 2; }
    MADE_IF=1
elif ip link show "$CAN_IF" | head -1 | grep -q '<[^>]*\bUP\b'; then
    CAN_IF_READY=1                        # 已经 UP：非 root 也能直接用（省掉 CAP_NET_ADMIN）
fi
if [ "$CAN_IF_READY" = "0" ]; then
    ip link set "$CAN_IF" up 2>/dev/null || { bad "ip link set $CAN_IF up 失败（需要 CAP_NET_ADMIN：用 sudo/wsl.exe -u root 跑本脚本）"; exit 2; }
fi
ok "使用 $CAN_IF（管理态 UP）"

EMIT_PID=""
cleanup() {
    [ -n "$EMIT_PID" ] && kill "$EMIT_PID" 2>/dev/null
    if [ "$MADE_IF" = "1" ] && [ "$KEEP" = "0" ]; then
        ip link del dev "$CAN_IF" 2>/dev/null && info "已清理临时 $CAN_IF（--keep 可保留）"
    fi
}
trap cleanup EXIT

sec "② 接口自检（vcan 与 真 CAN 走同一套判定）"
can_if_check

FRAMES=62
OUT=$DUMP_DIR/vcan_125.txt
STATS=$DUMP_DIR/vcan_125.stats

sec "③ 合成流量：40 帧干净 @20ms → 1 帧 byte2≠0 → 1 帧 DLC=4 → 20 帧干净（共 $FRAMES 帧）"
(
    sleep 0.5
    for i in $(seq 1 40); do
        cansend "$CAN_IF" "$(printf '125#%02X00000000000000' $((i % 60)))"; sleep 0.02
    done
    cansend "$CAN_IF" "125#0000010000000000"     # 8 字节但 byte2=01 → 保留字节违规
    cansend "$CAN_IF" "125#00000000"             # 只 4 字节 → DLC 违规
    for i in $(seq 1 20); do
        cansend "$CAN_IF" "$(printf '125#%02X00000000000000' $((i % 60)))"; sleep 0.02
    done
) &
EMIT_PID=$!

can_capture "$FRAMES" 10 "$OUT"
can_stats_125 "$OUT" >"$STATS"
sed 's/^/     /' "$STATS"
wait "$EMIT_PID" 2>/dev/null; EMIT_PID=""

sec "④ 抓包统计断言（对应 01 的 ⑤）"
assert_range "0x125 帧数" "$(stats_get "$STATS" count)" "$((FRAMES * 9 / 10))" 999999
assert_eq "DLC≠8 的 0x125 帧数（注入 1 帧 DLC=4 → 必须=1）" "$(stats_get "$STATS" dlc_bad)" 1
assert_eq "byte2..7 非 0 的帧数（注入 1 帧 byte2=01 → 必须=1）" "$(stats_get "$STATS" rsvd_bad)" 1
assert_eq "协议外 ID 帧数（只有 125 → 必须=0）" "$(stats_get "$STATS" other)" 0
assert_range "平均周期(ms)（合成 20ms + cansend 开销）" "$(round0 "$(stats_get "$STATS" mean_ms)")" 20 30
mmax=$(round0 "$(stats_get "$STATS" max_ms)")
if [ "$mmax" = "NA" ] || [ "$mmax" -lt 0 ]; then bad "最大周期取不到"
elif [ "$mmax" -lt 100 ]; then ok "最大周期 $(stats_get "$STATS" max_ms) ms < 100ms（recv_timeout 契约）"
else bad "最大周期 $(stats_get "$STATS" max_ms) ms ≥ 100ms"; fi
kmh=20                                       # 与合成器末帧（i=20=0x14）对齐，模拟"SWD 侧变量"
exp=$(printf '%02X %02X' $((kmh & 0xFF)) $(( (kmh >> 8) & 0xFF )))
assert_eq "末帧 byte0/1 与 SWD 侧 g_speed_kmh=$kmh 互证（小端）" "$(stats_get "$STATS" last_val)" "$exp"

sec "⑤ can_send 线上字节布局（candump 反查：脚本发的就是协议字节）"
( timeout 4 candump -ta -n 2 "$CAN_IF" >"$DUMP_DIR/vcan_tx.txt" 2>&1 ) &
sleep 0.6
can_send "123#0300000000000000" >/dev/null   # level=3 紧急制动
can_send "124#6400000000000000" >/dev/null   # 100cm（小端低字节 0x64）
wait
sed 's/^/     /' "$DUMP_DIR/vcan_tx.txt"
assert_eq "0x123 帧（ID DLC byte0）" \
    "$(awk '$3=="123" { print $3, $4, $5 }' "$DUMP_DIR/vcan_tx.txt" | tr '\n' '|' | sed 's/|$//')" \
    "123 [8] 03"
assert_eq "0x124 帧（ID DLC byte0=100cm 低字节）" \
    "$(awk '$3=="124" { print $3, $4, $5 }' "$DUMP_DIR/vcan_tx.txt" | tr '\n' '|' | sed 's/|$//')" \
    "124 [8] 64"

sec "⑥ 结论"
info "本干跑只证明「CAN 管线 + 判定逻辑」可用；位速率真值/BTR/AEB 状态仍必须用 01/02/03 在真机上验。"
finish
