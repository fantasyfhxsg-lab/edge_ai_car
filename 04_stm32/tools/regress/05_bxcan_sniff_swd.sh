#!/usr/bin/env bash
# ============================================================================
# 05_bxcan_sniff_swd.sh —— **总线侧抓包，但不需要 CANable**：用 STM32 自己的 bxCAN 当嗅探器
#
# 为什么需要它：
#   04 证「总线上真的这么收发」，但它要 CANable；hw_verify 只用 ST-Link，却是"静默环回 +
#   自己注入刺激"。**手边只有 ST-Link、而 RDK 正挂在总线上**时，两者都差一口气：
#   前者缺抓包器，后者看不到对端真实帧。本脚本补的就是这一档 —— 直接抓真实总线的原始字节。
#
# 原理（纯 SWD，固件零改动、与对端无关）：
#   bxCAN 收到一帧后，**只把它存进"命中的最小索引过滤器组"对应的 FIFO**。固件只用
#   组 0（掩码全 0 = 全通 → FIFO0），所以组 0 一定会先命中 → 任何新加的组（比如 14）都
#   抢不到帧（实测：加组 14 全通→FIFO1，抓包 0 帧）。
#   于是本脚本反过来做：**把固件已有的组 0 临时改路由到 FIFO1**（`CAN_FFA1R.FFA0 = 1`，
#   一个位），固件读 FIFO0 读不到东西，帧全落到 FIFO1 给我们读；抓完再把这个位清回。
#   * 收发器照旧由硬件 ACK ⇒ **对端（RDK）看到的是一条完全健康的总线**，不会像"静默环回"
#     那样因无人 ACK 把对端顶进 error-passive/bus-off。
#   * 代价（可控、可逆）：抓包期间固件收不到 0x100 ⇒ 判活超时(hb_timeout_ms) ⇒ 进 HOLD
#     （PWM=0 + 短路制动 = 安全态；0x101 仍照发）。还原 FFA1R 后，收到带**新 byte3**
#     的 0x100 即自动回 CRUISE —— 由脚本末尾的 C5 快照自证。
#   * 抓包读数：`CAN_RF1R.FMP1` 轮询 + `RI1R/RDT1R/RDL1R/RDH1R` + `RF1R.RFOM1` 释放。
#     实测 SWD 稳态 ~1000 次寄存器访问/s（≈1ms/次）→ 总线 ~20 帧/s 时**追得上**，漏帧只
#     在开抓瞬间 FIFO1（仅 3 格）积压时发生，脚本会把 FIFO1 溢出次数与由 byte3 跳号
#     估出的丢帧数一起报出来（正常 ≤ 几帧）。
#
# 两种模式：
#   --mode normal（默认）：抓真实总线 —— RDK 的 0x100/0x102 原始字节与周期。
#                           ⚠ 0x101 抓不到：Normal 下节点**不接收自己发的帧**（这是 CAN 语义，
#                             不是抓包器的问题）；0x101 的线上字节由 `--mode lb` 证。
#   --mode lb          ：先 `reset` 进"静默环回"再抓 —— 固件自己发的 0x101 会被环回进
#                         RX FIFO，于是能验 0x101 的线上字节（0xAA + seq 递增 + 后 6 字节 0）；
#                         同时脚本按计划注入 6 帧已知 0x100/0x102，用来证明"注入→线上→嗅探→
#                         解码"这条链路本身是好的（工具自证）。**会摘出总线几秒**。
#
# 判定项：
#   F1 帧格式：全部 IDE=0（标准帧）/ RTR=0（数据帧）/ DLC=8
#   F2 0x100：出现且 byte0∈{0,1,2,3}、byte1/byte2=距离/置信度、byte3 逐帧 +1、byte4~7=0
#   F3 0x101：byte0=0xAA、byte1 逐帧 +1、byte2~7=0（仅 --mode lb 可判；normal 下报"不适用"）
#   F4 0x102：byte0=0x55、byte1∈{1,2}、byte2 **应答与心跳共用同一计数器且逐帧 +1**、byte3~7=0
#   F5 总线上没有别的 ID（只有 0x100/0x101/0x102）
#   F6 周期：0x100 / 0x101 / 0x102 的 min/avg/max（规格：0x100 50ms、0x101 200ms、0x102 200ms）
#   R1 抓包窗口内固件 `g_rdk_hb_rx` 原地不动（**证明确实把 RX 抢走了**，否则抓的是空气）
#   R2 还原后 1s 内 `g_rdk_hb_rx` 重新累加（**证明还原生效、RX 交还固件**）
#   R3 寄存器还原：`BTR`=Normal、`MCR.INRQ`=0、`MSR.INAK`=0、`FFA1R.FFA0`=0、`ESR`=0
#   R4 收尾安全：`st` 不是 LOCK(3)，且 `g_rdk_pwm_out` 与 `cmd` 自洽
#
# 用法：
#   bash tools/regress/05_bxcan_sniff_swd.sh                  # 抓真实总线 10s
#   bash tools/regress/05_bxcan_sniff_swd.sh --dur 5000       # 抓 5s
#   bash tools/regress/05_bxcan_sniff_swd.sh --mode lb        # 环回自证（验 0x101 线上字节）
#   ⚠ 需要 root（openocd 要 /dev/bus/usb）：脚本会自己走 wsl.exe -u root / sudo -n 提权
#     （两条都没有时用工具链容器跑，见 tools/regress/README.md §1.6；已在 openocd **0.11.0** 与
#      **0.12.0+dev** 两种版本上实测通过 —— 两者 echo 整数的进制不同，脚本内 `n2d()` 已归一化）
#   ⚠ 抓包期间固件会进 HOLD（PWM=0 + 制动）；还原后按 RDK 当前 cmd 恢复
#     → **车轮请架空**（若 RDK 正在发 cmd=01，恢复瞬间车会走）
#
# 退出码：0=全通过（含"仅提示"）；1=有 ❌；2=前置不满足；3=SWD 读回失败/符号缺失
# ============================================================================
set -u
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"
regress_init "05 bxCAN 嗅探（SWD，无 CANable）"
maybe_reexec_as_root "$@"

MODE=normal
DUR=10000
while [ $# -gt 0 ]; do
    case "$1" in
        --mode)   MODE=${2:-}; shift 2 ;;
        --dur)    DUR=${2:-};  shift 2 ;;
        -h|--help) sed -n '2,60p' "$0"; exit 0 ;;
        *) echo "用法：bash $0 [--mode normal|lb] [--dur 毫秒]"; exit 2 ;;
    esac
done
case "$MODE" in normal|lb) ;; *) echo "❌ --mode 只能是 normal 或 lb"; exit 2 ;; esac
case "$DUR" in ''|*[!0-9]*) echo "❌ --dur 必须是毫秒数"; exit 2 ;; esac
[ "$DUR" -ge 2000 ] || { echo "❌ --dur 太小（≥2000ms 才有统计意义）"; exit 2; }

# ---- 与固件/协议同步的常量（改 Src/rdk_config.c 或 .clinerules §5 必须同步这里）----
HB_TIMEOUT_MS=$(proto_num RDK_HB_TIMEOUT_MS_DEF); HB_TIMEOUT_MS=${HB_TIMEOUT_MS:-300}   # 判活阈值（只看 0x100；单一真源 Inc/can_protocol.h）
SPEC_100_MS=50             # .clinerules §5：0x100 周期 50ms
SPEC_101_MS=200            # 0x101 / 0x102 周期 200ms
BTR_NORMAL=0x004E0003      # 500kbps：BRP=3 TS1=14 TS2=3 SJW=1（现役值）
BTR_LOOP=0xC04E0003        # 上面 + LBKM|SILM（--mode lb）
ALLOW_GAP=4                # 允许的 byte3/seq 跳号次数（FIFO1 只有 3 格，开抓瞬间会丢几帧）

sec "0. 前置"
[ -f "$ELF" ] || { echo "❌ 找不到 $ELF（先 make）"; exit 2; }
sym(){ arm-none-eabi-nm -n "$ELF" 2>/dev/null | awk -v s="$1" '$3==s{print "0x"$1; exit}'; }
declare -A A
for p in g_rdk_state:A_ST g_rdk_cmd:A_CMD g_rdk_req_count:A_RQ g_rdk_hb_rx:A_HX \
         g_rdk_hb_dup:A_HD g_rdk_hb_bad:A_HB g_rdk_ack_count:A_AK g_rdk_pwm_out:A_PW \
         g_ticks:A_TK g_rdk_last_cnt:A_LN; do
    A[${p##*:}]=$(sym "${p%%:*}")
    [ -n "${A[${p##*:}]}" ] || { echo "❌ 符号缺失 ${p%%:*} —— 当前 elf 不是 heartbeat_v1 镜像？（先 make）"; exit 3; }
done
need_cmd openocd "ST-Link 调试 + 读寄存器" || exit 2
need_cmd arm-none-eabi-nm "从 elf 现读符号地址（无硬编码）" || exit 2
info "ELF=$ELF  MODE=$MODE  DUR=${DUR}ms  日志目录 $DUMP_DIR"
info "协议阈值：0x100 规格 ${SPEC_100_MS}ms、0x101/0x102 规格 ${SPEC_101_MS}ms、判活只看 0x100 且阈值 ${HB_TIMEOUT_MS}ms"
[ "$MODE" = lb ] && warn "lb 模式会把板子摘出总线几秒（静默环回）" || true

# ---------------------------------------------------------------- 生成 Tcl（一次 openocd 会话）
TCL_VARS=$DUMP_DIR/05_vars.tcl
TCL_BODY=$DUMP_DIR/05_body.tcl
cat >"$TCL_VARS" <<VARS
$(for p in g_rdk_state:A_ST g_rdk_cmd:A_CMD g_rdk_req_count:A_RQ g_rdk_hb_rx:A_HX \
             g_rdk_hb_dup:A_HD g_rdk_hb_bad:A_HB g_rdk_ack_count:A_AK g_rdk_pwm_out:A_PW \
             g_ticks:A_TK g_rdk_last_cnt:A_LN; do printf 'set %s %s\n' "${p##*:}" "${A[${p##*:}]}"; done)
set DUR $DUR
set MODE ${MODE^^}
VARS

cat >"$TCL_BODY" <<'TCL'
# 寄存器（CAN1 基址 0x40006400，见 Core/Inc/stm32f407_regs.h）
set MCR   0x40006400
set MSR   0x40006404
set TSRR  0x40006408
set RF0R  0x4000640C
set RF1R  0x40006410
set ESRR  0x40006418
set BTRR  0x4000641C
set RI1   0x400065C0
set RDT1  0x400065C4
set RDL1  0x400065C8
set RDH1  0x400065CC
set TXTIR 0x40006590
set TXDTR 0x40006594
set TXDLR 0x40006598
set TXDHR 0x4000659C
set FMR   0x40006600
set FFA1R 0x40006614
set BTR_NORMAL 0x004E0003
set BTR_LOOP   0xC04E0003

proc rd   {a} { mem2array v 32 $a 1; return $v(0) }
proc rd8  {a} { mem2array v  8 $a 1; return $v(0) }
proc rd16 {a} { mem2array v 16 $a 1; return $v(0) }
proc wr   {a v} { mww $a $v }
proc hex  {v} { return [format "%08X" $v] }
proc waitb {a mask want maxit} {          # 等寄存器位（返回 1/0）
    set t 0
    while {$t < $maxit} {
        if {[expr {([rd $a] & $mask) == $want}]} { return 1 }
        incr t
    }
    return 0
}
proc inject {id dl dh} {                  # 用邮箱 1（固件 0x101 走 TME0 最低空邮箱=邮箱 0，互不抢）
    global TXDTR TXDLR TXDHR TXTIR
    if {$dl eq "" || $dh eq ""} { echo "E9 调度表项损坏 id=$id dl='$dl' dh='$dh'"; return }
    wr $TXDTR 8
    wr $TXDLR $dl
    wr $TXDHR $dh
    wr $TXTIR [expr {($id << 21) | 1}]
}

init
echo "C0 [rd8 $A_ST] [rd8 $A_CMD] [rd $A_RQ] [rd $A_HX] [rd $A_HD] [rd $A_HB] [rd8 $A_AK] [rd16 $A_PW] [rd $A_TK] [rd $ESRR] [rd $BTRR]"

if {$MODE eq "LB"} {
    reset halt
    sleep 300
    resume
    sleep 1500                    ;# 让固件跑完 Can_Init（它会写回 BTR=Normal；RDK 在线则这段会记账）
} else {
    sleep 200
}
echo "C1a [rd8 $A_ST] [rd8 $A_CMD] [rd $A_RQ] [rd $A_HX] [rd $A_HD] [rd $A_HB] [rd8 $A_AK] [rd16 $A_PW] [rd $A_TK] [rd8 $A_LN]"

# ---- 抢 RX：进 INIT 窗口，把组 0（全通、索引最低）改路由到 FIFO1（+ lb 模式同时切静默环回）----
wr $MCR [expr {[rd $MCR] | 1}]
if {![waitb $MSR 1 1 300]} { echo "E1 INAK 未置位" }
if {$MODE eq "LB"} { wr $BTRR $BTR_LOOP }
wr $FMR   [expr {[rd $FMR] | 1}]          ;# FINIT=1
wr $FFA1R [expr {[rd $FFA1R] | 1}]        ;# FFA0=1 → 组 0 的帧进 FIFO1
wr $FMR   [expr {[rd $FMR] & ~1}]         ;# 生效
wr $MCR [expr {[rd $MCR] & ~1}]
if {![waitb $MSR 1 0 500]} { echo "E2 INAK 未清" }
echo "C1 BTR=[hex [rd $BTRR]] FFA1R=[hex [rd $FFA1R]] RF0R=[hex [rd $RF0R]] RF1R=[hex [rd $RF1R]] MSR=[hex [rd $MSR]]"

# ---- 抓包（lb 模式的 6 帧已知刺激在排空循环里按计划注入 → FIFO1 不积压）----
set sched {{300 0x100 0x115F8900 0} {450 0x100 0x125F8900 0} {600 0x100 0x135F8900 0} {750 0x102 0x210155 0} {900 0x102 0x220155 0} {1050 0x102 0x230255 0}}
set sp 0
set nf 0
set ovr 0
if {$MODE eq "LB"} { echo "S 调度表 n=[llength $sched] 首项=[lindex $sched 0] 末项=[lindex $sched 5]" }
set t0 [clock milliseconds]
set tend [expr {$t0 + $DUR}]
while {[clock milliseconds] < $tend} {
    if {$MODE eq "LB"} {
        set el [expr {[clock milliseconds] - $t0}]
        while {$sp < [llength $sched] && $el >= [lindex [lindex $sched $sp] 0]} {
            set e [lindex $sched $sp]
            inject [lindex $e 1] [lindex $e 2] [lindex $e 3]
            incr sp
        }
    }
    set rf [rd $RF1R]
    if {($rf & 0x3) == 0} {
        if {$rf & 0x10} { incr ovr; wr $RF1R 0x10 }    ;# FOVR1：记一次溢出并清标志
        continue
    }
    set ts [expr {[clock milliseconds] - $t0}]
    set cnt [expr {$rf & 0x3}]
    for {set k 0} {$k < $cnt} {incr k} {
        set ri [rd $RI1]
        set dt [rd $RDT1]
        set dl [rd $RDL1]
        set dh [rd $RDH1]
        echo "F $ts [expr {($ri >> 21) & 0x7FF}] [expr {$ri & 2}] [expr {$ri & 1}] [expr {$dt & 0xF}] [expr {$dl & 0xFF}] [expr {($dl >> 8) & 0xFF}] [expr {($dl >> 16) & 0xFF}] [expr {($dl >> 24) & 0xFF}] [expr {$dh & 0xFF}] [expr {($dh >> 8) & 0xFF}] [expr {($dh >> 16) & 0xFF}] [expr {($dh >> 24) & 0xFF}]"
        incr nf
        wr $RF1R 0x20                                  ;# RFOM1：释放
    }
}
echo "C2 frames=$nf fifo1_ovr=$ovr TSR=[hex [rd $TSRR]] ESR=[hex [rd $ESRR]] RF1R=[hex [rd $RF1R]]"
echo "C3 [rd8 $A_ST] [rd8 $A_CMD] [rd $A_RQ] [rd $A_HX] [rd $A_HD] [rd $A_HB] [rd8 $A_AK] [rd16 $A_PW] [rd $A_TK]"

# ---- 还原：组 0 回 FIFO0（+ lb 模式 BTR 回 Normal）----
wr $MCR [expr {[rd $MCR] | 1}]
if {[waitb $MSR 1 1 300]} {
    if {$MODE eq "LB"} { wr $BTRR $BTR_NORMAL }
    wr $FMR   [expr {[rd $FMR] | 1}]
    wr $FFA1R [expr {[rd $FFA1R] & ~1}]
    wr $FMR   [expr {[rd $FMR] & ~1}]
    wr $MCR [expr {[rd $MCR] & ~1}]
    waitb $MSR 1 0 500
}
while {([rd $RF1R] & 0x3) != 0} { rd $RI1; wr $RF1R 0x20 }
echo "C4 BTR=[hex [rd $BTRR]] FFA1R=[hex [rd $FFA1R]] MCR=[hex [rd $MCR]] MSR=[hex [rd $MSR]] ESR=[hex [rd $ESRR]] RF1R=[hex [rd $RF1R]]"
sleep 1000
echo "C5 [rd8 $A_ST] [rd8 $A_CMD] [rd $A_RQ] [rd $A_HX] [rd $A_HD] [rd $A_HB] [rd8 $A_AK] [rd16 $A_PW] [rd $A_TK]"
shutdown
TCL

RAW=$DUMP_DIR/05_raw.txt
info "跑一次 openocd 会话（前端 ${DUR}ms 为抓包窗口）…"
timeout $(( DUR / 1000 + 90 )) openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c 'gdb_port 0' -f "$TCL_VARS" -f "$TCL_BODY" >"$RAW" 2>&1
RC=$?
CLEAN=$RAW.clean
tr -d '\r' <"$RAW" | sed -e 's/\x1b\[[0-9;]*m//g' >"$CLEAN"
if ! grep -q '^C4 ' "$CLEAN"; then
    echo "❌ openocd 会话没跑完（rc=$RC）—— 原始输出 $RAW"; tail -5 "$CLEAN"; exit 3
fi
grep -E '^(C0|C1a|C1|C2|C3|C4|C5|E1|E2)' "$CLEAN" | sed 's/^/  /'
NF=$(grep -c '^F ' "$CLEAN" || true)
info "抓到 $NF 帧（原始 $RAW）"

# ---------------------------------------------------------------- 帧内容核对（awk 出证据 + #CHK 行）
CHK=$DUMP_DIR/05_frames.awk
cat >"$CHK" <<'AWK'
# 输入：openocd 干净日志里的 "F ts id ide rtr dlc d0..d7" 行（全十进制）
# 输出：人类可读证据 + "#CHK ok|bad|warn <文字>"（由外层 bash 记账）
BEGIN { n100=0; n101=0; n102=0; neth=0; pad0=0; bad101=0; bad102=0; gap100=0; gap102=0; gap101=0;
        fmt=0; fmtbad=0; cmdok=1; ack=0; hb=0; inj100=0; inj102a=0; inj102h=0; other="" }
/^F / {
    ts=$2; id=$3; ide=$4; rtr=$5; dlc=$6
    d0=$7; d1=$8; d2=$9; d3=$10; d4=$11; d5=$12; d6=$13; d7=$14
    fmt++
    if (ide!=0 || rtr!=0 || dlc!=8) fmtbad++
    if (id==256) {                                  # 0x100
        n100++
        if (!(d0 in cmdseen)) cmdseen[d0]=0
        cmdseen[d0]++
        if (d0>3) cmdok=0
        if (d4!=0||d5!=0||d6!=0||d7!=0) pad0++
        if (p100!="") { if ((d3-p100+256)%256 != 1) gap100++ }
        p100=d3; t100[++c100]=ts
        if (d0==0 && d1==137 && d2==95 && d3>=17 && d3<=19) inj100++    # 0x89/0x5F/0x11~0x13 = 注入刺激
    } else if (id==257) {                           # 0x101
        n101++
        if (d0!=170) bad101++
        if (d2!=0||d3!=0||d4!=0||d5!=0||d6!=0||d7!=0) pad0++
        if (p101!="") { if ((d1-p101+256)%256 != 1) gap101++ }
        p101=d1; t101[++c101]=ts
    } else if (id==258) {                           # 0x102
        n102++
        if (d0!=85) bad102++
        if (d1!=1 && d1!=2) bad102++
        if (d3!=0||d4!=0||d5!=0||d6!=0||d7!=0) pad0++
        if (p102!="") { if ((d2-p102+256)%256 != 1) gap102++ }
        p102=d2; t102[++c102]=ts
        if (d1==1) ack++
        else if (d1==2) { hb++; t102h[++c102h]=ts }
        if (d2==33||d2==34) inj102a++          # 0x21/0x22 注入应答
        if (d2==35) inj102h++                  # 0x23 注入心跳
    } else { neth++; other=other sprintf(" 0x%03X", id) }
}
function stat(t, n, nm,   i,mn,mx,sum,cnt,g,a) {
    if (n<2) return
    mn=-1; mx=-1; sum=0; cnt=0
    for (i=2;i<=n;i++) { g=t[i]-t[i-1]; if (g<=0) continue
        if (mn<0||g<mn) mn=g; if (g>mx) mx=g; sum+=g; cnt++ }
    if (cnt<1) return
    a=(t[n]-t[1])/(n-1)
    printf "-- %s 周期(ms)：%d 帧，整体均=%.1f，非零间隔 min=%d avg=%.1f max=%d\n", nm, n, a, mn, sum/cnt, mx
}
function chk(lv, t) { printf "#CHK %s %s\n", lv, t }
END {
    printf "\n-- 帧格式（标准帧 IDE=0 / 数据帧 RTR=0 / DLC=8）：共 %d 帧，违例 %d\n", fmt, fmtbad
    if (fmt==0)          chk("bad",  "一帧都没抓到（过滤器未生效 / 接线 / 总线上本来就静止？）")
    else if (fmtbad==0)  chk("ok",   "帧格式：全部标准帧 + 数据帧 + DLC=8")
    else                 chk("bad",  sprintf("帧格式违例 %d 帧（IDE/RTR/DLC 不符）", fmtbad))

    printf "-- 帧计数：0x100=%d  0x101=%d  0x102=%d  其他=%d%s\n", n100, n101, n102, neth, other
    if (n100>0) { printf "-- 0x100 cmd 分布："; for (k in cmdseen) printf " cmd=%d×%d", k, cmdseen[k]; printf "\n" }
    if (n100==0)          chk("bad",  "没抓到 0x100（RDK 没在发 / 没握手？）")
    else if (cmdok)       chk("ok",   sprintf("0x100 出现 %d 帧，byte0（cmd）全部 ∈{0,1,2,3}", n100))
    else                  chk("bad",  "0x100 出现 cmd 越界帧（byte0>3）")
    if (pad0==0)          chk("ok",   "未用字节：0x100 byte4~7 / 0x102 byte3~7 / 0x101 byte2~7 全 0")
    else                  chk("bad",  sprintf("未用字节非 0 的帧 %d 个", pad0))
    if (n100>1 && gap100==0) chk("ok", "0x100 byte3 逐帧 +1（无跳号）")
    if (n100>1 && gap100>0)  chk((gap100<=ALLOW?"warn":"bad"), sprintf("0x100 byte3 跳号 %d 处（FIFO1 仅 3 格，开抓瞬间会丢几帧，属抓包侧）", gap100))
    if (n102>0) {
        if (bad102==0 && gap102==0)
            chk("ok",  sprintf("0x102：byte0=0x55、byte1∈{1,2}、应答与心跳共用 seq 且逐帧 +1（应答 %d + 心跳 %d）", ack, hb))
        else
            chk("bad", sprintf("0x102 契约违例（byte0/byte1 违例 %d 处，seq 跳号 %d 处）", bad102, gap102))
    } else chk("warn", "没抓到 0x102（RDK 没应答 0x101？）")
    if (neth==0) chk("ok",  "总线上只有 0x100/0x101/0x102，无其他 ID")
    else         chk("warn", sprintf("总线上还有别的 ID：%s", other))

    if (MODE=="LB") {
        if (n101>0 && bad101==0 && gap101==0)
            chk("ok",  sprintf("0x101 线上字节：byte0=0xAA + byte1 逐帧 +1 + 后 6 字节 0（%d 帧，环回自证）", n101))
        else if (n101==0) chk("bad", "环回模式下没抓到 0x101（固件没发 / 环回没生效）")
        else              chk("bad", sprintf("0x101 契约违例（byte0 错 %d 处，seq 跳号 %d 处）", bad101, gap101))
        if (inj100==3)    chk("ok",  "注入自证：3 帧已知 0x100 全抓到（注入→线上→嗅探→解码 通路 OK）")
        else              chk("bad", sprintf("注入自证：3 帧已知 0x100 只抓到 %d 帧", inj100))
        if (inj102a==2 && inj102h==1)
                          chk("ok",  "注入自证：2 帧 0x102 应答(seq 0x21/0x22) + 1 帧心跳(seq 0x23) 全抓到")
        else              chk("warn", sprintf("注入自证：0x102 应答抓到 %d/2、心跳 %d/1", inj102a, inj102h))
        if (n101>1) {                                   # 环回：0x101 周期可断言（刺激只有固件自己）
            stat(t101, c101, "0x101")
            a=(t101[c101]-t101[1])/(c101-1)
            if (a>=0.9*S101 && a<=1.1*S101) chk("ok",  sprintf("0x101 周期 %.1fms ≈ 规格 %dms（环回自证）", a, S101))
            else                            chk("bad", sprintf("0x101 周期 %.1fms 偏离规格 %dms", a, S101))
        }
    } else {
        if (n101==0) chk("warn", "0x101 抓不到属正常：Normal 下节点不接收自己发的帧（要验其线上字节用 --mode lb）")
        else         chk("warn", sprintf("Normal 模式却抓到 %d 帧 0x101（本节点会存自发送帧？）", n101))
        if (ack>0) {
            want=DUR/S101; r=ack/want
            printf "-- 0x102 应答数 %d，期望 ≈%.1f（窗口 %dms ÷ 0x101 周期 %dms）→ 比值 %.2f\n", ack, want, DUR, S101, r
            if (r>=0.6 && r<=1.4) chk("ok",   sprintf("0x102 应答节拍与 0x101 周期 %dms 相符（RDK 每帧 0x101 都回）", S101))
            else                  chk("warn", sprintf("0x102 应答数比值 %.2f 偏离 1（RDK 未逐帧应答？）", r))
        }
    }
    if (hb>1) {
        stat(t102h, c102h, "0x102 心跳(type=2)")
        a=(t102h[c102h]-t102h[1])/(c102h-1)
        if (a>=0.75*S101 && a<=1.6*S101) chk("ok",   sprintf("0x102 心跳周期 %.0fms ≈ 规格 %dms", a, S101))
        else                             chk("warn", sprintf("0x102 心跳周期 %.0fms 偏离规格 %dms（RDK 侧节拍）", a, S101))
    }
    if (MODE!="LB" && n100>1) {
        stat(t100, c100, "0x100")
        a=(t100[c100]-t100[1])/(c100-1)
        if (a<=2.5*S100) chk("ok",   sprintf("0x100 周期 %.0fms（≤2.5×规格 %dms，板端只观测不断言）", a, S100))
        else             chk("warn", sprintf("0x100 周期 %.0fms 与规格 %dms 不符（RDK 侧节拍，板端不断言）", a, S100))
    } else if (n100>1) stat(t100, c100, "0x100")
    if (n102>1) stat(t102, c102, "0x102 合计")
}
AWK
awk -v MODE="${MODE^^}" -v DUR="$DUR" -v ALLOW="$ALLOW_GAP" \
    -v S100="$SPEC_100_MS" -v S101="$SPEC_101_MS" \
    -f "$CHK" "$CLEAN" >"$DUMP_DIR/05_frames.txt"
grep -E '^--' "$DUMP_DIR/05_frames.txt" | sed 's/^/  /' || true
while IFS= read -r line; do
    lv=${line#\#CHK }; lv=${lv%% *}
    txt=${line#\#CHK $lv }
    case "$lv" in
        ok)   ok   "$txt" ;;
        bad)  bad  "$txt" ;;
        warn) warn "$txt" ;;
    esac
done < <(grep '^#CHK ' "$DUMP_DIR/05_frames.txt" || true)

# ---------------------------------------------------------------- 固件侧/寄存器侧核对（R1~R4）
sec "R. 固件侧交叉核对（嗅探窗口 / 还原 / 寄存器）"
fld(){ local v; v=$(grep -m1 "^$1 " "$CLEAN" | awk -v n="$2" '{print $n}'); n2d "$v"; }
# 归一化整数写法：**不同 openocd 版本 echo 整数的方式不同** —— 0.11.0 打十进制（`1 345`），
# 0.12.0+dev 打十六进制（`0x1 0x79e`）。`mem2array` 的返回值直接 echo 就会带上 0x，
# 于是下面 `case "$ST5" in 3)`、`[ "$RQ3" -gt "$RQ1" ]` 都会退化成字符串比较而误报。
# 统一转成十进制（`$((  ))` 两种写法都能算）。kv() 取的是 Tcl `[hex …]` 格式化的值，不受影响。
n2d(){ case "${1:-}" in 0x*|0X*) echo $(( $1 )) ;; *) echo "${1:-}" ;; esac; }
kv(){  grep -m1 "^$1 " "$CLEAN" | tr ' ' '\n' | awk -F= -v k="$2" '$1==k{print $2}'; }
rbit(){ echo $(( (0x${1:-0} >> $2) & 1 )); }   # 取寄存器第 $2 位（值形如 00000C08）
ST0=$(fld C0 2);  CMD0=$(fld C0 3);  HX0=$(fld C0 5);  CMD=$(fld C5 3)
ST1=$(fld C1a 2); CMD1=$(fld C1a 3); RQ1=$(fld C1a 4); HX1=$(fld C1a 5); LN1=$(fld C1a 10)
ST3=$(fld C3 2);  CMD3=$(fld C3 3);  RQ3=$(fld C3 4);  HX3=$(fld C3 5)
ST5=$(fld C5 2);  RQ5=$(fld C5 4);   HX5=$(fld C5 5);  PW5=$(fld C5 9)
info "计数器：st/cmd/hb_rx/req  C0=$ST0/$CMD0/$HX0/-  C1a=$ST1/$CMD1/$HX1/$RQ1  C3=$ST3/$CMD3/$HX3/$RQ3  C5=$ST5/$CMD/$HX5/$RQ5"

DHX=$(( ${HX3:-0} - ${HX1:-0} ))
if [ "$DHX" -le 0 ]; then ok "抓包窗口内 g_rdk_hb_rx 原地不动（$HX1 → $HX3）：RX 确实被抢到 FIFO1"
elif [ "$DHX" -le 2 ]; then ok "抓包窗口内 g_rdk_hb_rx 只 +$DHX（$HX1 → $HX3）：切换瞬间的在途帧，RX 已被抢到 FIFO1"
else bad "抓包窗口内固件仍在收 0x100（$HX1 → $HX3，+$DHX）：FFA1R 改路由没生效，抓到的可能不完整"; fi

if [ "${RQ3:-0}" -gt "${RQ1:-0}" ] 2>/dev/null; then ok "抓包期间 0x101 仍在周期发出（req $RQ1 → $RQ3）"
else warn "抓包期间 0x101 没发出（req $RQ1 → $RQ3）"; fi

if [ "${HX5:-0}" -gt "${HX3:-0}" ] 2>/dev/null; then ok "还原后 1s 内 g_rdk_hb_rx 重新累加（$HX3 → $HX5）：RX 已交还固件"
else warn "还原后 1s 内没有新 0x100（$HX3 → $HX5）：RDK 不在线？固件留在 HOLD（PWM=0，安全态）"; fi

BTR=$(kv C4 BTR); FFA1R=$(kv C4 FFA1R); MCR=$(kv C4 MCR); MSR=$(kv C4 MSR); ESR=$(kv C4 ESR)
ESR2=$(kv C2 ESR); OVR=$(kv C2 fifo1_ovr)
info "寄存器：BTR=$BTR FFA1R=$FFA1R MCR=$MCR MSR=$MSR ESR(C2/C4)=$ESR2/$ESR  fifo1_ovr=$OVR"
case "${BTR:-}" in
    ""|00000000)        bad "BTR 未还原（读到 '$BTR'）" ;;
    004E0003|00000003)  ok  "BTR 已还原为 Normal 500kbps（$BTR）" ;;
    *)                  bad "BTR=$BTR，期望 $BTR_NORMAL" ;;
esac
if [ -z "${FFA1R:-}" ] || [ "$(rbit "$FFA1R" 0)" != 0 ]; then bad "FFA1R=$FFA1R：组 0 仍指向 FIFO1（固件收不到帧）"
else ok "FFA1R=$FFA1R：组 0 已交还 FIFO0"; fi
if [ -z "${MCR:-}" ] || [ "$(rbit "$MCR" 0)" != 0 ]; then bad "MCR=$MCR：INRQ 仍为 1（CAN 停在初始化模式）"
else ok "MCR=$MCR：INRQ=0"; fi
if [ -z "${MSR:-}" ] || [ "$(rbit "$MSR" 1)" != 0 ]; then bad "MSR=$MSR：INAK 仍为 1 或读数异常"
else ok "MSR=$MSR：INAK=0（未卡在初始化）"; fi
[ "${ESR2:-1}" = "00000000" ] && [ "${ESR:-1}" = "00000000" ] && ok "CAN_ESR 全程 = 0（无错误状态：LEC/错误计数均 0）" \
    || bad "CAN_ESR 非 0（C2=$ESR2 C4=$ESR）"
[ "${OVR:-9}" = "0" ] && ok "FIFO1 无溢出（抓包跟得上总线）" \
    || warn "FIFO1 溢出 $OVR 次（开抓瞬间积压，字节契约仍以 0x100 byte3 跳号计数为准）"

case "${ST5:-}" in
    3) bad "快照 C5 st=3 LOCK（需复位；不该出现在本脚本流程里）" ;;
    0) ok "快照 C5 st=0 STATIC（无 0x100 可执行，安全）" ;;
    1) ok "快照 C5 st=1 CRUISE（已按 RDK 当前 cmd 恢复）" ;;
    2) ok "快照 C5 st=2 HOLD（PWM=0 短路制动，安全态）" ;;
    *) warn "快照 C5 st='$ST5' 未识别" ;;
esac
case "${CMD:-255}" in
    0|3) [ "${PW5:-x}" = "0" ]   && ok "cmd=$CMD → g_rdk_pwm_out=$PW5（停车/刹停一致）" || warn "cmd=$CMD 但 pwm=$PW5（期望 0）" ;;
    1)   [ "${PW5:-x}" = "300" ] && ok "cmd=$CMD → g_rdk_pwm_out=$PW5（巡航幅值一致）" || warn "cmd=$CMD 但 pwm=$PW5（期望 300）" ;;
    2)   [ "${PW5:-x}" = "150" ] && ok "cmd=$CMD → g_rdk_pwm_out=$PW5（预警幅值一致）" || warn "cmd=$CMD 但 pwm=$PW5（期望 150）" ;;
    255) warn "还没收到过有效 0x100（g_rdk_cmd=0xFF）—— RDK 侧没在发？" ;;
    *)   warn "g_rdk_cmd='$CMD' 非预期" ;;
esac

sec "结论"
finish
