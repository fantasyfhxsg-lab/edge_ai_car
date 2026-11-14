#!/usr/bin/env bash
# ============================================================================
# tools/regress/lib.sh —— 三个复现脚本的公共库（只被 source，不单独执行）
#
# 提供四件事：
#   1) 记账 + 断言（ok/bad/warn/assert_eq/assert_range …，末尾统一出结论与退出码）
#   2) CAN 通道抽象：能在"本机 can-utils"或"WSL 免密 root"两侧任一执行命令，
#      脚本里不关心自己跑在容器、WSL 还是 Windows 侧
#   3) SWD 读回：nm -S 现读符号地址+宽度（无硬编码）→ openocd mdw → 解析成 name=值；
#      ⚠ 只 halt/resume，**绝不用 reset run**，否则会清掉 g_brake_latched/g_brake_rx_ms
#   4) 串口识别：按 USB VID:PID 认 CH340(1a86:7523)，**不会把 CANable(16d0:117e) 认成串口**
#
# 约定：调用方脚本开头 `set -u`，然后 source 本文件，调用 regress_init "<脚本名>"
# ============================================================================

# ---------------------------------------------------------------- 路径与全局
regress_init() {
    REGRESS_NAME=${1:-regress}
    SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    ROOT=$(cd "$SELF_DIR/../.." && pwd)            # 04_stm32 目录
    ELF=${ELF:-$ROOT/build/car.elf}
    CAN_IF=${CAN_IF:-can0}                          # slcand 没给接口名时内核叫 can0（不是故障）
    DUMP_DIR=${DUMP_DIR:-${TMPDIR:-/tmp}/regress-$(id -un)}   # 原始 openocd/candump 输出留这里
    if ! mkdir -p "$DUMP_DIR" 2>/dev/null || [ ! -w "$DUMP_DIR" ]; then
        DUMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/regress-XXXXXX")   # 目录被别的用户(如 root)建过 → 换一个
    fi
    OCD_CFG=(-f interface/stlink.cfg -f target/stm32f4x.cfg)
    PASS=0; FAIL=0; WARNS=0
    CAN_SIDE=local; CAN_PREFIX=()      # can_detect 前的安全默认值（否则 can_sh 会报 unbound variable）
    SWD_NAMES=(); SWD_MDW=(); SWD_OUT=""
    # 记下"谁在调用"：脚本会被提权到 root 跑（openocd 需要 /dev/bus/usb），
    # build/ 与 DUMP_DIR 会变成 root 属主，导致调用者之后的 `make clean` 报 Permission denied。
    # 所以在 EXIT 时统一把这两处属主还回去（见 regress_handback_build）。
    if [ "$(id -u)" != "0" ]; then
        REGRESS_OWNER="$(id -un):$(id -gn)"
        export REGRESS_OWNER
    fi
    trap regress_handback_build EXIT
    cd "$ROOT" || exit 2
}


# ---------------------------------------------------------------- 协议常数（唯一真源）
proto_num() {   # 从 Inc/can_protocol.h 现取协议级数值，避免脚本硬编码导致"假失败"
    # 用法：HB_TIMEOUT=$(proto_num RDK_HB_TIMEOUT_MS_DEF)   （取不到时调用方自行兜底）
    grep -oE "^[[:space:]]*#define[[:space:]]+$1[[:space:]]+[0-9]+u?" \
        "${ROOT:-.}/Inc/can_protocol.h" 2>/dev/null | grep -oE '[0-9]+' | head -1
}


# 收尾钩子：root 跑完把构建产物/日志目录属主还给调用者（失败不影响判定与退出码）
regress_handback_build() {
    [ "$(id -u)" = "0" ] || return 0
    [ -n "${REGRESS_OWNER:-}" ] || return 0
    if [ -d "$ROOT/build" ]; then
        chown -R "$REGRESS_OWNER" "$ROOT/build" 2>/dev/null || true
    fi
    if [ -n "${DUMP_DIR:-}" ] && [ -d "$DUMP_DIR" ]; then
        chown -R "$REGRESS_OWNER" "$DUMP_DIR" 2>/dev/null || true
    fi
    return 0
}

# ---------------------------------------------------------------- 输出与记账
sec()  { printf '\n=== %s ===\n' "$*"; }
info() { printf '  %s\n' "$*"; }
ok()   { PASS=$((PASS + 1)); printf '  ✅ %s\n' "$*"; }
bad()  { FAIL=$((FAIL + 1)); printf '  ❌ %s\n' "$*"; }
warn() { WARNS=$((WARNS + 1)); printf '  ⚠  %s\n' "$*"; }

# 断言（actual = "NA"/空 视为取不到：SWD 符号缺失、总线上没抓到、或命令失败）
assert_eq() {                    # 名 实际 期望
    if [ "$2" = "NA" ] || [ -z "${2:-}" ]; then bad "$1 取不到（SWD 符号缺失 / 总线上没抓到 / 命令失败）"; return; fi
    if [ "$2" = "$3" ]; then ok "$1 = $2"; else bad "$1 = $2，期望 $3"; fi
}
assert_ne() {                    # 名 实际 不该等于的值
    if [ "$2" = "NA" ] || [ -z "${2:-}" ]; then bad "$1 取不到"; return; fi
    if [ "$2" != "$3" ]; then ok "$1 = $2（≠ $3）"; else bad "$1 = $2，不该等于 $3"; fi
}
assert_flag() {                  # 名 实际 期望位掩码是否"全含"
    local m=$3
    if [ "${2:-NA}" = "NA" ]; then bad "$1 取不到"; return; fi
    if [ $(( $2 & m )) -eq $m ]; then ok "$1 = 0x$(printf %X "$2")（含位 0x$(printf %X "$m")）"
    else bad "$1 = 0x$(printf %X "$2")，期望含位 0x$(printf %X "$m")"; fi
}
assert_noflag() {                # 名 实际 期望"全不含"的位掩码
    local m=$3
    if [ "${2:-NA}" = "NA" ]; then bad "$1 取不到"; return; fi
    if [ $(( $2 & m )) -eq 0 ]; then ok "$1 = 0x$(printf %X "$2")（不含位 0x$(printf %X "$m")）"
    else bad "$1 = 0x$(printf %X "$2")，不该含位 0x$(printf %X "$m")"; fi
}
assert_range() {                 # 名 实际 下限 上限（整数）
    if [ "${2:-NA}" = "NA" ]; then bad "$1 取不到"; return; fi
    if [ "$2" -ge "$3" ] && [ "$2" -le "$4" ]; then ok "$1 = $2（$3~$4 内）"
    else bad "$1 = $2，超出 $3~$4"; fi
}

finish() {
    printf '\n—— %s 结论：' "$REGRESS_NAME"
    if [ "$FAIL" -gt 0 ]; then printf '❌ 失败 %d 项（通过 %d，提示 %d）\n' "$FAIL" "$PASS" "$WARNS"
    elif [ "$WARNS" -gt 0 ]; then printf '⚠  通过 %d 项，有 %d 条提示\n' "$PASS" "$WARNS"
    else printf '✅ 全部通过（%d 项）\n' "$PASS"; fi
    printf '   原始输出保留在 %s/\n' "$DUMP_DIR"
    [ "$FAIL" -eq 0 ]
}

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || { bad "缺命令 $1${2:+（$2）}"; return 1; }
    return 0
}

# 非 root 时自动换到 root 通道（openocd 要 /dev/bus/usb 权限）——避免用户手动 sudo
maybe_reexec_as_root() {
    [ "$(id -u)" = "0" ] && return 0
    [ "${REGRESS_AS_ROOT:-0}" = "1" ] && { warn "以 REGRESS_AS_ROOT=1 但 uid≠0 继续（openocd 可能失败）"; return 0; }
    if command -v wsl.exe >/dev/null 2>&1 && wsl.exe -u root -e true >/dev/null 2>&1; then
        info "非 root → 经 wsl.exe -u root 重新执行（免密通道，见 README §步骤③-②）"
        exec wsl.exe -u root -e env REGRESS_AS_ROOT=1 CAN_IF="$CAN_IF" \
             REGRESS_OWNER="${REGRESS_OWNER:-}" bash "$SELF_DIR/$(basename "$0")" "$@"
    fi
    if sudo -n true 2>/dev/null; then
        info "非 root → sudo -n 重新执行"
        exec sudo -n env REGRESS_AS_ROOT=1 CAN_IF="$CAN_IF" \
             REGRESS_OWNER="${REGRESS_OWNER:-}" bash "$SELF_DIR/$(basename "$0")" "$@"
    fi
    bad "需要 root（openocd/stp）：sudo bash $0 $* ，或先跑 wsl.exe -u root -e bash $0"
    exit 2
}

# ---------------------------------------------------------------- CAN 通道
# 脚本不关心自己跑在哪一侧：本机（容器/WSL 内）有 can-utils 就用本机，
# 否则经 wsl.exe -u root 到 WSL 侧执行（README §步骤③-② 的免密通道）。
can_detect() {
    if command -v candump >/dev/null 2>&1 && command -v cansend >/dev/null 2>&1; then
        if [ "$(id -u)" = "0" ]; then
            CAN_SIDE=local; CAN_PREFIX=()
        elif sudo -n true 2>/dev/null; then
            CAN_SIDE=sudo; CAN_PREFIX=(sudo -n)
        else
            CAN_SIDE=local; CAN_PREFIX=()
            warn "非 root 且无免密 sudo：CAN raw socket 可能被拒（Operation not permitted）；可 sudo 跑本脚本"
        fi
    elif command -v wsl.exe >/dev/null 2>&1; then
        wsl.exe -u root -e true >/dev/null 2>&1 || { bad "wsl.exe -u root 通道不可用"; return 1; }
        CAN_SIDE=wsl; CAN_PREFIX=(wsl.exe -u root -e)
    else
        bad "既没有本机 can-utils，也没有 wsl.exe 通道：装 can-utils 或按 README §步骤③ 起 slcan"
        return 1
    fi
    info "CAN 通道 = $CAN_SIDE（接口 $CAN_IF）"
    return 0
}

# 在 CAN 侧执行命令（"$@"，不经 shell）
can_do() {
    if [ "${#CAN_PREFIX[@]}" -eq 0 ]; then "$@" 2>&1 | tr -d '\r'
    else "${CAN_PREFIX[@]}" "$@" 2>&1 | tr -d '\r'; fi
}
# 在 CAN 侧执行一段 shell（需要管道/重定向时用）
can_sh() {
    if [ "${#CAN_PREFIX[@]}" -eq 0 ]; then bash -c "$1" 2>&1 | tr -d '\r'
    else "${CAN_PREFIX[@]}" bash -c "$1" 2>&1 | tr -d '\r'; fi
}

# 接口自检：存在 + state UP + 是 CAN 类型 + 500k 位速率（slcan 路线 type 280）
# 说明：slcand 少给第二个参数时内核会把接口命名成 can0（**不是故障**），故这里允许
#       接口名不同 → 自动探测 can0 / slcan0 / 任何 type 280 的接口。
can_if_autodetect() {
    local want=$CAN_IF
    if can_sh "ip link show $want >/dev/null 2>&1 && echo yes" | grep -q yes; then return 0; fi
    local alt
    for alt in can0 slcan0 can1; do
        if can_sh "ip link show $alt >/dev/null 2>&1 && echo yes" | grep -q yes; then
            warn "接口 $want 不存在，自动改用 $alt（slcand 未给接口名时就是 can0）"
            CAN_IF=$alt; return 0
        fi
    done
    bad "找不到任何 CAN 接口（$want/can0/slcan0）；先按 README §步骤③-① 起 slcand + ip link set up"
    return 1
}

can_if_check() {                 # 打印接口详情并判定 UP / CAN 类型 / 位速率
    local info_out bitrate slcand
    info_out=$(can_sh "ip -d link show $CAN_IF")
    printf '%s\n' "$info_out" | sed -n '1,8p' | sed 's/^/     /'
    if printf '%s\n' "$info_out" | grep -q 'state UP'; then
        ok "接口 $CAN_IF state UP"
    elif printf '%s\n' "$info_out" | head -1 | grep -q '<[^>]*\bUP\b'; then
        # vcan / 某些 can_dev 驱动报 state UNKNOWN，但 flags 里已有 UP（admin up 即可用）
        ok "接口 $CAN_IF 管理态 UP（flags 含 UP，驱动上报 state UNKNOWN）"
    else
        bad "接口 $CAN_IF 不是 UP（ip link set $CAN_IF up）"
    fi
    if printf '%s\n' "$info_out" | grep -q 'link/can'; then
        ok "接口族 = link/can（CAN 报文语义正常）"
    else
        bad "接口 $CAN_IF 不是 CAN 族（candump 报文字段会不可靠）"
    fi
    # slcan 特征：bittiming 常量表 [ 10000, 20000, ... ] + 有 slcand 进程（真 CAN 控制器不会这样显示）
    slcand=$(can_sh "pgrep -a slcand" | head -1)
    if [ -n "$slcand" ]; then
        ok "slcand 在跑：$slcand"
    else
        info "没有 slcand 进程（接口可能由别的工具/内核模块拉起）"
    fi
    # slcan 不暴露比特定时（bitrate 0）是正常的：真实位速率以 ④ 的 STM32 侧 BTR 反推为准
    bitrate=$(printf '%s\n' "$info_out" | sed -n 's/.*bitrate \([0-9][0-9]*\).*/\1/p' | head -1)
    case "${bitrate:-}" in
        500000) ok "内核记录位速率 = 500000 ✓" ;;
        0|"")   if printf '%s\n' "$info_out" | grep -qE '\[ *10000,'; then
                    warn "slcan 由适配器管定时（bitrate 0 正常）→ 以 ④ 的 BTR 反推 500000 为准"
                else
                    warn "内核未暴露比特定时（bitrate 0）→ 以 ④ 的 BTR 反推 500000 为准"
                fi ;;
        *)      bad "内核记录位速率 = $bitrate，不是 500000" ;;
    esac
    if printf '%s\n' "$info_out" | grep -qE 'can state (ERROR-PASSIVE|BUS-OFF)'; then
        warn "接口当前状态非 ERROR-ACTIVE（总线/终端电阻/对端供电异常）"
    fi
}

can_send() {                     # $1 = cansend 帧串，如 123#0300000000000000
    can_sh "cansend $CAN_IF '$1'"
}

# 抓 N 帧就自然退出（**不要用 timeout 杀 candump**：缓冲会被丢掉 → 假"收不到帧"）；
# 仍加 timeout 兜底 + stdbuf -oL 行缓冲，保证兜底杀进程时已到达的行不丢。
can_capture() {                  # $1=帧数 $2=兜底超时秒 $3=输出文件 [$4=candump ID 过滤，如 100:7F0]
    local n=$1 t=$2 out=$3 filt=${4:-} got
    rm -f "$out"
    can_sh "stdbuf -oL timeout $t candump -ta -n $n $CAN_IF${filt:+,$filt}" >"$out" 2>&1
    got=$(grep -cE '^[[:space:]]*\(' "$out" 2>/dev/null)
    case "$got" in ''|*[!0-9]*) got=0 ;; esac
    if [ "$CAN_SIDE" = "wsl" ] && [ "$got" -lt "$n" ]; then
        can_sh "pkill -f 'candump .*$CAN_IF'" >/dev/null 2>&1     # 清掉可能残留的 WSL 侧进程
    fi
    info "抓到 $got / $n 帧（原始：$out）"
}

# 0x125 报文统计：帧数/周期/载荷契约（小端 km/h + byte2..7 必须为 0）
can_stats_125() {                # $1=捕获文件 → 输出 key=value 行
    awk '
      BEGIN { n = 0; other = 0; dlcbad = 0; rsvdbad = 0; nd = 0; dmin = -1; dmax = 0; sum = 0
              vals = ""; first = ""; last = ""; pts = 0 }
      function num(s) { gsub(/[()]/, "", s); return s + 0 }
      {
        ts = num($1); id = $3; dlc = $4; gsub(/[\[\]]/, "", dlc)
        if (id != "125") { other++; next }
        n++
        if (dlc + 0 != 8) dlcbad++
        if (dlc + 0 == 8) for (i = 7; i <= 12; i++) if ($i != "00") rsvdbad++
        # 注：只在 DLC=8 时查 byte2..7（短帧少掉的字段不该算成"保留字节非 0"，那是 dlc_bad 的账）
        v = $5 " " $6
        if (first == "") first = v
        last = v
        if (index(vals, v) == 0) vals = (vals == "" ? v : vals "," v)
        if (n > 1) { d = (ts - pts) * 1000.0; if (dmin < 0 || d < dmin) dmin = d
                     if (d > dmax) dmax = d; sum += d; nd++ }
        pts = ts
      }
      END {
        printf "count=%d\nother=%d\ndlc_bad=%d\nrsvd_bad=%d\n", n + 0, other + 0, dlcbad + 0, rsvdbad + 0
        printf "first_val=%s\nlast_val=%s\nvals=%s\n", (first == "" ? "-" : first), (last == "" ? "-" : last), (vals == "" ? "-" : vals)
        if (nd > 0) printf "min_ms=%.2f\nmax_ms=%.2f\nmean_ms=%.2f\nfps=%.1f\n", dmin, dmax, sum / nd, 1000.0 * nd / sum
        else printf "min_ms=-1\nmax_ms=-1\nmean_ms=-1\nfps=0\n"
      }' "$1"
}
stats_get() { grep -m1 "^$2=" "$1" | cut -d= -f2-; }

# ---------------------------------------------------------------- SWD 读回
# 纪律：**只 halt + resume，绝不 reset run**（复位会清掉 g_brake_latched /
# g_brake_rx_ms 等运行态，联调读回会拿到假结论）。会话末尾 shutdown，核心继续跑。
swd_build() {                    # $1.. = 要读的名字（elf 符号 或 内置寄存器别名）
    local plan base words
    SWD_NAMES=("$@")
    plan=$(python3 "$SELF_DIR/swd_read.py" --elf "$ELF" plan "$@") || { bad "生成 mdw 计划失败"; return 1; }
    SWD_MDW=()
    while read -r base words; do
        [ -n "${base:-}" ] || continue
        SWD_MDW+=(-c "mdw $base $words")
    done <<<"$plan"
    info "SWD 计划：${SWD_MDW[*]}"
}

swd_session() {                  # $1 = raw 输出文件
    openocd "${OCD_CFG[@]}" -c 'gdb_port 0' -c init -c halt "${SWD_MDW[@]}" \
            -c resume -c shutdown >"$1" 2>&1
}

swd_read_all() {                 # $1 = 输出前缀（生成 .dump / .vals / .err）
    local out=$1 i rc
    for i in 1 2 3; do
        swd_session "$out.dump"
        if grep -qE '^0x[0-9a-fA-F]{8}:' "$out.dump"; then
            python3 "$SELF_DIR/swd_read.py" --elf "$ELF" --dump "$out.dump" emit "${SWD_NAMES[@]}" \
                >"$out.vals" 2>"$out.err"
            rc=$?
            [ "$rc" = "3" ] && warn "$(tail -1 "$out.err")"
            SWD_OUT="$out"
            return 0
        fi
        warn "第 $i 次 SWD 会话没读到内存（ST-Link 重枚举 / USB 抖动），2s 后重试"
        sleep 2
    done
    bad "3 次都没读到内存，原始输出尾部："
    tail -6 "$out.dump" | sed 's/^/     /'
    return 1
}

val()  { awk -F= -v k="$1" '$1 == k { v = $2 } END { print (v == "" ? "NA" : v) }' "$SWD_OUT.vals"; }
sval() { awk -F= -v k="$1.s32" '$1 == k { v = $2 } END { print (v == "" ? "NA" : v) }' "$SWD_OUT.vals"; }
hexv() { local v; v=$(val "$1"); [ "$v" = "NA" ] && { echo NA; return; }; printf '0x%X' "$v"; }
maskv() { local v; v=$(val "$1"); [ "$v" = "NA" ] && { echo NA; return; }; echo $(( v & $2 )); }
round0() { case "${1:-}" in ''|NA|-) echo NA ;; *) awk -v v="$1" 'BEGIN { printf "%d", v + (v >= 0 ? 0.5 : -0.5) }' ;; esac; }
# 浮点符号（本版固件无 float32 全局；保留此解码器备用）：SWD 的 mdw 只能读回**位模式**，
# 直接当整数看会得到 0x40A00000 这类值；这里按 IEEE754 小端解码成十进制显示。
# 拿不到 python3 时退回原始位模式（带 0x 前缀，一眼能看出"没解码"）。
f32() {                          # $1 = 原始 u32（十进制或 0x…）→ 如 "1.234"
    case "${1:-}" in ''|NA|-) echo NA; return ;; esac
    python3 -c 'import math, struct, sys
v = int(sys.argv[1], 0) & 0xFFFFFFFF
f = struct.unpack("<f", struct.pack("<I", v))[0]
print("0x%08X（非 float32 位模式）" % v if (math.isnan(f) or math.isinf(f)) else "%.4g" % f)' "$1" 2>/dev/null \
        || printf '0x%X\n' "$1"
}
check_nums() {   # 确认这些符号都取到值（否则随后的 $(( )) 会炸）
    local s
    for s in "$@"; do
        [ "$(val "$s")" = "NA" ] && { bad "符号 $s 取不到（SWD/elf 不匹配）"; return 1; }
    done
    return 0
}

# ---------------------------------------------------------------- 串口（可选）
# 按 USB VID:PID 认 CH340；**不认 CANable2(16d0:117e)**，避免把 CAN 适配器当串口读
usb_ids_of() {                   # $1=/dev/ttyXXX → "vid:pid"（取不到则返回 1）
    local tty base
    tty=$(basename "$1")
    for base in "/sys/class/tty/$tty/device/.." "/sys/class/tty/$tty/device"; do
        if [ -r "$base/idVendor" ] && [ -r "$base/idProduct" ]; then
            printf '%s:%s' "$(cat "$base/idVendor")" "$(cat "$base/idProduct")"
            return 0
        fi
    done
    return 1
}
find_serial_dev() {
    local d ids
    [ -n "${SERIAL_DEV:-}" ] && { printf '%s' "$SERIAL_DEV"; return 0; }
    for d in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$d" ] || continue
        ids=$(usb_ids_of "$d") || continue
        case "$ids" in
            1a86:7523|1a86:55d4|10c4:ea60) printf '%s' "$d"; return 0 ;;   # CH340/CH341/CP2102
        esac
    done
    return 1
}
serial_capture() {               # $1=秒 $2=输出文件（拿不到串口就跳过，不判失败）
    local secs=$1 out=$2 dev
    dev=$(find_serial_dev) || {
        warn "本环境没有 CH340 串口：Windows 侧先 usbipd attach --wsl --busid 1-3（或直接看 XCOM）"
        return 1
    }
    stty -F "$dev" 115200 raw -echo 2>/dev/null || { warn "stty $dev 失败（权限/占用？XCOM 独占 COM5）"; return 1; }
    timeout "$secs" cat "$dev" >"$out" 2>&1
    info "串口 $dev 采了 ${secs}s → $out（$(wc -l <"$out" | tr -d ' ') 行）"
}

# 整场后台抓串口（比"结尾抓 3s"有用：能抓到测试过程中打印的 [AEB] 状态跳变）
SERIAL_PID=""
serial_start_bg() {              # $1=输出文件；返回 0 = 已启动
    local dev
    dev=$(find_serial_dev) || return 1
    stty -F "$dev" 115200 raw -echo 2>/dev/null || return 1
    SERIAL_OUT="$1"
    cat "$dev" >"$1" 2>/dev/null &
    SERIAL_PID=$!
    info "后台抓串口 $dev → $1（结束时停）"
    return 0
}
serial_stop_bg() {
    [ -n "${SERIAL_PID:-}" ] || return 0
    kill "$SERIAL_PID" 2>/dev/null
    wait "$SERIAL_PID" 2>/dev/null
    SERIAL_PID=""
    [ -s "${SERIAL_OUT:-}" ] && info "串口原文共 $(wc -l <"$SERIAL_OUT" | tr -d ' ') 行 → $SERIAL_OUT"
    return 0
}

# ---------------------------------------------------------------- 构建/烧写
elf_check() {                    # $1=必须存在的符号 $2=必须不存在的符号（可空） $3=构建提示（可空）
    [ -f "$ELF" ] || { bad "找不到 $ELF（先 make 或 make CAN_LOOPBACK=1）"; [ -n "${3:-}" ] && info "  提示：$3"; return 1; }
    if [ -n "${1:-}" ] && ! arm-none-eabi-nm "$ELF" | grep -qE " $1\$"; then
        bad "当前 $ELF 不含符号 $1 → 不是预期镜像（Normal/回环混用会得出错结论）"
        [ -n "${3:-}" ] && info "  提示：$3"
        return 1
    fi
    if [ -n "${2:-}" ] && arm-none-eabi-nm "$ELF" | grep -qE " $2\$"; then
        bad "当前 $ELF 含符号 $2 → 镜像类型不对"
        [ -n "${3:-}" ] && info "  提示：$3"
        return 1
    fi
    ok "镜像检查通过：$ELF（$(stat -c%s "$ELF") 字节）"
}

do_build() {                     # $1.. = make 变量（如 CAN_LOOPBACK=1）
    local args=("$@")
    info "make clean && make ${args[*]}"
    make -C "$ROOT" clean >/dev/null 2>&1
    make -C "$ROOT" "${args[@]}" >"$DUMP_DIR/make.log" 2>&1
    if grep -qE 'warning|error' "$DUMP_DIR/make.log"; then
        grep -E 'warning|error' "$DUMP_DIR/make.log" | head -10 | sed 's/^/     /'
        bad "编译出现 warning/error（验收线是 0/0，见 .clinerules §2）"
        return 1
    fi
    ok "编译通过（0 warning 0 error，日志 $DUMP_DIR/make.log）"
}

do_flash() {
    info "openocd 烧写 + verify + reset"
    if ! openocd "${OCD_CFG[@]}" -c 'gdb_port 0' -c "program $ELF verify reset exit" \
            >"$DUMP_DIR/flash.log" 2>&1; then
        bad "烧写失败（看 $DUMP_DIR/flash.log）"
        tail -5 "$DUMP_DIR/flash.log" | sed 's/^/     /'
        return 1
    fi
    ok "烧写完成（日志 $DUMP_DIR/flash.log）"
    sleep 1                      # 烧写会话退出后 ST-Link 常重新枚举，立刻再开会 open failed
}

# 只复位、不重烧（RDK 状态机要求"复位 → 重新握手"才允许前进 / 退出 HOLD）：
# 比 do_flash 快一个数量级，用于把板子拉回已知的上电初态。
do_reset() {
    info "openocd reset run（不重烧，回到上电初态）"
    if ! openocd "${OCD_CFG[@]}" -c 'gdb_port 0' -c 'init' -c 'reset run' -c shutdown \
            >"$DUMP_DIR/reset.log" 2>&1; then
        bad "复位失败（看 $DUMP_DIR/reset.log）"
        tail -5 "$DUMP_DIR/reset.log" | sed 's/^/     /'
        return 1
    fi
    ok "已复位（日志 $DUMP_DIR/reset.log）"
    sleep 1                      # RDK boot ≈0.63s：睡满 1s 后正好处于握手态
}

