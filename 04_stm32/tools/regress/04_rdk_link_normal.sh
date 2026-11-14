#!/usr/bin/env bash
# ============================================================================
# 04_rdk_link_normal.sh —— RDK 行车链路契约测试（heartbeat_v1：0x100/0x101/0x102）
#
# 判定全用 **SWD 读回**（nm 现读符号地址，无硬编码）+ 总线抓包，不靠人眼看串口；
# `--serial` 时另抓串口原文供人工复核。
#
# 固件契约（Inc/can_protocol.h「RDK 行车链路协议」+ Src/main.c 状态机）：
#   状态码 st：0 STATIC / 1 CRUISE / 2 HOLD / 3 LOCK
#   STATIC  上电/复位：PWM=0，0x101@200ms 恒发，**忽略 0x100**（只记账不执行）；
#           2 帧有效应答（DLC=8 && byte0=0x55 && byte1=0x01 && byte2 递增）→ CRUISE；
#           60s 未握手 → LOCK（只有 --lock 才测，默认跳过）。
#   CRUISE  唯一可驱动状态；每 200ms 发 0x101；按有效 0x100 的 byte0 执行
#           （00 停车 PWM=0 / 01 巡航 / 02 预警 warn_pwm / 03 刹停+短路制动）；
#           **判活只看 0x100**：静默 ≥ hb_timeout_ms(300) → HOLD。
#   HOLD    PWM=0 + 0x101@200ms；**唯一恢复条件 = 收到有效 0x100**（0x102 不能恢复）。
#   LOCK    PWM=0 + LED 常亮 + **停发 0x101**；需复位。
#
# 覆盖：
#   T1  复位初态：st=0 STATIC、PWM=0（未握手不许动）
#   T1b 总线侧 0x101：DLC=8 + AA + seq + 6×00，周期 ≈200ms
#   T1c DLC 契约：0x102 短帧（DLC=2）必须被拒（应答不计数）→ §5「数据段固定 8 字节」
#   T1d STATIC 忽略 0x100（不执行命令）+ byte3 去重（同 cnt 第二帧被丢弃、不刷新判活）
#   T2  2×0x102（DLC=8，byte2 递增）→ st=1 CRUISE：PWM 真的到引脚（CCR/GPIO/斜坡）
#   T2b 编码器闭环（负载侧证据，未上电则只提示）
#   T3  持续喂 0x100 cmd=01：维持巡航；有效帧累加、重复帧 0、坏帧 0
#   T4  0x100 cmd=02 → warn_pwm + buzz=1（LED0 慢闪）
#   T4b 0x100 cmd=00 → PWM=0 但**仍在 CRUISE**（停车不是 HOLD）
#   T5  0x100 cmd=03 → PWM=0 + 短路制动，**仍在 CRUISE**（本轮已无 STOPPING 态）
#   T6  判活铁律（**核心回归**）：只停 0x100、继续发 0x102 → ≤ hb_timeout_ms(300) 仍进 HOLD（PWM=0）
#   T7  HOLD 恢复：0x102 不能恢复；重复 0x100 不能恢复；**新 byte3 的有效 0x100** 才恢复
#   T8  （--lock）60s 未握手 → LOCK：st=3 + PWM=0 + **停发 0x101**
#
# 用法：
#   bash tools/regress/04_rdk_link_normal.sh              # 复位到初态（推荐）
#   bash tools/regress/04_rdk_link_normal.sh --no-reset   # 不复位（初态不保证，T1 放宽）
#   bash tools/regress/04_rdk_link_normal.sh --serial     # 顺带抓串口原文
#   bash tools/regress/04_rdk_link_normal.sh --lock       # 追加 T8（+65s，验证 60s 锁定）
#   CAN_IF=slcan0 bash tools/regress/04_rdk_link_normal.sh
#
# ⚠ 安全：T2~T5 会真的驱动电机（cruise_pwm=300 → 30% 占空，合计约 1s）。请架空车轮。
#   脚本退出前补发 cmd=03（HOLD 下被忽略；万一还在巡航则立即刹停）。
# ============================================================================
set -u
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
regress_init "④ RDK 行车链路契约测试（heartbeat_v1 0x100/0x101/0x102）"

RESET=1; SERIAL=0; LOCKTEST=0
for a in "$@"; do
    case "$a" in
        --no-reset) RESET=0 ;;
        --serial)   SERIAL=1 ;;
        --lock)     LOCKTEST=1 ;;
        -h|--help)  sed -n '2,45p' "$0"; exit 0 ;;
        *) echo "未知参数：$a（支持 --no-reset / --serial / --lock）"; exit 2 ;;
    esac
done

maybe_reexec_as_root "$@"


# ---- 与 Src/rdk_config.c 的标定同步（改配置必须同步这里，否则判定会"假失败"）----
CRUISE_PWM=300        # g_rdk_cfg.cruise_pwm
WARN_PWM=150          # g_rdk_cfg.warn_pwm
HB_TIMEOUT=$(proto_num RDK_HB_TIMEOUT_MS_DEF); HB_TIMEOUT=${HB_TIMEOUT:-300}   # g_rdk_cfg.hb_timeout_ms（单一真源 Inc/can_protocol.h；护栏 [200,500]）
HS_PERIOD=200         # g_rdk_cfg.hs_period_ms（0x101 恒发周期）
LOCK_S=60             # g_rdk_cfg.lock_timeout_ms / 1000
# ---- 状态码 / 执行机构 / 命令码（与 Src/main.c、Inc/can_protocol.h 一一对应）----
ST_STATIC=0; ST_CRUISE=1; ST_HOLD=2; ST_LOCK=3
MOTOR_FORWARD=1; MOTOR_BRAKE=3
CMD_PARK=0; CMD_CRUISE=1; CMD_ALERT=2; CMD_STOP=3
STOP_V_MAX=2          # 编码器残速上限 cm/s（松边界：rdk_config 阈值≈1.22）
CMD_NONE=255          # g_rdk_cmd 初值
ENC_LIVE=0            # T2b 是否测到编码器脉冲（决定 T5 的运动学判定做不做）

# ---- 帧构造（**每条都必须 DLC=8**：'#' 后正好 16 个 hex 字符）----
# 0x100 byte0=cmd byte1=距离cm byte2=置信度 byte3=计数(递增) byte4-7=0
hb_frame() { printf '100#%02X%02X%02X%02X00000000' "$1" "$2" "$3" "$4"; }
# 0x102 byte0=0x55 byte1=0x01 应答 / 0x02 心跳 byte2=seq(递增) byte3-7=0
hs_ack()   { printf '102#5501%02X0000000000' "$1"; }
hs_hb()    { printf '102#5502%02X0000000000' "$1"; }

CNT=0   # 0x100 byte3 计数：逐帧递增，否则被应用层当重复帧丢弃（且不刷新看门狗）
burst() {   # $1=cmd $2=距离 $3=置信度 $4=帧数 $5=帧间隔(s)
    local cmd=$1 dist=$2 conf=$3 n=$4 iv=$5 i script=""
    for ((i = 0; i < n; i++)); do
        CNT=$(( (CNT + 1) % 256 ))
        script+="cansend $CAN_IF $(hb_frame "$cmd" "$dist" "$conf" "$CNT"); sleep $iv; "
    done
    can_sh "$script" >/dev/null
}

do_handshake() {   # 2×0x102 应答（DLC=8，byte2 递增 1→2）；DLC≠8 一律被驱动层拒（见 T1c）
    can_sh "cansend $CAN_IF $(hs_ack 1); sleep 0.02; cansend $CAN_IF $(hs_ack 2)" >/dev/null
}

# 后台"喂狗"：进巡航后**必须每 < hb_timeout_ms(300) 有有效 0x100**，否则看门狗本地刹停（这是固件设计）。
# SWD 读回（halt/resume 要几百 ms）期间必须持续喂，否则读到的是"刚被掐掉的停态"，
# 会把正确的固件行为判成失败。计数域用 0xA0.. 与主流程 CNT 分开。
FEED_PID=""
feed_start() {   # $1=cmd $2=帧数 $3=间隔(s)
    local i
    (
        for ((i = 0; i < $2; i++)); do
            can_sh "cansend $CAN_IF $(hb_frame "$1" 20 90 $(( (0xA0 + i) % 256 )))" >/dev/null 2>&1
            sleep "$3"
        done
    ) &
    FEED_PID=$!
}
feed_stop() {
    if [ -n "$FEED_PID" ]; then kill "$FEED_PID" 2>/dev/null; wait "$FEED_PID" 2>/dev/null; fi
    FEED_PID=""
    return 0
}
# 后台只发 0x102（递增 seq）：T6 用来证明「0x102 不能阻止判活」
ACK_PID=""
ack_start() {   # $1=帧数 $2=间隔(s)
    local i
    (
        for ((i = 0; i < $2; i++)); do
            can_sh "cansend $CAN_IF $(hs_ack $(( (0x30 + i) % 256 )))" >/dev/null 2>&1
            sleep "$3"
        done
    ) &
    ACK_PID=$!
}
ack_stop() {
    if [ -n "$ACK_PID" ]; then kill "$ACK_PID" 2>/dev/null; wait "$ACK_PID" 2>/dev/null; fi
    ACK_PID=""
    return 0
}
# 收尾安全网：不管脚本从哪一步退出，最后一次动作都是"刹停"（HOLD 下被忽略；巡航下立即停）
SAFE_ARMED=0
safe_stop() {
    [ "$SAFE_ARMED" = 1 ] || return 0
    CNT=$(( (CNT + 1) % 256 ))
    can_sh "cansend $CAN_IF $(hb_frame "$CMD_STOP" 0 0 "$CNT")" >/dev/null 2>&1 || true
}
trap 'feed_stop; ack_stop; safe_stop' EXIT


# ---------------------------------------------------------------- ① 前置
sec "① 前置：工具链 / 镜像 / CAN 通道"
need_cmd openocd "容器里先按 README §③ 进环境" || exit 2
need_cmd arm-none-eabi-nm || exit 2
need_cmd python3 || exit 2
elf_check g_rdk_state g_lb_round "make clean && make   # 本脚本验 heartbeat_v1 的 Normal 镜像，默认构建即是" || exit 2
can_detect || exit 2
can_if_autodetect || exit 2

# ---------------------------------------------------------------- ② 复位初态
sec "② 复位到上电初态（--no-reset 可跳过）"
if [ "$RESET" = 1 ]; then
    do_reset || exit 2
else
    info "跳过复位：初态不保证（若板子已 LOCK，T1 的断言会失败，属预期）"
fi
SAFE_ARMED=1     # 从这一刻起，任何退出路径都会补发一次 cmd=03（安全网）

# ---------------------------------------------------------------- ③ T1 初态
sec "③ T1 初态：STATIC(0) + PWM=0 —— 未握手不许动"
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_req_count g_rdk_ack_count g_current_speed g_state
swd_read_all "$DUMP_DIR/t1_init" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_req_count g_rdk_ack_count g_current_speed g_state || exit 2
info "st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) cmd=$(val g_rdk_cmd) req=$(val g_rdk_req_count) ack=$(val g_rdk_ack_count) spd=$(val g_current_speed) motor=$(val g_state)"
assert_eq "st = STATIC(${ST_STATIC})（上电/复位后必须等握手）" "$(val g_rdk_state)" "$ST_STATIC"
assert_eq "PWM 目标 = 0（未握手不得驱动）"           "$(val g_rdk_pwm_out)" 0
assert_eq "g_current_speed = 0（电机抽象层也停着）"  "$(val g_current_speed)" 0
assert_ne "电机不在正转"                             "$(val g_state)" "$MOTOR_FORWARD"
assert_eq "cmd = 0xFF（没有任何 0x100 命令被执行过）" "$(val g_rdk_cmd)" "$CMD_NONE"
assert_eq "ack 计数 = 0（还没收到应答）"             "$(val g_rdk_ack_count)" 0
if [ "$RESET" = 1 ]; then
    assert_range "0x101 已周期性发出（req_count，≈2s/200ms）" "$(val g_rdk_req_count)" 1 30
else
    warn "--no-reset：req_count=$(val g_rdk_req_count)（上电时长不定，不做区间判定）"
fi

# ---------------------------------------------------------------- ④ T1b 总线侧 0x101
sec "④ T1b 总线侧 0x101：DLC=8 + AA + seq + 6×00，周期 ≈${HS_PERIOD}ms"
can_if_check "$CAN_IF"
can_capture 5 3 "$DUMP_DIR/t1_101.dump" "101:7FF"
n101=$(grep -cE '^[[:space:]]*\(' "$DUMP_DIR/t1_101.dump" 2>/dev/null || true)
case "$n101" in ''|*[!0-9]*) n101=0 ;; esac
fmt101=$(grep -cE '101[[:space:]]+\[8\][[:space:]]+AA[[:space:]]+[0-9A-Fa-f]{2}([[:space:]]+00){6}' "$DUMP_DIR/t1_101.dump" 2>/dev/null || true)
case "$fmt101" in ''|*[!0-9]*) fmt101=0 ;; esac
info "0x101 帧 = $n101，格式合规 = $fmt101（原始：$DUMP_DIR/t1_101.dump）"
assert_eq "0x101 在 3s 内抓满 5 帧（⇒ 周期 ≤600ms；若 60s 就停发只能抓到 0~1 帧）" "$n101" 5
assert_eq "0x101 帧格式全部合规（DLC=8 且 AA+seq+6×00）" "$fmt101" "$n101"

# ---------------------------------------------------------------- ⑤ T1c DLC 契约
sec "⑤ T1c DLC 契约：0x102 短帧（DLC=2）必须被拒（应答不计数）"
can_send '102#5501' >/dev/null
sleep 0.15
swd_build g_rdk_state g_rdk_ack_count g_rdk_last_rx_102
swd_read_all "$DUMP_DIR/t1c_dlc" || exit 2
check_nums g_rdk_state g_rdk_ack_count g_rdk_last_rx_102 || exit 2
info "st=$(val g_rdk_state) ack=$(val g_rdk_ack_count) last_rx_102=$(val g_rdk_last_rx_102)"
assert_eq "ack 计数仍为 0（短帧没进应用层）"     "$(val g_rdk_ack_count)" 0
assert_eq "last_rx_102 仍为 0（驱动层就丢了）"     "$(val g_rdk_last_rx_102)" 0
assert_eq "st 仍 STATIC(${ST_STATIC})"            "$(val g_rdk_state)" "$ST_STATIC"

# ---------------------------------------------------------------- ⑥ T1d STATIC 忽略 + 去重
sec "⑥ T1d STATIC 忽略 0x100（只记账不执行）+ byte3 去重（同 cnt 第二帧丢弃）"
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb_rx g_rdk_hb_dup g_rdk_hb_bad g_rdk_hb g_rdk_last_cnt g_rdk_hb_ms g_rdk_last_rx_100
swd_read_all "$DUMP_DIR/t1d_base" || exit 2
check_nums g_rdk_hb_rx g_rdk_hb_dup g_rdk_hb_bad g_rdk_cmd || exit 2
rx0=$(val g_rdk_hb_rx); dup0=$(val g_rdk_hb_dup); bad0=$(val g_rdk_hb_bad)
info "baseline: rx=$rx0 dup=$dup0 bad=$bad0"
f_dup=$(hb_frame "$CMD_STOP" 25 80 7)      # 故意用 cmd=03：STATIC 里也不许动电机
can_send "$f_dup" >/dev/null              # 第 1 帧：有效帧（记账）
sleep 0.15
can_send "$f_dup" >/dev/null              # 第 2 帧：cnt 与上一帧相同 → 必须被去重丢弃
sleep 0.15
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb_rx g_rdk_hb_dup g_rdk_hb_bad g_rdk_hb g_rdk_last_cnt g_rdk_hb_ms g_rdk_last_rx_100
swd_read_all "$DUMP_DIR/t1d_after" || exit 2
check_nums g_rdk_hb_rx g_rdk_hb_dup g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb || exit 2
info "after: rx=$(val g_rdk_hb_rx) dup=$(val g_rdk_hb_dup) bad=$(val g_rdk_hb_bad) hb=$(hexv g_rdk_hb) st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) cmd=$(val g_rdk_cmd)"
assert_eq "有效帧被记账：hb_rx = $rx0 + 1"            "$(val g_rdk_hb_rx)" "$((rx0 + 1))"
assert_eq "重复帧被丢弃：hb_dup = $dup0 + 1"          "$(val g_rdk_hb_dup)" "$((dup0 + 1))"
assert_eq "无坏帧：hb_bad 不变（DLC=8 且 cmd=03 合法）" "$(val g_rdk_hb_bad)" "$bad0"
assert_eq "保留最近有效帧：g_rdk_hb = 0x501903（cmd=03|dist=25|conf=80）" "$(printf '0x%X' "$(( $(val g_rdk_hb) & 0xFFFFFF ))")" 0x501903
assert_eq "去重基准 last_cnt = 7"                     "$(val g_rdk_last_cnt)" 7
assert_eq "判活时间戳与 hb_ms 同步（last_rx_100 == hb_ms）" "$(val g_rdk_last_rx_100)" "$(val g_rdk_hb_ms)"
assert_eq "st 仍 STATIC(${ST_STATIC})（STATIC 忽略 0x100）" "$(val g_rdk_state)" "$ST_STATIC"
assert_eq "PWM 仍 0（STATIC 收到 cmd=03 也不执行）"   "$(val g_rdk_pwm_out)" 0
assert_eq "cmd 仍 0xFF（STATIC 不执行任何 0x100 命令）" "$(val g_rdk_cmd)" "$CMD_NONE"

# ---------------------------------------------------------------- ⑦ T2 握手→CRUISE
sec "⑦ T2 2×0x102 应答 → CRUISE(1)：PWM 真的到引脚（CCR/GPIO/斜坡）"
do_handshake                        # 2 帧 DLC=8 应答（byte2 递增 1→2）
feed_start "$CMD_CRUISE" 60 0.1     # 进巡航起必须每 <200ms 有有效 0x100（后台自动喂 6s）
sleep 0.4
swd_build g_rdk_state g_rdk_ack_count g_rdk_cmd g_rdk_pwm_out g_current_speed g_state g_rdk_hb_rx GPIOB_ODR TIM3_ARR TIM3_CCR1 TIM3_CCR2
swd_read_all "$DUMP_DIR/t2_cruise" || exit 2
check_nums g_rdk_state g_rdk_ack_count g_rdk_cmd g_rdk_pwm_out g_current_speed g_state g_rdk_hb_rx GPIOB_ODR TIM3_ARR TIM3_CCR1 TIM3_CCR2 || exit 2
arr=$(val TIM3_ARR)
exp_ccr=$(awk -v p="$CRUISE_PWM" -v a="$arr" 'BEGIN{printf "%d", p*(a+1)/1000.0 + 0.5}')
info "st=$(val g_rdk_state) ack=$(val g_rdk_ack_count) cmd=$(val g_rdk_cmd) pwm=$(val g_rdk_pwm_out) spd=$(val g_current_speed) motor=$(val g_state) ARR=$arr → CCR=$exp_ccr"
assert_eq "st = CRUISE(${ST_CRUISE})（累计 2 帧有效应答即进巡航）" "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "握手应答计数 = 2"                        "$(val g_rdk_ack_count)" 2
assert_eq "cmd = 01（进巡航即按巡航命令驱动）"      "$(val g_rdk_cmd)" "$CMD_CRUISE"
assert_eq "PWM 目标 = ${CRUISE_PWM}"                "$(val g_rdk_pwm_out)" "$CRUISE_PWM"
assert_eq "斜坡已爬到 ${CRUISE_PWM}（RAMP_STEP=80/20ms）" "$(val g_current_speed)" "$CRUISE_PWM"
assert_eq "电机状态 = FORWARD(1)"                   "$(val g_state)" "$MOTOR_FORWARD"
assert_range "0x100 开始被记账（hb_rx ≥ 1）"        "$(val g_rdk_hb_rx)" 1 100000000
assert_eq "AIN1=BIN1=H、AIN2=BIN2=L（PB0/PB4，掩码 0x33）" "$(maskv GPIOB_ODR 0x33)" "$((0x11))"
assert_eq "STBY 拉高（PB3）"                        "$(maskv GPIOB_ODR 0x08)" 8
assert_eq "TIM3_CH1(左轮 PWMA) CCR = ${exp_ccr}"    "$(val TIM3_CCR1)" "$exp_ccr"
assert_eq "TIM3_CH2(右轮 PWMB) CCR = ${exp_ccr}"    "$(val TIM3_CCR2)" "$exp_ccr"

# ---------------------------------------------------------------- ⑧ T2b 编码器
sec "⑧ T2b 编码器闭环（负载侧证据；车轮架空/未上电时只提示，不判失败）"
swd_build g_rdk_v_cmps g_wheel_l_cmps g_wheel_r_cmps TIM4_CNT TIM8_CNT
swd_read_all "$DUMP_DIR/t2b_enc_a" || exit 2
check_nums g_rdk_v_cmps g_wheel_l_cmps g_wheel_r_cmps TIM4_CNT TIM8_CNT || exit 2
c1=$(( $(val TIM4_CNT) + $(val TIM8_CNT) )); v1=$(val g_rdk_v_cmps)
sleep 0.4
swd_build g_rdk_v_cmps g_wheel_l_cmps g_wheel_r_cmps TIM4_CNT TIM8_CNT
swd_read_all "$DUMP_DIR/t2b_enc_b" || exit 2
c2=$(( $(val TIM4_CNT) + $(val TIM8_CNT) )); v2=$(val g_rdk_v_cmps)
info "轮速 v_cmps：$v1 → $v2；编码器计数（TIM4+TIM8）：$c1 → $c2；左右轮：$(val g_wheel_l_cmps) / $(val g_wheel_r_cmps) cm/s"
if [ "$c1" != "$c2" ] || [ "$v2" != 0 ]; then
    ENC_LIVE=1
    ok "编码器在计数（闭环可观测）"
    assert_range "实测轮速 g_rdk_v_cmps > 0" "$v2" 1 100000
else
    warn "编码器无脉冲（车轮架空/不上电）→ 后续运动学判定按\"未上电\"处理"
fi

# ---------------------------------------------------------------- ⑨ T3 维持巡航
sec "⑨ T3 持续喂 0x100 cmd=01：维持巡航 + 有效/重复/坏帧计数"
feed_stop; feed_start "$CMD_CRUISE" 60 0.1
sleep 1.0
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb_rx g_rdk_hb_dup g_rdk_hb_bad g_rdk_hb g_current_speed
swd_read_all "$DUMP_DIR/t3_cruise" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb_rx g_rdk_hb_dup g_rdk_hb_bad g_rdk_hb g_current_speed || exit 2
info "st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) cmd=$(val g_rdk_cmd) rx=$(val g_rdk_hb_rx) dup=$(val g_rdk_hb_dup) bad=$(val g_rdk_hb_bad) hb=$(hexv g_rdk_hb)"
assert_eq "st 仍 CRUISE(${ST_CRUISE})"          "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "PWM 仍 ${CRUISE_PWM}"                 "$(val g_rdk_pwm_out)" "$CRUISE_PWM"
assert_eq "cmd 仍 01"                            "$(val g_rdk_cmd)" "$CMD_CRUISE"
assert_eq "斜坡保持 ${CRUISE_PWM}"                "$(val g_current_speed)" "$CRUISE_PWM"
assert_eq "最近有效帧 = 0x5A1401（cmd=01|dist=20|conf=90）" "$(printf '0x%X' "$(( $(val g_rdk_hb) & 0xFFFFFF ))")" 0x5A1401
assert_range "有效帧持续累加（hb_rx ≥ 10）"       "$(val g_rdk_hb_rx)" 10 100000000
assert_eq "重复帧 = 0（喂狗帧 byte3 逐帧递增，不该被去重）" "$(val g_rdk_hb_dup)" 0
assert_eq "坏帧 = 0（DLC=8 且 cmd≤3）"           "$(val g_rdk_hb_bad)" 0

# ---------------------------------------------------------------- ⑩ T4 cmd=02
sec "⑩ T4 0x100 cmd=02 → 预警减速（warn_pwm=${WARN_PWM} + buzz=1）"
feed_stop; feed_start "$CMD_ALERT" 40 0.1
sleep 0.4
swd_build g_rdk_state g_rdk_cmd g_rdk_pwm_out g_rdk_buzz_on g_current_speed g_state
swd_read_all "$DUMP_DIR/t4_alert" || exit 2
check_nums g_rdk_state g_rdk_cmd g_rdk_pwm_out g_rdk_buzz_on g_current_speed g_state || exit 2
info "st=$(val g_rdk_state) cmd=$(val g_rdk_cmd) pwm=$(val g_rdk_pwm_out) buzz=$(val g_rdk_buzz_on) spd=$(val g_current_speed)"
assert_eq "st 仍 CRUISE(${ST_CRUISE})（预警只减速，不掉状态）" "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "cmd = 02"                              "$(val g_rdk_cmd)" "$CMD_ALERT"
assert_eq "PWM 目标 = ${WARN_PWM}"                 "$(val g_rdk_pwm_out)" "$WARN_PWM"
assert_eq "斜坡已降到 ${WARN_PWM}"                  "$(val g_current_speed)" "$WARN_PWM"
assert_eq "buzz/LED 慢闪请求 = 1"                   "$(val g_rdk_buzz_on)" 1
assert_eq "电机仍 FORWARD(1)（半速前进）"          "$(val g_state)" "$MOTOR_FORWARD"

# ---------------------------------------------------------------- ⑪ T4b cmd=00
sec "⑪ T4b 0x100 cmd=00 → PWM=0 但**仍在 CRUISE**（停车 ≠ HOLD）"
feed_stop; feed_start "$CMD_PARK" 40 0.1
sleep 0.5
swd_build g_rdk_state g_rdk_cmd g_rdk_pwm_out g_current_speed g_state g_rdk_buzz_on
swd_read_all "$DUMP_DIR/t4b_park" || exit 2
check_nums g_rdk_state g_rdk_cmd g_rdk_pwm_out g_current_speed g_state g_rdk_buzz_on || exit 2
info "st=$(val g_rdk_state) cmd=$(val g_rdk_cmd) pwm=$(val g_rdk_pwm_out) spd=$(val g_current_speed) motor=$(val g_state)"
assert_eq "st 仍 CRUISE(${ST_CRUISE})（0x100 还在流，不该进 HOLD）" "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "cmd = 00"                              "$(val g_rdk_cmd)" "$CMD_PARK"
assert_eq "PWM 目标 = 0"                           "$(val g_rdk_pwm_out)" 0
assert_eq "斜坡已回 0（滑行停车）"                  "$(val g_current_speed)" 0
assert_eq "电机状态 = STOP(0)（没被短路制动）"      "$(val g_state)" 0
assert_eq "buzz 清零"                              "$(val g_rdk_buzz_on)" 0

# ---------------------------------------------------------------- ⑫ T5 cmd=03
sec "⑫ T5 0x100 cmd=03 → PWM=0 + 短路制动，仍在 CRUISE（本轮已无 STOPPING 态）"
feed_stop; feed_start "$CMD_STOP" 40 0.1
sleep 0.4
swd_build g_rdk_state g_rdk_cmd g_rdk_pwm_out g_current_speed g_state GPIOB_ODR g_rdk_v_cmps
swd_read_all "$DUMP_DIR/t5_brake" || exit 2
check_nums g_rdk_state g_rdk_cmd g_rdk_pwm_out g_current_speed g_state GPIOB_ODR g_rdk_v_cmps || exit 2
info "st=$(val g_rdk_state) cmd=$(val g_rdk_cmd) pwm=$(val g_rdk_pwm_out) spd=$(val g_current_speed) motor=$(val g_state) IN=0x$(printf '%X' "$(maskv GPIOB_ODR 0x33)")"
assert_eq "st 仍 CRUISE(${ST_CRUISE})"            "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "cmd = 03"                              "$(val g_rdk_cmd)" "$CMD_STOP"
assert_eq "PWM 目标 = 0"                           "$(val g_rdk_pwm_out)" 0
assert_eq "斜坡速度 = 0"                            "$(val g_current_speed)" 0
assert_eq "电机状态 = BRAKE(3)（短路制动）"         "$(val g_state)" "$MOTOR_BRAKE"
assert_eq "IN1=IN2=H（PB0/PB1/PB4/PB5，掩码 0x33 全置位）" "$(maskv GPIOB_ODR 0x33)" "$((0x33))"
if [ "$ENC_LIVE" = 1 ]; then
    assert_range "刹停后残速 ≤ ${STOP_V_MAX} cm/s" "$(val g_rdk_v_cmps)" 0 "$STOP_V_MAX"
else
    warn "编码器未上电：跳过刹停残速判定（g_rdk_v_cmps=$(val g_rdk_v_cmps)）"
fi

# ---------------------------------------------------------------- ⑬ T6 判活铁律
sec "⑬ T6 判活铁律（**核心回归**）：只停 0x100、继续发 0x102 → ≤${HB_TIMEOUT}ms 仍进 HOLD"
feed_stop; feed_start "$CMD_CRUISE" 60 0.1     # 先让车真的在巡航
sleep 0.6
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd
swd_read_all "$DUMP_DIR/t6_before" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_rdk_cmd || exit 2
info "前置：st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) cmd=$(val g_rdk_cmd)"
assert_eq "前置 st = CRUISE(${ST_CRUISE})" "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "前置 PWM = ${CRUISE_PWM}（正在前进）" "$(val g_rdk_pwm_out)" "$CRUISE_PWM"
feed_stop                       # ← 只停 0x100（**唯一判活源**）
ack_start 300 0.08              # ← 0x102 应答照常流（DLC=8、seq 递增）
sleep 0.8                       # ≥ hb_timeout_ms(300) 余量：静默满 0.8s 后判 HOLD（阈值 300ms）
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd g_state g_rdk_hb_ms g_rdk_last_rx_100 g_rdk_last_rx_102 g_ticks
swd_read_all "$DUMP_DIR/t6_timeout" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_state g_rdk_hb_ms g_rdk_last_rx_100 g_rdk_last_rx_102 g_ticks || exit 2
_now=$(val g_ticks)
age100=$(( _now - $(val g_rdk_last_rx_100) ))
age102=$(( _now - $(val g_rdk_last_rx_102) ))
info "st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) motor=$(val g_state) age100=${age100}ms age102=${age102}ms（now=$_now）"
assert_eq "st = HOLD(${ST_HOLD})（0x100 静默 ≥${HB_TIMEOUT}ms ⇒ 本地刹停）" "$(val g_rdk_state)" "$ST_HOLD"
assert_eq "PWM 目标 = 0（本地刹停）"                    "$(val g_rdk_pwm_out)" 0
assert_eq "电机状态 = BRAKE(3)（短路制动保持）"          "$(val g_state)" "$MOTOR_BRAKE"
assert_range "0x100 确实静默了 ≥ ${HB_TIMEOUT}ms（age100）" "$age100" "$HB_TIMEOUT" 600000
assert_range "0x102 仍在流（age102 < ${HB_TIMEOUT}ms）⇒ 它挡不住判活" "$age102" 0 "$HB_TIMEOUT"

# ---------------------------------------------------------------- ⑭ T7 HOLD 恢复
sec "⑭ T7 HOLD 恢复：0x102 不能恢复 / 重复 0x100 不能恢复 / 新 byte3 的有效 0x100 才恢复"
ack_stop
can_sh "cansend $CAN_IF $(hs_ack 5); sleep 0.05; cansend $CAN_IF $(hs_ack 6); sleep 0.05; cansend $CAN_IF $(hs_hb 7); sleep 0.05; cansend $CAN_IF $(hs_ack 8)" >/dev/null
sleep 0.3
swd_build g_rdk_state g_rdk_pwm_out g_rdk_ack_count
swd_read_all "$DUMP_DIR/t7a_ack_only" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_rdk_ack_count || exit 2
info "T7a：st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) ack=$(val g_rdk_ack_count)"
assert_eq "T7a st 仍 HOLD(${ST_HOLD})（0x102 应答/心跳都不能解除 HOLD）" "$(val g_rdk_state)" "$ST_HOLD"
assert_eq "T7a PWM 仍 0"                              "$(val g_rdk_pwm_out)" 0
assert_eq "T7a HOLD 里 0x102 不参与计数（ack_count 仍 2）" "$(val g_rdk_ack_count)" 2
swd_build g_rdk_last_cnt g_rdk_hb_dup
swd_read_all "$DUMP_DIR/t7b_last" || exit 2
check_nums g_rdk_last_cnt g_rdk_hb_dup || exit 2
l=$(val g_rdk_last_cnt); d0=$(val g_rdk_hb_dup)
info "T7b：上一帧有效 0x100 的 byte3 = $l（重发同一帧，必须被去重丢弃）"
can_send "$(hb_frame "$CMD_CRUISE" 20 90 "$l")" >/dev/null
sleep 0.3
swd_build g_rdk_state g_rdk_pwm_out g_rdk_hb_dup
swd_read_all "$DUMP_DIR/t7b_dup" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_rdk_hb_dup || exit 2
info "T7b：st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) dup=$(val g_rdk_hb_dup)"
assert_eq "T7b st 仍 HOLD(${ST_HOLD})（byte3 未变的重复帧不能恢复）" "$(val g_rdk_state)" "$ST_HOLD"
assert_eq "T7b PWM 仍 0"                              "$(val g_rdk_pwm_out)" 0
assert_eq "T7b 重复帧被记账（hb_dup = $d0 + 1）"      "$(val g_rdk_hb_dup)" "$((d0 + 1))"
l2=$(( (l + 1) % 256 ))
can_send "$(hb_frame "$CMD_CRUISE" 20 90 "$l2")" >/dev/null   # 新 byte3 → HOLD 唯一恢复条件
feed_start "$CMD_CRUISE" 60 0.1                     # 恢复后立刻继续喂，否则又要超时
sleep 0.4
swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb_rx
swd_read_all "$DUMP_DIR/t7c_recover" || exit 2
check_nums g_rdk_state g_rdk_pwm_out g_rdk_cmd g_rdk_hb_rx || exit 2
info "T7c：st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) cmd=$(val g_rdk_cmd) rx=$(val g_rdk_hb_rx)"
assert_eq "T7c st = CRUISE(${ST_CRUISE})（新 byte3 的有效 0x100 = 唯一恢复条件）" "$(val g_rdk_state)" "$ST_CRUISE"
assert_eq "T7c PWM 回到 ${CRUISE_PWM}"                 "$(val g_rdk_pwm_out)" "$CRUISE_PWM"
assert_eq "T7c cmd = 01"                              "$(val g_rdk_cmd)" "$CMD_CRUISE"

# ---------------------------------------------------------------- ⑮ T8 --lock
if [ "$LOCKTEST" = 1 ]; then
    sec "⑮ T8（--lock）60s 未握手 → LOCK(3)：PWM=0 + 停发 0x101（需复位恢复）"
    feed_stop; ack_stop
    do_reset || exit 2                     # 回 STATIC 并重置 60s 锁定窗口
    info "静默等待 $((LOCK_S + 4))s（不发任何 0x102，让 60s 窗口走完）..."
    sleep $((LOCK_S + 4))
    swd_build g_rdk_state g_rdk_pwm_out g_rdk_cmd
    swd_read_all "$DUMP_DIR/t8_lock" || exit 2
    check_nums g_rdk_state g_rdk_pwm_out || exit 2
    info "st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) cmd=$(val g_rdk_cmd)"
    assert_eq "st = LOCK(${ST_LOCK})" "$(val g_rdk_state)" "$ST_LOCK"
    assert_eq "PWM = 0（LOCK 保持刹停）" "$(val g_rdk_pwm_out)" 0
    can_capture 1 2 "$DUMP_DIR/t8_no101.dump" "101:7FF"
    n0=$(grep -cE '^[[:space:]]*\(' "$DUMP_DIR/t8_no101.dump" 2>/dev/null || true)
    case "$n0" in ''|*[!0-9]*) n0=0 ;; esac
    assert_eq "LOCK 中 0x101 已停发（2s 抓包 0 帧）" "$n0" 0
else
    sec "⑮ T8（--lock）60s 未握手 → LOCK —— 跳过（默认不跑；加 --lock 启用，约 +65s）"
fi

# ---------------------------------------------------------------- ⑯ 收尾
sec "⑯ 收尾：总线健康（ESR/REC/TEC/Bus-Off）+ 安全停 + RX 溢出"
feed_stop; ack_stop
CNT=$(( (CNT + 1) % 256 ))
can_sh "cansend $CAN_IF $(hb_frame "$CMD_STOP" 0 0 "$CNT")" >/dev/null   # 收尾刹停（HOLD 下被忽略）
sleep 0.3
swd_build g_rdk_state g_rdk_pwm_out CAN1_ESR g_rx_overrun
swd_read_all "$DUMP_DIR/final" || exit 2
check_nums g_rdk_state g_rdk_pwm_out CAN1_ESR g_rx_overrun || exit 2
esr=$(val CAN1_ESR)
ov1=$(val g_rx_overrun)
info "st=$(val g_rdk_state) pwm=$(val g_rdk_pwm_out) ESR=0x$(printf '%X' "$esr") rx_overrun=$ov1"
assert_eq "CAN1_ESR.TEC = 0（无发送错误）" "$(( (esr >> 24) & 0xFF ))" 0
assert_eq "CAN1_ESR.REC = 0（无接收错误）" "$(( (esr >> 8) & 0xFF ))" 0
assert_eq "CAN1_ESR.BOFF = 0（未 Bus-Off）" "$(( (esr >> 2) & 1 ))" 0
assert_eq "PWM = 0（脚本退出前已刹停）" "$(val g_rdk_pwm_out)" 0
sleep 1                      # 静默期：此刻总线上没有 0x100/0x102 输入
swd_build g_rx_overrun
swd_read_all "$DUMP_DIR/final2" || exit 2
check_nums g_rx_overrun || exit 2
ov2=$(val g_rx_overrun)
assert_eq "静默期 RX 溢出计数不再增长（$ov1 → $ov2）" "$ov2" "$ov1"
[ "$ov1" = 0 ] || warn "g_rx_overrun=$ov1 ≠ 0：多为 SWD halt 期间 CAN RX FIFO 灌满（halt 时 CPU 不排空中断），非固件缺陷"

finish
