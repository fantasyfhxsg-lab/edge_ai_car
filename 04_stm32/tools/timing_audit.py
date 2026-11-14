#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""时序常量审计（离线工具，不参与固件构建：Makefile 只编译 Src/*.c）

背景：2026 年 CAN 联调准备期人工复核发现三类"常量与文档不符"问题——它们都
      不会让编译报错（0 警告 0 错误），只在总线/示波器/电机上暴露：
        1) main.c 的 RCC->CFGR 写 (4UL << 10)，注释声称 "APB1=42MHz(÷4)"，但
           PPRE1 的 0b100 是 ÷2（ST 官方 RCC_CFGR_PPRE1_DIV2=0x1000，
           DIV4=0x1400）→ APB1 实为 84MHz → CAN 实跑 1Mbps 而非 500kbps。
           （APB2 用同一编码 4<<13 = ÷2 = 84MHz，与 USART1 115200 实测吻合，
             可作为"PPRE1 也是÷2"的旁证。）
        2) motor_driver.c 把 ±1000 域速度直写 TIM3->CCR1/2，而 ARR=49
           → 任何 ≥5% 指令饱和成满占空比，RAMP_STEP=80 的斜坡同时失效。
        3) 注释里残留历史错误 BRR（0x008B / 0x2D93）等"错饵常量"。
      4) 另有三类"时序不变量"同样固化成检查项：调度层任务函数内出现 delay_ms 阻塞
         （规则 §3 历史事故）、ISR 里写入的共享变量漏 volatile、编码器被配成带时基
         （PSC/ARR 非自由计数 → 测量窗口随总线频率漂移）。
      本脚本把"由源码常量推算的时序"与目标值逐项对比，让同类问题不再靠肉眼。

用法（任意目录均可；脚本按自身位置自动定位工程根，也可用 PROJ_ROOT 覆盖）：
    python3 tools/timing_audit.py
    PROJ_ROOT=/home/fantasy/workspace2/edge_ai_car/04_stm32 python3 tools/timing_audit.py
退出码：0 = 全部通过 / 1 = 有硬性不符(FAIL) / 2 = 源码解析失败（未找到预期常量）
"""
import glob
import os
import re
import sys

# 工程根：PROJ_ROOT 优先（CI/容器可用它指向别处的拷贝），否则由本脚本位置推导
# （tools/timing_audit.py → 上两级即工程根），因此在任何 cwd 下、任何克隆路径都能直接跑。
_ROOT = (os.environ.get('PROJ_ROOT') or
         os.path.dirname(os.path.dirname(os.path.abspath(__file__)))).rstrip('/\\')
if not os.path.isfile(_ROOT + '/Src/main.c'):
    print('[解析失败] 工程根不对（未找到 %s/Src/main.c）：'
          '请用 PROJ_ROOT 指定 04_stm32 目录，例如 '
          'PROJ_ROOT=/path/to/04_stm32 python3 tools/timing_audit.py' % _ROOT)
    sys.exit(2)

MAIN = _ROOT + '/Src/main.c'
CAN = _ROOT + '/Src/can_driver.c'
CANH = _ROOT + '/Inc/can_driver.h'
MOT = _ROOT + '/Src/motor_driver.c'
ENC = _ROOT + '/Src/encoder.c'
ENCH = _ROOT + '/Inc/encoder.h'
PROTO = _ROOT + '/Inc/can_protocol.h'
SRC = _ROOT + '/Src'

HSE_HZ = 8000000                      # 板载 8MHz 晶振（main.c SystemClock_Config）
UART_BAUD = 115200                    # 调试口目标波特率
UART_BRR_ERR_MAX_PCT = 2.5            # USART 异步模式允许的波特率误差
CAN_SAMPLE_PT_RANGE = (70.0, 87.5)    # 采样点合理区间(%)
PWM_TARGET_HZ = 20000                 # TB6612 PWM 目标（"20kHz 无啸叫"）
SERVO_TARGET_HZ = 50                  # MG996R 目标帧率

# PPRE1/PPRE2/HPRE 编码：0xx = ÷1, 100 = ÷2, 101 = ÷4, 110 = ÷8, 111 = ÷16
DIV = {0: 1, 1: 1, 2: 1, 3: 1, 4: 2, 5: 4, 6: 8, 7: 16}

ok, bad, warn = [], [], []


def read(path):
    try:
        with open(path, encoding='utf-8') as f:
            return f.read()
    except OSError as exc:
        print('[解析失败] %s: %s' % (path, exc))
        sys.exit(2)


def grab(src, pattern, what):
    m = re.search(pattern, src)
    if not m:
        print('[解析失败] 未找到 %s\n           正则: %s' % (what, pattern))
        sys.exit(2)
    return m


def shift_pairs(expr):
    """'(8UL << 0) | (336UL << 6)' -> {0: 8, 6: 336}"""
    return {int(sh): int(val) for val, sh in re.findall(r'\((\d+)UL\s*<<\s*(\d+)\)', expr)}


def bare_ul(expr):
    """取不跟随 '<<' 的裸常量，如 BTR 里的 BRP-1=3"""
    out = []
    for m in re.finditer(r'(\d+)UL', expr):
        if not expr[m.end():m.end() + 4].lstrip().startswith('<<'):
            out.append(int(m.group(1)))
    return out


def strip_comments(src):
    """去掉块注释/行注释，避免把注释里的历史常量当成现役常量"""
    src = re.sub(r'/\*.*?\*/', ' ', src, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', src)


def fn_body(src, name):
    """取 'void <name>(void) {...}' 的函数体（花括号计数），找不到返回 None"""
    m = re.search(r'(?m)^[^\n]*\b%s\s*\(\s*void\s*\)\s*\{' % re.escape(name), src)
    if m is None:
        return None
    start, depth = src.index('{', m.end() - 1), 0
    for j in range(start, len(src)):
        if src[j] == '{':
            depth += 1
        elif src[j] == '}':
            depth -= 1
            if depth == 0:
                return src[start + 1:j]
    return None


def fmt(v, unit):
    if unit == 'Hz':
        if abs(v) >= 1e6:
            return '%.6g MHz' % (v / 1e6)
        if abs(v) >= 1e3:
            return '%.6g kHz' % (v / 1e3)
        return '%.6g Hz' % v
    if unit == '%':
        return '%.2f %%' % v
    return '%.3f %s' % (v, unit)


def check(name, actual, target, tol, unit, hard=True, note=''):
    good = abs(actual - target) <= tol
    line = '%-24s 推算 %-14s 目标 %-13s' % (name, fmt(actual, unit), fmt(target, unit))
    (ok if good else (bad if hard else warn)).append(
        '%s %s%s' % ('PASS' if good else ('FAIL' if hard else 'WARN'), line, note))
    return good


def main():
    main_src, can_src = read(MAIN), read(CAN)
    can_h, mot_src = read(CANH), read(MOT)
    # 分层补全后（平台层拆出 Src/platform.c），时钟/串口/SysTick 等常量不再都在
    # main.c：这类"跨文件常量"统一从 Src 下全部 .c 的合并正文里找，
    # 这样搬文件（平台层/驱动层归位）不会让审计报"解析失败"。
    all_src = '\n'.join(read(p) for p in sorted(glob.glob(SRC + '/*.c')))

    # ---------- 1. 时钟树（PLLCFGR: M/N/P/Q + PLLSRC=HSE）----------
    pll = shift_pairs(grab(all_src, r'(?m)^[ \t]*RCC->PLLCFGR\s*=\s*([^;]+);',
                           'PLLCFGR 赋值').group(1))
    m, n, p = pll[0], pll[6], pll[16]
    if not (pll.get(22, 0) & 1):
        print('[解析失败] PLLSRC(bit22) 未选 HSE')
        sys.exit(2)
    vco_out = (HSE_HZ / m) * n
    sysclk = vco_out / (2 * (p + 1))          # P[1:0]: 00=÷2, 01=÷4, 10=÷6, 11=÷8

    # ---------- 2. 总线分频（CFGR: HPRE/PPRE1/PPRE2/SW）----------
    cfgr = shift_pairs(grab(all_src, r'(?m)^[ \t]*RCC->CFGR\s*=\s*([^;]+);',
                            'CFGR 赋值').group(1))
    ahb = sysclk / DIV[cfgr[4]]
    apb1 = ahb / DIV[cfgr[10]]
    apb2 = ahb / DIV[cfgr[13]]
    check('SYSCLK', sysclk, 168e6, 0, 'Hz')
    check('APB2(USART1/TIM1)', apb2, 84e6, 0, 'Hz')
    if abs(apb1 - 42e6) > 1:
        warn.append('WARN APB1 推算 %.3f MHz，但 main.c 注释与 .clinerules §1 声称 42MHz；'
                    'PPRE1 写的是 0b100=÷2（÷4 应为 (5UL << 10) = 0x1400）' % (apb1 / 1e6))
    else:
        ok.append('PASS APB1 42.000 MHz 与文档声明一致')

    # ---------- 3. CAN 位时序（BTR + 波特率 + 采样点）----------
    can_bps = float(grab(can_h, r'#define\s+CAN_BITRATE_BPS\s+(\d+)u',
                         'CAN_BITRATE_BPS').group(1))
    btr_expr = grab(can_src, r'uint32 btr\s*=\s*([^;]+);', 'CAN BTR 表达式').group(1)
    btr, brp_m1 = shift_pairs(btr_expr), bare_ul(btr_expr)
    if len(brp_m1) != 1:
        print('[解析失败] BTR 表达式里 BRP-1 常量不唯一: %s' % btr_expr)
        sys.exit(2)
    brp, ts1, ts2, sjw = brp_m1[0] + 1, btr[16] + 1, btr[20] + 1, btr[24] + 1
    if not re.search(r'btr\s*\|=\s*CAN_BTR_LBKM\s*\|\s*CAN_BTR_SILM', can_src):
        print('[解析失败] 未找到回环设置 btr |= CAN_BTR_LBKM | CAN_BTR_SILM')
        sys.exit(2)
    can_baud = apb1 / (brp * (1 + ts1 + ts2))
    sample = 100.0 * (1 + ts1) / (1 + ts1 + ts2)
    check('CAN 位速率(APB1=%dM)' % round(apb1 / 1e6), can_baud, can_bps,
          can_bps * 0.001, 'Hz', note='  BRP=%d TS1=%d TS2=%d SJW=%d' % (brp, ts1, ts2, sjw))
    s_lo, s_hi = CAN_SAMPLE_PT_RANGE
    if s_lo <= sample <= s_hi:
        ok.append('PASS CAN 采样点 %.2f%% 落在 %.1f~%.1f%%' % (sample, s_lo, s_hi))
    else:
        bad.append('FAIL CAN 采样点 %.2f%% 超出 %.1f~%.1f%%' % (sample, s_lo, s_hi))

    # ---------- 4. USART1 波特率（BRR = 尾数<<4 | 分数/16）----------
    brr = int(grab(all_src, r'(?m)^[ \t]*USART1->BRR\s*=\s*(0x[0-9A-Fa-f]+)\s*;',
                   'USART1 BRR 赋值').group(1), 16)
    uart_baud = apb2 / (16.0 * ((brr >> 4) + (brr & 0xF) / 16.0))
    err = abs(uart_baud - UART_BAUD) / UART_BAUD * 100.0
    if err <= UART_BRR_ERR_MAX_PCT:
        ok.append('PASS USART1 BRR=0x%04X -> %.1f baud，误差 %.3f%% (<= %.1f%%)'
                  % (brr, uart_baud, err, UART_BRR_ERR_MAX_PCT))
    else:
        bad.append('FAIL USART1 BRR=0x%04X -> %.1f baud，误差 %.3f%% > %.1f%%'
                   % (brr, uart_baud, err, UART_BRR_ERR_MAX_PCT))
    for dead in re.findall(r'(?m)^\s*//+\s*USART1->BRR\s*=\s*(0x[0-9A-Fa-f]+)', all_src):
        warn.append('WARN 源码注释里残留历史 BRR %s（非当前值，易被误启用）' % dead)

    # ---------- 5. TIM3 PWM 频率（APB1 分频≠1 → 定时器时钟 = 2×APB1）----------
    t3_psc = int(grab(all_src, r'(?m)^[ \t]*TIM3->PSC\s*=\s*(\d+)\s*;', 'TIM3 PSC').group(1))
    t3_arr = int(grab(all_src, r'(?m)^[ \t]*TIM3->ARR\s*=\s*(\d+)\s*;', 'TIM3 ARR').group(1))
    tim_apb1 = apb1 * (2 if DIV[cfgr[10]] != 1 else 1)
    check('TIM3 PWM(ARR=%d)' % t3_arr, tim_apb1 / ((t3_psc + 1) * (t3_arr + 1)),
          PWM_TARGET_HZ, PWM_TARGET_HZ * 0.05, 'Hz', hard=False, note='  注释声称 20kHz')

    # ---------- 6. 速度域 ↔ CCR 域契约（motor_driver）----------
    clamp = int(grab(mot_src, r'if\s*\(\s*speed\s*>\s*(\d+)\s*\)',
                     'Motor_SetSpeed 上限钳位').group(1))
    ramp = int(grab(mot_src, r'#define\s+RAMP_STEP\s+(\d+)', 'RAMP_STEP').group(1))
    if clamp > t3_arr and 'TIM3->ARR' not in mot_src:
        bad.append('FAIL motor_driver 把 ±%d 域速度直写 CCR1/2（ARR=%d，>=%d 即满占空比），'
                   '未按 ARR 换算；RAMP_STEP=%d 随之失效' % (clamp, t3_arr, t3_arr + 1, ramp))
    else:
        ok.append('PASS motor_driver 速度域(±%d) 与 CCR 域(0..%d) 契约一致' % (clamp, t3_arr + 1))

    # ---------- 7. TIM1 舵机 50Hz ----------
    t1_psc = int(grab(all_src, r'(?m)^[ \t]*TIM1->PSC\s*=\s*(\d+)\s*;', 'TIM1 PSC').group(1))
    t1_arr = int(grab(all_src, r'(?m)^[ \t]*TIM1->ARR\s*=\s*(\d+)\s*;', 'TIM1 ARR').group(1))
    tim_apb2 = apb2 * (2 if DIV[cfgr[13]] != 1 else 1)
    check('TIM1 舵机', tim_apb2 / ((t1_psc + 1) * (t1_arr + 1)), SERVO_TARGET_HZ, 0.5, 'Hz')

    # ---------- 8. SysTick 1ms ----------
    rvr = int(grab(all_src, r'(?m)^[ \t]*SysTick->RVR\s*=\s*(\d+)UL\s*-\s*1UL\s*;',
                   'SysTick RVR').group(1))
    check('SysTick 周期', (rvr + 1) / sysclk * 1000.0, 1.0, 0.001, 'ms')
    if 'SYSTICK_CSR_CLKSOURCE' not in all_src:
        warn.append('WARN SysTick 未显式选内核时钟(CLKSOURCE)，1ms 前提不成立')

    # ---------- 9. 调度层：任务函数内禁止阻塞延时（规则 §3）----------
    task_names = re.findall(r'(?m)^\s*(?:static\s+)?void\s+(Task_\w+)\s*\(\s*void\s*\)\s*\{', main_src)
    if not task_names:
        print('[解析失败] main.c 未找到 Task_xxx(void) 任务函数')
        sys.exit(2)
    blocked = [t for t in task_names if 'delay_ms(' in strip_comments(fn_body(main_src, t) or '')]
    if blocked:
        bad.append('FAIL 任务函数 %s 内出现 delay_ms 阻塞（会挤掉 10ms 轮询 → CAN 丢帧，规则 §3）'
                   % '/'.join(blocked))
    else:
        ok.append('PASS 调度层 %s 内无 delay_ms 阻塞' % '/'.join(task_names))

    # ---------- 10. 编码器：自由计数（无时基） + 测速窗口契约 ----------
    enc_src, enc_h = read(ENC), read(ENCH)
    for tim in ('TIM4', 'TIM8'):
        psc = int(grab(enc_src, r'(?m)^[ \t]*%s->PSC\s*=\s*(\d+)\s*;' % tim, '%s PSC' % tim).group(1))
        arr = int(grab(enc_src, r'(?m)^[ \t]*%s->ARR\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*;' % tim,
                       '%s ARR' % tim).group(1), 0)
        if psc == 0 and arr == 0xFFFF:
            ok.append('PASS %s 正交自由计数（PSC=0, ARR=0xFFFF，不随 APB1 频率漂移）' % tim)
        else:
            bad.append('FAIL %s 被配成 PSC=%d ARR=0x%X：编码器模式不该带时基' % (tim, psc, arr))
    window = int(grab(enc_h, r'#define\s+ENCODER_CONTROL_PERIOD_MS\s+(\d+)u',
                      'ENCODER_CONTROL_PERIOD_MS').group(1))
    if any('Encoder_GetSpeed' in strip_comments(fn_body(main_src, t) or '') for t in task_names):
        ok.append('PASS 测速窗口 %d ms 已挂到调度任务（0x125 按此窗口换算）' % window)
    else:
        warn.append('WARN 0x125 未接入：Encoder_GetSpeed*() 必须在周期=ENCODER_CONTROL_PERIOD_MS'
                    '(%d ms) 的任务里调用，否则 cm/s→km/h 按比例错' % window)

    # ---------- 11. ISR 写入的共享变量必须 volatile（规则 §3）----------
    for isr in re.findall(r'(?m)^\s*void\s+(\w+_Handler)\s*\(\s*void\s*\)', all_src):
        body = strip_comments(fn_body(all_src, isr) or '')
        for ident in sorted(set(re.findall(r'\b(g_\w+)\s*(?:\+\+|--|\+=|=|\(\s*\))', body))):
            decl = re.search(r'(?m)^[^\n]*\b%s\b[^\n]*;' % ident, all_src)
            if decl is not None and 'volatile' not in decl.group(0):
                bad.append('FAIL %s 写入的共享变量 %s 未声明 volatile' % (isr, ident))
            else:
                ok.append('PASS %s 写入的 %s 已声明 volatile' % (isr, ident))

    # ---------- 12. 协议头：ID 与波特率声明一致性（规则 §5）----------
    proto = read(PROTO)
    ids = [i.lower() for i in re.findall(r'#define\s+CAN_ID_\w+\s+(0x[0-9A-Fa-f]+)', proto)]
    miss = [x for x in ('0x123', '0x124', '0x125') if x not in ids]
    if miss:
        warn.append('WARN can_protocol.h 缺 §5 的 %s（现为: %s）——按 §5 对齐（禁止单边改动）'
                    % (','.join(miss), ' '.join(ids)))
    else:
        ok.append('PASS can_protocol.h 已含 §5 的 0x123/0x124/0x125')
    proto_bps = float(grab(proto, r'#define\s+CAN_BITRATE\s+(\d+)',
                           'can_protocol.h CAN_BITRATE').group(1))
    if abs(proto_bps - can_bps) > 1:
        bad.append('FAIL can_protocol.h CAN_BITRATE=%.0f 与 can_driver.h CAN_BITRATE_BPS=%.0f 不一致'
                   % (proto_bps, can_bps))
    else:
        ok.append('PASS 两份头文件波特率声明一致（%.0f bps）' % proto_bps)
    code = strip_comments(all_src)
    if '0x125' in code and '0.036' not in code:
        bad.append('FAIL 0x125 车速上报已接但未见 §5 换算常数 0.036（cm/s → km/h）')

    print('=== 时序常量审计（源码常量 -> 推算值 -> 目标值）===')
    for line in ok + warn + bad:
        print('  ' + line)
    print('--- PASS %d / WARN %d / FAIL %d ---' % (len(ok), len(warn), len(bad)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
