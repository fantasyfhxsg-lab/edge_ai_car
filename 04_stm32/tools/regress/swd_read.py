#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""swd_read.py —— SWD 读回的"计划 + 解析"器（tools/regress 公共库，被 lib.sh 调用）

为什么要有它：
  openocd 的 `mdw` 只认「地址 + 字数」。如果脚本里硬写地址和宽度，代码一改就漂
  （本仓库踩过的坑：地址平移、宽度写错串到相邻字段）。本脚本改用 `nm -S` **从当前
  elf 现读** 每个符号的地址与宽度，自动算出一个 mdw 覆盖区间；读回后再按同样宽度
  还原，所以三个复现脚本里**没有任何硬编码的 g_* 地址/宽度**。

子命令：
  plan <名字...>                -> 打印 openocd 需要的若干行 "mdw <base> <words>"
  emit --dump FILE <名字...>    -> 打印 NAME=<无符号十进制>（4 字节再给 NAME.s32=<有符号>）

"名字" 可以是：① elf 里的全局符号（如 g_brake_latched）；② 内置寄存器别名
（RCC_CFGR / CAN1_BTR / GPIOB_ODR ...，地址照抄 Core/Inc/stm32f407_regs.h 的
 基址 + 结构体偏移，不臆造数值）。

退出码：0 正常；2 参数/elf 错误；3 emit 时有名字取不到（脚本据此判"镜像不对/缺符号"）。
"""
import argparse
import os
import re
import subprocess
import sys

NM = os.environ.get('NM', 'arm-none-eabi-nm')

# ---- 内置寄存器别名（只读观测用；地址 = 基址 + 结构体偏移，来源见文件头注释）----
REG = {
    # RCC   @ AHB1PERIPH_BASE(0x40020000)+0x3800，字段序 CR(0x00) PLLCFGR(0x04) CFGR(0x08)
    'RCC_CR':      0x40023800,
    'RCC_PLLCFGR': 0x40023804,
    'RCC_CFGR':    0x40023808,
    # GPIOB @ AHB1PERIPH_BASE+0x0400，字段序 MODER OTYPER OSPEEDR PUPDR IDR ODR -> ODR=+0x14
    #        电机/刹车引脚：AIN1=PB0 AIN2=PB1 STBY=PB3 BIN1=PB4 BIN2=PB5（motor_driver.h）
    'GPIOB_ODR':   0x40020414,
    # CAN1  @ APB1PERIPH_BASE(0x40000000)+0x6400：MCR 0x00 MSR 0x04 ESR 0x18 BTR 0x1C
    #        （can_driver.c 有 _Static_assert(offsetof(CAN_TypeDef,BTR)==0x01C) 兜底）
    'CAN1_MCR':    0x40006400,
    'CAN1_MSR':    0x40006404,
    'CAN1_ESR':    0x40006418,
    'CAN1_BTR':    0x4000641C,
    # TIM3 @ APB1PERIPH_BASE(0x40000000)+0x0400：字段序 CR1(0x00) CR2(0x04) SMCR(0x08) DIER(0x0C)
    #      SR(0x10) EGR(0x14) CCMR1(0x18) CCMR2(0x1C) CCER(0x20) CNT(0x24) PSC(0x28) ARR(0x2C)
    #      RESERVED0(0x30) CCR1(0x34) CCR2(0x38)（Core/Inc/stm32f407_regs.h 的 TIM_TypeDef）
    #      电机 PWM：ARR=49 → 20kHz；CCR 域 0..50 对应 0..100% 占空比（motor_driver.c）
    'TIM3_ARR':    0x4000042C,
    'TIM3_CCR1':   0x40000434,
    'TIM3_CCR2':   0x40000438,
    # TIM4/TIM8 编码器计数：TIM4_BASE = APB1(0x40000000)+0x0800、TIM8_BASE = APB2(0x40010000)+0x0400，
    #     CNT = +0x24（TIM_TypeDef/TIM_AdvTypeDef 前 10 个字段一致，见 Core/Inc/stm32f407_regs.h）
    #     左轮 TIM4（PD12/13）、右轮 TIM8（PC6/7），20ms 采样窗内累计脉冲（encoder.c）
    'TIM4_CNT':    0x40000824,
    'TIM8_CNT':    0x40010424,
}
REG_WIDTH = 4

MAX_WORDS = 512          # 单块 dump 上限（防符号表白算出一个巨大的区间）
LINE_RE = re.compile(r'^0x([0-9a-fA-F]{8}):\s+(.*)$')


def symtab(elf):
    """返回 {name: (addr, size)}；只认数据类符号（b/B/d/D）。"""
    try:
        out = subprocess.run([NM, '-S', '-n', elf], capture_output=True, text=True, check=True).stdout
    except FileNotFoundError:
        sys.exit('找不到 %s（装 arm-none-eabi-binutils，或用 NM= 指定）' % NM)
    except subprocess.CalledProcessError as e:
        sys.exit('nm 读取 %s 失败：%s' % (elf, e.stderr.strip()))
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) != 4:
            continue
        addr, size, typ, name = p
        if typ not in 'bBdD':
            continue
        try:
            a = int(addr, 16)
            s = int(size, 16)
        except ValueError:
            continue
        if a == 0:
            continue
        if s == 0:                      # 少数符号没尺寸信息：按字宽兜底并提示
            sys.stderr.write('[swd_read] 注意：%s 无尺寸信息，按 4 字节处理\n' % name)
            s = 4
        syms[name] = (a, s)
    return syms


def lookup(syms, names):
    """名字 → (addr, size)；返回 (命中列表, 未命中列表)。"""
    hit, miss = [], []
    for n in names:
        if n in syms:
            hit.append((n,) + syms[n])
        elif n in REG:
            hit.append((n, REG[n], REG_WIDTH))
        else:
            miss.append(n)
    return hit, miss


def cmd_plan(syms, names):
    """把要读的名字归并成尽量少的 mdw 块（openocd 的 mdw 越少，会话越快越稳）。"""
    hit, miss = lookup(syms, names)
    for n in miss:
        sys.stderr.write('[swd_read] 计划里跳过：%s 不在 elf 也不在寄存器表\n' % n)
    rng = sorted((a, a + s) for _n, a, s in hit)
    if not rng:
        return 1

    lo = min(a for a, _b in rng) & ~0x3      # mdw 要求 4 字节对齐
    hi = max(b for _a, b in rng)
    if (hi - lo + 3) // 4 <= MAX_WORDS:
        blocks = [(lo, (hi - lo + 3) // 4)]  # 跨度不大：整段一次读完（RAM 符号通常就落这里）
    else:
        blocks = []                          # 跨度太大：按"间隔 ≤ 16 字节"聚类，逐块读
        for a, b in rng:
            base = a & ~0x3
            if blocks and base - blocks[-1][1] <= 16:
                blocks[-1][1] = max(blocks[-1][1], b)
            else:
                blocks.append([base, b])
        capped = []
        for base, end in blocks:
            words = (end - base + 3) // 4
            if words > MAX_WORDS:
                sys.stderr.write('[swd_read] 块 0x%08X 过大（%d 字），截断到 %d 字\n'
                                 % (base, words, MAX_WORDS))
                words = MAX_WORDS
            capped.append((base, words))
        blocks = capped

    for base, words in blocks:
        print('0x%08X %d' % (base, words))
    return 0


def read_dump(path):
    """openocd dump → {byte_addr: byte}"""
    mem = {}
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = LINE_RE.match(line.strip())
            if not m:
                continue
            addr, rest = int(m.group(1), 16), m.group(2).split()
            for i, w in enumerate(rest):
                if len(w) > 8 or not re.fullmatch(r'[0-9a-fA-F]+', w):
                    continue
                v = int(w, 16)
                for b in range(4):                 # mdw = 每段 4 字节，小端
                    mem[addr + i * 4 + b] = (v >> (8 * b)) & 0xFF
    return mem


def cmd_emit(syms, names, dump):
    mem = read_dump(dump)
    hit, miss = lookup(syms, names)
    missing = list(miss)
    for name, addr, size in hit:
        want = [addr + i for i in range(size)]
        if not all(a in mem for a in want):
            missing.append(name)
            print('%s=NA' % name)
            continue
        u = int.from_bytes(bytes(mem[a] for a in want), 'little')
        print('%s=%d' % (name, u))
        print('%s.h=0x%X' % (name, u))
        if size == 4:
            s = u - (1 << 32) if u >= (1 << 31) else u
            print('%s.s32=%d' % (name, s))
    if missing:
        sys.stderr.write('[swd_read] 取不到：%s（多半是镜像不含该符号，或地址不在 dump 范围）\n'
                         % ' '.join(missing))
        return 3
    return 0


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('--elf', required=True)
    ap.add_argument('--dump')
    ap.add_argument('cmd', choices=['plan', 'emit'])
    ap.add_argument('names', nargs='*')
    a = ap.parse_args()
    if not os.path.isfile(a.elf):
        sys.exit('找不到 elf：%s（先 make）' % a.elf)
    syms = symtab(a.elf)
    if a.cmd == 'plan':
        return cmd_plan(syms, a.names)
    if not a.dump:
        sys.exit('emit 需要 --dump')
    return cmd_emit(syms, a.names, a.dump)


if __name__ == '__main__':
    sys.exit(main())
