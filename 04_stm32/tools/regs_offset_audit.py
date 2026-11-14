#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""寄存器结构体偏移审计（离线工具，不参与固件构建，Makefile 只编译 Src/*.c）

背景：2026 年 CAN 联调时发现 Core/Inc/stm32f407_regs.h 的 CAN_TypeDef 邮箱区
      少了 0x100（RESERVED0[24] 应为 [88]），导致 TX/RX 邮箱全落在保留地址上：
      编译 0 警告、Can_Send 还"成功"，但回环/总线上永远收不到帧。为避免同类
      "隐性错位"再次发生，本脚本把本项目结构体逐字段偏移与 ST 官方头文件
      stm32f407xx.h 里注释的 Address offset 对齐校验。

用法（任意目录均可；脚本按自身位置自动定位工程根，也可用 PROJ_ROOT 覆盖）：
  1) 取官方头文件（1.1MB）：
     curl -sL -o /tmp/stm32f407xx.h \\
       https://raw.githubusercontent.com/STMicroelectronics/cmsis_device_f4/master/Include/stm32f407xx.h
  2) 生成校验源：
     python3 tools/regs_offset_audit.py /tmp/stm32f407xx.h      # 输出 /tmp/probe_off.c
  3) 交叉编译器校验（0 错误 = 全部字段偏移与 ST 官方一致）：
     arm-none-eabi-gcc -I Core/Inc -std=gnu99 -Wall -Wextra -fsyntax-only /tmp/probe_off.c

说明：本项目只用了 GPIO/RCC/TIM/USART/CAN/SYSCFG/SysTick，官方有而本项目未定义
      且未使用的字段（RCC.SSCGR/PLLI2SCFGR、TIM.OR 等）自动跳过。
"""
import os
import re
import sys

# 工程根：PROJ_ROOT 优先，否则由本脚本位置推导（tools/regs_offset_audit.py → 上两级即工程根），
# 因此在任何 cwd / 任何克隆路径都能直接跑。
_ROOT = (os.environ.get('PROJ_ROOT') or
         os.path.dirname(os.path.dirname(os.path.abspath(__file__)))).rstrip('/\\')
OURS = _ROOT + '/Core/Inc/stm32f407_regs.h'
if not os.path.isfile(OURS):
    print('[解析失败] 工程根不对（未找到 %s）：请用 PROJ_ROOT 指定 04_stm32 目录' % OURS)
    sys.exit(2)
OUT = '/tmp/probe_off.c'

# 官方结构体名 -> 本项目结构体名（本项目把 TIM 拆成基础/高级两个）
STRUCT_MAP = [
    ('GPIO_TypeDef', 'GPIO_TypeDef'),
    ('RCC_TypeDef', 'RCC_TypeDef'),
    ('TIM_TypeDef', 'TIM_TypeDef'),
    ('USART_TypeDef', 'USART_TypeDef'),
    ('SYSCFG_TypeDef', 'SYSCFG_TypeDef'),
    ('CAN_TypeDef', 'CAN_TypeDef'),
]
TIM_ADV = {'RCR', 'BDTR', 'DCR', 'DMAR'}   # 仅高级定时器 TIM1/TIM8 有
# 官方是结构体数组，本项目展平命名：数组元素 0 的首个寄存器
CAN_ARR = {'sTxMailBox': 'TX0R', 'sFIFOMailBox': 'RX0R', 'sFilterRegister': 'F0R1'}


def st_fields(src, name):
    """从 ST 官方头文件里取出某个结构体的 (字段, 偏移) 列表"""
    end = src.index('} %s;' % name)
    start = src.rindex('typedef struct', 0, end)
    out = []
    for line in src[start:end].splitlines():
        m = re.search(r'Address offset:\s*(0x[0-9A-Fa-f]+)', line)
        if not m:
            continue
        fld = line.split('/*')[0].strip().rstrip(';')
        for qual in ('__IO', '__I', '__O'):
            fld = fld.replace(qual, ' ')
        fld = ' '.join(fld.split()).split()
        if len(fld) == 2:                  # 只认 "类型 字段名" 形式
            out.append((fld[1], m.group(1)))
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    src = open(sys.argv[1], encoding='utf-8', errors='replace').read()
    ours = open(OURS, encoding='utf-8').read()

    def defined(f):
        return re.search(r'\b' + re.escape(f) + r'\s*;', ours) is not None

    lines = ['#include "stm32f407_regs.h"', '#include <stddef.h>']
    n, skipped = 0, []
    for st, base in STRUCT_MAP:
        for fld, off in st_fields(src, st):
            name = fld.split('[')[0]
            cur = 'TIM_AdvTypeDef' if name in TIM_ADV else base
            if name in CAN_ARR:             # 官方结构体数组 -> 本项目 TX0R/RX0R/F0R1
                tgt = CAN_ARR[name]
                lines.append('_Static_assert(offsetof(%s, %s)==%s, "%s.%s");'
                             % (cur, tgt, off, cur, tgt))
                n += 1
                continue
            if name == 'AFR':               # 官方 AFR[2] -> 本项目 AFRL/AFRH
                lines.append('_Static_assert(offsetof(%s, AFRL)==0x20, "AFRL");' % cur)
                lines.append('_Static_assert(offsetof(%s, AFRH)==0x24, "AFRH");' % cur)
                n += 2
                continue
            if name == 'EXTICR':            # 官方 EXTICR[4] -> 本项目 4 个独立字段
                for k in range(4):
                    lines.append('_Static_assert(offsetof(%s, EXTICR%d)==0x%02X, "EXTICR%d");'
                                 % (cur, k + 1, 0x08 + 4 * k, k + 1))
                    n += 1
                continue
            if not defined(name):
                skipped.append('%s.%s' % (st, name))
                continue
            lines.append('_Static_assert(offsetof(%s, %s)==%s, "%s.%s");'
                         % (cur, name, off, cur, name))
            n += 1
    # SysTick 属 Cortex-M4 内核（core_cm4.h），偏移固定
    for fld, off in (('CSR', '0x00'), ('RVR', '0x04'), ('CVR', '0x08'), ('CALIB', '0x0C')):
        lines.append('_Static_assert(offsetof(SysTick_TypeDef, %s)==%s, "SysTick.%s");'
                     % (fld, off, fld))
        n += 1

    open(OUT, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('[审计] 已生成 %s （%d 条断言）' % (OUT, n))
    if skipped:
        print('[审计] 本项目未定义/未使用而跳过：%s' % ', '.join(skipped))
    print('[审计] 下一步：arm-none-eabi-gcc -I Core/Inc -std=gnu99 -fsyntax-only %s' % OUT)
    return 0


if __name__ == '__main__':
    sys.exit(main())
