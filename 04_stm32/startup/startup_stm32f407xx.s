    /* 汇编语法约定：统一语法（unified）/ 目标内核 cortex-m4 / 软浮点 ABI / 生成 Thumb 指令
     * 注意：本文件是**小写 .s**，不经过 C 预处理器 → 注释只能用结尾的 C 风格块注释，不能用井号注释 */
    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

/* ============ 中断向量表（复位后硬件从这里取初始 SP 与复位入口） ============
 * 表项顺序由 ARMv7-M 架构与 ST 的 IRQ 编号固定死：改顺序 = 中断跳错函数。
 * 第 0 项不是函数而是一个值：初始栈顶地址（_estack 来自链接脚本 = RAM 末尾）。 */
.section .isr_vector,"a",%progbits
.word   _estack              /* 初始 MSP：栈从 RAM 末尾向下生长 */
.word   Reset_Handler        /* 复位入口：上电/复位后执行的第一条指令 */
.word   NMI_Handler          /* 不可屏蔽中断 */
.word   HardFault_Handler    /* 硬件错误：数组越界/野指针最终都到这里 */
.word   MemManage_Handler    /* 存储器管理错误（本工程未开 MPU，基本不会进） */
.word   BusFault_Handler     /* 总线错误（访问不存在的地址） */
.word   UsageFault_Handler   /* 用法错误（非法指令/非对齐访问等） */
.word   0                    /* 4~6 号保留：必须占位，否则后面全部错位 */
.word   0
.word   0
.word   0
.word   SVC_Handler          /* 系统服务调用（本工程未用） */
.word   DebugMon_Handler     /* 调试监视器（未用） */
.word   0                    /* 保留位，占位用 */
.word   PendSV_Handler       /* 可挂起系统服务（RTOS 切换上下文用；本工程未用） */
.word   SysTick_Handler      /* 1ms 系统节拍：main.c 里强定义，驱动 millis() */

/* STM32F407ZGT6 外部中断向量：IRQ 0..81（共 82 项），顺序/编号与 ST 官方 stm32f407xx.h IRQn_Type 一致。
 * 修正说明（旧版本缺陷）：此处原按 STM32F1 编号排（把 DMA1_Stream7 放在 18），自 18 起整体错位一项，
 *   例如注释标 "CAN1_RX0=21"，而硬件上 21 实为 CAN1_RX1 —— 启用 CAN 接收中断时必踩，现按官方重排：
 *   18=ADC  19=CAN1_TX  20=CAN1_RX0  21=CAN1_RX1  22=CAN1_SCE  23=EXTI9_5  47=DMA1_Stream7  79=Reserved。
 * 未使用的 IRQ 统一指向 Default_Handler（不再填 0）：填 0 时该中断一旦被误使能，CPU 会跳到
 *   0x00000000 执行垃圾代码；指向 Default_Handler 则干净停在 b Infinite_Loop，便于调试。
 *   79 为 F407 的保留位（无 CRYP），与 ST 官方一致填 0。 */
.word   Default_Handler      /*  0: WWDG */
.word   Default_Handler      /*  1: PVD */
.word   Default_Handler      /*  2: TAMP_STAMP */
.word   Default_Handler      /*  3: RTC_WKUP */
.word   Default_Handler      /*  4: FLASH */
.word   Default_Handler      /*  5: RCC */
.word   Default_Handler      /*  6: EXTI0 */
.word   Default_Handler      /*  7: EXTI1 */
.word   Default_Handler      /*  8: EXTI2 */
.word   Default_Handler      /*  9: EXTI3 */
.word   Default_Handler      /* 10: EXTI4 */
.word   Default_Handler      /* 11: DMA1_Stream0 */
.word   Default_Handler      /* 12: DMA1_Stream1 */
.word   Default_Handler      /* 13: DMA1_Stream2 */
.word   Default_Handler      /* 14: DMA1_Stream3 */
.word   Default_Handler      /* 15: DMA1_Stream4 */
.word   Default_Handler      /* 16: DMA1_Stream5 */
.word   Default_Handler      /* 17: DMA1_Stream6 */
.word   Default_Handler      /* 18: ADC */
.word   Default_Handler      /* 19: CAN1_TX */
.word   Default_Handler      /* 20: CAN1_RX0 */
.word   Default_Handler      /* 21: CAN1_RX1 */
.word   Default_Handler      /* 22: CAN1_SCE */
.word   Default_Handler      /* 23: EXTI9_5 */
.word   Default_Handler      /* 24: TIM1_BRK_TIM9 */
.word   Default_Handler      /* 25: TIM1_UP_TIM10 */
.word   Default_Handler      /* 26: TIM1_TRG_COM_TIM11 */
.word   Default_Handler      /* 27: TIM1_CC */
.word   Default_Handler      /* 28: TIM2 */
.word   Default_Handler      /* 29: TIM3 */
.word   Default_Handler      /* 30: TIM4 */
.word   Default_Handler      /* 31: I2C1_EV */
.word   Default_Handler      /* 32: I2C1_ER */
.word   Default_Handler      /* 33: I2C2_EV */
.word   Default_Handler      /* 34: I2C2_ER */
.word   Default_Handler      /* 35: SPI1 */
.word   Default_Handler      /* 36: SPI2 */
.word   Default_Handler      /* 37: USART1 */
.word   Default_Handler      /* 38: USART2 */
.word   Default_Handler      /* 39: USART3 */
.word   Default_Handler      /* 40: EXTI15_10 */
.word   Default_Handler      /* 41: RTC_Alarm */
.word   Default_Handler      /* 42: OTG_FS_WKUP */
.word   Default_Handler      /* 43: TIM8_BRK_TIM12 */
.word   Default_Handler      /* 44: TIM8_UP_TIM13 */
.word   Default_Handler      /* 45: TIM8_TRG_COM_TIM14 */
.word   Default_Handler      /* 46: TIM8_CC */
.word   Default_Handler      /* 47: DMA1_Stream7 */
.word   Default_Handler      /* 48: FSMC */
.word   Default_Handler      /* 49: SDIO */
.word   Default_Handler      /* 50: TIM5 */
.word   Default_Handler      /* 51: SPI3 */
.word   Default_Handler      /* 52: UART4 */
.word   Default_Handler      /* 53: UART5 */
.word   Default_Handler      /* 54: TIM6_DAC */
.word   Default_Handler      /* 55: TIM7 */
.word   Default_Handler      /* 56: DMA2_Stream0 */
.word   Default_Handler      /* 57: DMA2_Stream1 */
.word   Default_Handler      /* 58: DMA2_Stream2 */
.word   Default_Handler      /* 59: DMA2_Stream3 */
.word   Default_Handler      /* 60: DMA2_Stream4 */
.word   Default_Handler      /* 61: ETH */
.word   Default_Handler      /* 62: ETH_WKUP */
.word   Default_Handler      /* 63: CAN2_TX */
.word   Default_Handler      /* 64: CAN2_RX0 */
.word   Default_Handler      /* 65: CAN2_RX1 */
.word   Default_Handler      /* 66: CAN2_SCE */
.word   Default_Handler      /* 67: OTG_FS */
.word   Default_Handler      /* 68: DMA2_Stream5 */
.word   Default_Handler      /* 69: DMA2_Stream6 */
.word   Default_Handler      /* 70: DMA2_Stream7 */
.word   Default_Handler      /* 71: USART6 */
.word   Default_Handler      /* 72: I2C3_EV */
.word   Default_Handler      /* 73: I2C3_ER */
.word   Default_Handler      /* 74: OTG_HS_EP1_OUT */
.word   Default_Handler      /* 75: OTG_HS_EP1_IN */
.word   Default_Handler      /* 76: OTG_HS_WKUP */
.word   Default_Handler      /* 77: OTG_HS */
.word   Default_Handler      /* 78: DCMI */
.word   0                    /* 79: Reserved（F407 无 CRYP）*/
.word   Default_Handler      /* 80: HASH_RNG */
.word   Default_Handler      /* 81: FPU */

    .section .text
    .thumb_func
    .global Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* 1. 手动装载栈指针（复位时硬件已从向量表第 0 项装载过，这里再显式设一次更稳） */
    ldr r0, =_estack
    mov sp, r0

    /* 使能 FPU（Cortex-M4F） */
    /* CPACR(0xE000ED88) 的 CP10/CP11 各 2 位，写 0b1111 → 允许特权/非特权访问 FPU；
     * 不开这一步，任何浮点指令都会立刻 UsageFault（本工程测速/换算用到 float） */
    ldr r0, =0xE000ED88
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20)
    str r1, [r0]
    dsb                          /* 数据同步屏障：确保写操作完成 */
    isb                          /* 指令同步屏障：清流水线，让后面的代码看到新配置 */

    /* 拷贝 .data 段 */
    /* 有初值的全局变量存在 Flash（_sidata 起），运行时必须在 RAM（_sdata~_edata）里有一份副本 */
    ldr r1, =_sdata              /* r1 = RAM 目标起点 */
    ldr r2, =_edata              /* r2 = RAM 目标终点 */
    ldr r3, =_sidata             /* r3 = Flash 里的初值来源 */
    b LoopCopyDataInit           /* 先跳到循环条件判断（区间可能为空） */
CopyDataInit:
    ldr r4, [r3], #4             /* 取 4 字节初值，源指针后移（后索引寻址） */
    str r4, [r1], #4             /* 存到 RAM，目标指针后移 */
LoopCopyDataInit:
    cmp r1, r2                   /* 到终点了吗 */
    bcc CopyDataInit             /* 还没到 → 继续搬（bcc = 无符号小于） */

    /* 清零 .bss 段 */
    /* 无初值的全局变量按 C 标准必须是 0，启动时把这片 RAM 全写 0 */
    ldr r1, =_sbss               /* r1 = .bss 起点 */
    ldr r2, =_ebss               /* r2 = .bss 终点 */
    movs r3, #0                  /* 要写入的值 = 0 */
    b LoopFillZerobss            /* 先判条件 */
FillZerobss:
    str r3, [r1], #4             /* 写 4 字节 0，指针后移 */
LoopFillZerobss:
    cmp r1, r2                   /* 到终点了吗 */
    bcc FillZerobss              /* 没到 → 继续写 */

    bl main                      /* 跳到 C 语言 main（正常情况永不返回） */
    bx lr                        /* 兜底返回（main 若意外返回则回到启动前状态） */

    /* 弱默认异常处理器（ST 官方写法 .weak + .thumb_set）
     * 说明：早期实现是"多个标签叠在同一地址 + .weak"，只有首个标签被 .thumb_func
     * 标记为 Thumb 函数，其余（HardFault/NMI/...）在向量表里的表项 bit0=0。
     * 按 ARMv7-M，异常向量 bit[0] 必须为 1，否则取异常时 INVSTATE 故障→升级锁死，
     * 即"真出 HardFault 时 CPU 不是干净地停在这里，而是 Lockup"。
     * .thumb_set 会给符号置 Thumb 属性 → 表项地址带 bit0=1。
     * 注：main.c 里强定义 SysTick_Handler，其余均落到 Default_Handler。 */
    .section .text.Default_Handler,"ax",%progbits
    .thumb_func
Default_Handler:
    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler
    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler
Infinite_Loop:
    b Infinite_Loop
    .size Default_Handler, .-Default_Handler
