/**
 * stm32f407_regs.h - STM32F407 寄存器定义（自包含，免 HAL/CubeMX）
 *
 * ═══════════ 读这个文件前先懂两件事 ═══════════
 * 1. Cortex-M4 外设就是"内存映射的寄存器"：每个外设占用一段地址，
 *    用结构体指针（如 GPIOA）访问其寄存器，写特定 bit 就是配置。
 *    地址查 RM0090 的 "memory map" 章节，本文件只抄了用到的：
 *       AHB1:  GPIOA~G (0x40020000~0x40021800)、RCC(0x40023800)
 *       APB1:  TIM2/3/4/5、USART2 等（42MHz 域）
 *       APB2:  TIM1/8、USART1（84MHz 域）
 *       CAN1:  APB1 + 0x6400
 * 2. 为什么要自己定义？CubeMX/HAL 会带来几百个文件；
 *    这个工程只用 6 个外设（GPIO/RCC/TIM1/3/4/8/CAN1/USART1），
 *    手写 400 行寄存器宏就够，WSL 里 make 直出，无需 Windows 工具。
 *
 * 依据：RM0090 Reference Manual
 * 说明：只定义本工程用到的外设寄存器
 */
#ifndef STM32F407_REGS_H
#define STM32F407_REGS_H

#include <stdint.h>

/* ================= 地址基址 ================= */
/* 地址层级：PERIPH_BASE 下按总线分 APB1/APB2/AHB1 三个 64KB 窗口，外设再在窗口内偏移。
 * 这些常量都由手册 memory map 抄来，改动等于改硬件事实 —— 不要动。 */
#define FLASH_BASE        0x08000000UL   /* Flash 起始（链接脚本 ORIGIN(FLASH)，也是向量表地址） */
#define SRAM_BASE         0x20000000UL   /* SRAM 起始（.data/.bss/栈都在这里） */
#define PERIPH_BASE       0x40000000UL   /* 外设区总起点 */
#define APB1PERIPH_BASE   (PERIPH_BASE + 0x00000000UL)   /* APB1 域（42MHz）：TIM2/3/4/5、CAN1 */
#define APB2PERIPH_BASE   (PERIPH_BASE + 0x00010000UL)   /* APB2 域（84MHz）：TIM1/8、USART1 */
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)   /* AHB1 域（168MHz）：GPIOA~G、RCC */
#define GPIOA_BASE        (AHB1PERIPH_BASE + 0x0000UL)   /* GPIOA：每个端口占 0x400 */
#define GPIOB_BASE        (AHB1PERIPH_BASE + 0x0400UL)   /* GPIOB */
#define GPIOC_BASE        (AHB1PERIPH_BASE + 0x0800UL)   /* GPIOC */
#define GPIOD_BASE        (AHB1PERIPH_BASE + 0x0C00UL)   /* GPIOD */
#define GPIOE_BASE        (AHB1PERIPH_BASE + 0x1000UL)   /* GPIOE */
#define GPIOF_BASE        (AHB1PERIPH_BASE + 0x1400UL)   /* GPIOF（LED0=PF9 / LED1=PF10 在这里） */
#define GPIOG_BASE        (AHB1PERIPH_BASE + 0x1800UL)   /* GPIOG */
#define RCC_BASE          (AHB1PERIPH_BASE + 0x3800UL)   /* RCC 时钟/复位控制器 */
#define TIM2_BASE         (APB1PERIPH_BASE + 0x0000UL)   /* TIM2（本项目未用） */
#define TIM3_BASE         (APB1PERIPH_BASE + 0x0400UL)   /* TIM3：电机 PWM（PA6/PA7） */
#define TIM4_BASE         (APB1PERIPH_BASE + 0x0800UL)   /* TIM4：左轮编码器（PD12/PD13） */
#define TIM5_BASE         (APB1PERIPH_BASE + 0x0C00UL)   /* TIM5（本项目未用） */
#define USART1_BASE       (APB2PERIPH_BASE + 0x1000UL)   /* USART1：调试串口（PA9/PA10） */
#define TIM1_BASE         (APB2PERIPH_BASE + 0x0000UL)   /* TIM1：舵机 PWM（PA8，高级定时器） */
#define TIM8_BASE         (APB2PERIPH_BASE + 0x0400UL)   /* TIM8：右轮编码器（PC6/PC7，高级定时器） */
#define CAN1_BASE         (APB1PERIPH_BASE + 0x6400UL)   /* CAN1：AEB 通信（PA11/PA12） */

/* ================= GPIO ================= */
/* GPIO 引脚掩码：第 n 位对应 PIN_n，可用 | 组合多个引脚传给 gpio_mode 等助手 */
#define GPIO_PIN_0  0x0001U
#define GPIO_PIN_1  0x0002U
#define GPIO_PIN_2  0x0004U
#define GPIO_PIN_3  0x0008U
#define GPIO_PIN_4  0x0010U
#define GPIO_PIN_5  0x0020U
#define GPIO_PIN_6  0x0040U
#define GPIO_PIN_7  0x0080U
#define GPIO_PIN_8  0x0100U
#define GPIO_PIN_9  0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U

/* MODER 每脚 2 位的四种取值（与 gpio_mode 的 mode 参数直接对应） */
#define GPIO_MODE_INPUT  0x00U   /* 输入：读外部电平 */
#define GPIO_MODE_OUTPUT 0x01U   /* 通用推挽输出：LED/方向脚 */
#define GPIO_MODE_AF     0x02U   /* 复用功能：交给 TIM/USART/CAN 等外设驱动 */
#define GPIO_MODE_ANALOG 0x03U   /* 模拟：ADC/低功耗（本项目未用） */

// 新增：__IO 宏定义，CMSIS标准
/* 这三个限定符是 CMSIS 惯例写法：__IO = 可读写且可能被硬件异步改变 → 必须 volatile，
 * 否则编译器会把"反复读同一寄存器"优化成只读一次，导致轮询等状态永远等不到 */
#define __IO volatile
#define __I  volatile const    /* 只读寄存器（写它无意义） */
#define __O  volatile          /* 只写寄存器 */

typedef struct {
    volatile uint32_t MODER;      /* 0x00 端口模式：每脚 2 位（输入/输出/复用/模拟） */
    volatile uint32_t OTYPER;     /* 0x04 输出类型：0 推挽（本项目全用推挽）/ 1 开漏 */
    volatile uint32_t OSPEEDR;    /* 0x08 输出速度：每脚 2 位，低速已够驱动 LED/TB6612 */
    volatile uint32_t PUPDR;      /* 0x0C 上下拉：每脚 2 位（00 无 / 01 上拉 / 10 下拉） */
    volatile uint32_t IDR;        /* 0x10 输入数据：读引脚实际电平（gpio_read 用） */
    volatile uint32_t ODR;        /* 0x14 输出数据：读-改-写方式改电平（非原子，本工程用 BSRR 代替） */
    volatile uint32_t BSRR;       /* 0x18 置位/复位：低 16 位置 1、高 16 位清 0，单次写即原子生效 */
    volatile uint32_t LCKR;       /* 0x1C 配置锁定：锁住引脚配置防误改（本工程未用） */
    volatile uint32_t AFRL;       /* 0x20 复用功能低：引脚 0~7 各 4 位 AF 编号 */
    volatile uint32_t AFRH;       /* 0x24 复用功能高：引脚 8~15 各 4 位 AF 编号 */
} GPIO_TypeDef;

/* 端口指针：把基地址强制转成结构体指针，之后就能用 GPIOA->BSRR 这种写法访问寄存器 */
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF ((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG ((GPIO_TypeDef *)GPIOG_BASE)

/* ================= RCC ================= */
/*RCC = Reset and Clock Control，复位和时钟控制外设，STM32 管理所有外设时钟、复位的外设 */
typedef struct {
    volatile uint32_t CR;            /* 0x00 时钟控制：HSEON/PLLON/HSERDY/PLLRDY 等开关与就绪位 */
    volatile uint32_t PLLCFGR;       /* 0x04 PLL 配置：M/N/P/Q 分频与源选择（本工程配成 168MHz） */
    volatile uint32_t CFGR;          /* 0x08 时钟配置：SW/SWS 时钟源 + AHB/APB1/APB2 分频 */
    volatile uint32_t CIR;           /* 0x0C 时钟中断：就绪中断标志与使能（轮询方式，本工程未用） */
    volatile uint32_t AHB1RSTR;      /* 0x10 AHB1 外设复位（写 1 复位 GPIO/RCC 等） */
    volatile uint32_t AHB2RSTR;      /* 0x14 AHB2 外设复位 */
    volatile uint32_t AHB3RSTR;      /* 0x18 AHB3 外设复位 */
    volatile uint32_t RESERVED0;     /* 0x1C 保留：占位保证后续偏移正确，不可删 */
    volatile uint32_t APB1RSTR;      /* 0x20 APB1 外设复位（TIM/CAN 等） */
    volatile uint32_t APB2RSTR;      /* 0x24 APB2 外设复位（TIM1/8、USART1 等） */
    volatile uint32_t RESERVED1;     /* 0x28 保留 */
    volatile uint32_t RESERVED2;     /* 0x2C 保留 */
    volatile uint32_t AHB1ENR;       /* 0x30 AHB1 时钟使能（GPIOA~G 使能位在这里） */
    volatile uint32_t AHB2ENR;       /* 0x34 AHB2 时钟使能 */
    volatile uint32_t AHB3ENR;       /* 0x38 AHB3 时钟使能 */
    volatile uint32_t RESERVED3;     /* 0x3C 保留 */
    volatile uint32_t APB1ENR;       /* 0x40 APB1 时钟使能（TIM3/4、CAN1 使能位在这里） */
    volatile uint32_t APB2ENR;       /* 0x44 APB2 时钟使能（TIM1/8、USART1、SYSCFG 使能位在这里） */
    volatile uint32_t RESERVED4;     /* 0x48 保留 */
    volatile uint32_t RESERVED5;     /* 0x4C 保留 */
    volatile uint32_t AHB1LPENR;     /* 0x50 AHB1 低功耗时钟使能（睡眠模式用，本工程未用） */
    volatile uint32_t AHB2LPENR;     /* 0x54 AHB2 低功耗时钟使能 */
    volatile uint32_t AHB3LPENR;     /* 0x58 AHB3 低功耗时钟使能 */
    volatile uint32_t RESERVED6;     /* 0x5C 保留 */
    volatile uint32_t APB1LPENR;     /* 0x60 APB1 低功耗时钟使能 */
    volatile uint32_t APB2LPENR;     /* 0x64 APB2 低功耗时钟使能 */
    volatile uint32_t RESERVED7;     /* 0x68 保留 */
    volatile uint32_t RESERVED8;     /* 0x6C 保留 */
    volatile uint32_t BDCR;          /* 0x70 备份域控制：LSE 晶振、RTC 时钟源 */
    volatile uint32_t CSR;           /* 0x74 控制/状态：LSI 就绪、复位标志、RTC 相关 */
} RCC_TypeDef;

/* ===== 时钟使能位（每个外设 1 位，写 1 开时钟）=====
 * 注意：AHB1/APB1/APB2 各有独立的 ENR，同名位在不同 ENR 里含义不同（下面按 ENR 分组） */
/* AHB1ENR：GPIOA~G 的时钟使能 */
#define RCC_GPIOAEN  (1U << 0)
#define RCC_GPIOBEN  (1U << 1)
#define RCC_GPIOCEN  (1U << 2)
#define RCC_GPIODEN  (1U << 3)
#define RCC_GPIOEEN  (1U << 4)
#define RCC_GPIOFEN  (1U << 5)
#define RCC_GPIOGEN  (1U << 6)
/* APB2ENR：TIM1/TIM8/USART1 的时钟使能（注意这三位与 APB1ENR 的同位号含义完全不同） */
#define RCC_TIM1EN   (1U << 0)
#define RCC_TIM8EN   (1U << 1)
#define RCC_USART1EN (1U << 4)
/* APB1ENR：TIM2/3/4 与 CAN1 的时钟使能 */
#define RCC_TIM2EN   (1U << 0)
#define RCC_TIM3EN   (1U << 1)
#define RCC_TIM4EN   (1U << 2)
#define RCC_CAN1EN   (1U << 25)   /* CAN1 在 APB1ENR 的 bit25 */

#define RCC ((RCC_TypeDef *)RCC_BASE)

/* ================= 通用定时器 TIM2/3/4/5 ================= */
/* 通用定时器没有 BDTR/DCR/DMAR（那是高级定时器专有），所以字段比 TIM_AdvTypeDef 少 4 个，
 * 但前 13 个字段偏移完全一致 —— 两个结构体不能混用指针，否则 BDTR 会写到保留区。 */
typedef struct {
    volatile uint32_t CR1;        /* 0x00 控制1：CEN 使能计数、ARPE 自动重装预装载 */
    volatile uint32_t CR2;        /* 0x04 控制2：主从触发输出选择等（本工程未用） */
    volatile uint32_t SMCR;       /* 0x08 从模式控制：SMS=011 时为编码器模式 3（4 倍频） */
    volatile uint32_t DIER;       /* 0x0C DMA/中断使能（本工程轮询为主，未用） */
    volatile uint32_t SR;         /* 0x10 状态：更新/比较/捕获标志 */
    volatile uint32_t EGR;        /* 0x14 事件产生：软件强制产生更新等事件 */
    volatile uint32_t CCMR1;      /* 0x18 捕获/比较模式1：CH1/CH2 的输入捕获或 PWM 模式配置 */
    volatile uint32_t CCMR2;      /* 0x1C 捕获/比较模式2：CH3/CH4（本工程未用） */
    volatile uint32_t CCER;       /* 0x20 捕获/比较使能：CCxE 打开通道输出/捕获 */
    volatile uint32_t CNT;        /* 0x24 计数器：编码器模式下就是脉冲计数（测速读它） */
    volatile uint32_t PSC;        /* 0x28 预分频：计数时钟 = 定时器时钟/(PSC+1) */
    volatile uint32_t ARR;        /* 0x2C 自动重装：决定 PWM 周期（计数到它归零） */
    volatile uint32_t RESERVED0;  /* 0x30 保留（RM0090 里此处为 RCR 之外的空位，通用定时器无 RCR） */
    volatile uint32_t CCR1;       /* 0x34 比较值1：PWM 占空比（计数值<CCR1 时输出有效电平） */
    volatile uint32_t CCR2;       /* 0x38 比较值2 */
    volatile uint32_t CCR3;       /* 0x3C 比较值3 */
    volatile uint32_t CCR4;       /* 0x40 比较值4 */
} TIM_TypeDef;

#define TIM2 ((TIM_TypeDef *)TIM2_BASE)
#define TIM3 ((TIM_TypeDef *)TIM3_BASE)
#define TIM4 ((TIM_TypeDef *)TIM4_BASE)

/* ================= 高级定时器 TIM1/TIM8 ================= */
/* 比通用定时器多 RCR/BDTR/DCR/DMAR；BDTR.MOE 是高级定时器输出的总开关，PWM 不输出先查它 */
typedef struct {
    volatile uint32_t CR1;        /* 0x00 控制1：CEN 使能计数、ARPE 重装预装载 */
    volatile uint32_t CR2;        /* 0x04 控制2 */
    volatile uint32_t SMCR;       /* 0x08 从模式控制：TIM8 用编码器模式 3 */
    volatile uint32_t DIER;       /* 0x0C DMA/中断使能 */
    volatile uint32_t SR;         /* 0x10 状态标志 */
    volatile uint32_t EGR;        /* 0x14 事件产生 */
    volatile uint32_t CCMR1;      /* 0x18 捕获/比较模式1：PWM 模式与预装载 */
    volatile uint32_t CCMR2;      /* 0x1C 捕获/比较模式2 */
    volatile uint32_t CCER;       /* 0x20 捕获/比较使能 */
    volatile uint32_t CNT;        /* 0x24 计数器（TIM8 编码器读它测右轮） */
    volatile uint32_t PSC;        /* 0x28 预分频（TIM1 配 167 → 1MHz） */
    volatile uint32_t ARR;        /* 0x2C 自动重装（TIM1 配 19999 → 50Hz） */
    volatile uint32_t RCR;        /* 0x30 重复计数：每 N 个周期才产生一次更新（本工程未用，但必须占位） */
    volatile uint32_t CCR1;       /* 0x34 比较值1（舵机脉宽：1500 = 1.5ms） */
    volatile uint32_t CCR2;       /* 0x38 比较值2 */
    volatile uint32_t CCR3;       /* 0x3C 比较值3 */
    volatile uint32_t CCR4;       /* 0x40 比较值4 */
    volatile uint32_t BDTR;       /* 0x44 刹车/死区：bit15 MOE = 主输出使能（不开则无输出） */
    volatile uint32_t DCR;        /* 0x48 DMA 控制（未用） */
    volatile uint32_t DMAR;       /* 0x4C DMA 连续寻址（未用） */
} TIM_AdvTypeDef;

#define TIM1 ((TIM_AdvTypeDef *)TIM1_BASE)
#define TIM8 ((TIM_AdvTypeDef *)TIM8_BASE)

/* TIM 常用位 */
#define TIM_CR1_CEN      (1U << 0)     /* 计数器使能：置 1 才开始计数/输出波形 */
#define TIM_CR1_ARPE     (1U << 7)     /* ARR 预装载使能：改周期不会在本周期中途生效 */
#define TIM_SMCR_SMS     (3U << 0)     /* 编码器模式 3 = 11 */
#define TIM_CCMR1_IC1F   (0x0FU << 4)  /* 输入滤波 */
#define TIM_CCMR1_CC1S   (3U << 0)     /* 输入捕获1选择 */
#define TIM_CCMR1_CC2S   (3U << 8)     /* 通道2 输入源选择（编码器模式选 TI2） */
#define TIM_CCMR1_OC1M   (6U << 4)     /* 输出比较1模式 */
#define TIM_CCMR1_OC2M   (6U << 12)    /* 输出比较2模式 */
#define TIM_CCMR1_OC1PE  (1U << 3)     /* CCR1 预装载：改占空比不产生半周期毛刺 */
#define TIM_CCMR1_OC2PE  (1U << 11)    /* CCR2 预装载 */
#define TIM_CCER_CC1E    (1U << 0)     /* 通道1 输出/捕获使能 */
#define TIM_CCER_CC2E    (1U << 4)     /* 通道2 输出/捕获使能 */
#define TIM_CCER_CC1P    (1U << 1)     /* 通道1 极性：0 高有效 / 1 低有效（本工程保持 0） */
#define TIM_CCER_CC2P    (1U << 5)     /* 通道2 极性 */
#define TIM_BDTR_MOE     (1U << 15)    /* 高级定时器主输出使能：PWM 不出波形时先查这一位 */

/* ================= USART ================= */
typedef struct {
    volatile uint32_t SR;         /* 0x00 状态：TXE 发送寄存器空 / TC 发送完成 / RXNE 收到字节 */
    volatile uint32_t DR;         /* 0x04 数据：写它触发发送、读它取出收到的字节 */
    volatile uint32_t BRR;        /* 0x08 波特率：高 12 位整数分频 + 低 4 位小数分频 */
    volatile uint32_t CR1;        /* 0x0C 控制1：UE 使能、TE/RE 收发使能、字长与校验 */
    volatile uint32_t CR2;        /* 0x10 控制2：停止位长度等（8N1 用默认 0） */
    volatile uint32_t CR3;        /* 0x14 控制3：硬件流控/DMA（本工程未用） */
    volatile uint32_t GTPR;       /* 0x18 保护时间与预分频（智能卡/IrDA 用，本工程未用） */
} USART_TypeDef;

#define USART1 ((USART_TypeDef *)USART1_BASE)
/* 串口位定义：括号里是所在寄存器与位数，便于对照 RM0090 */
#define USART_SR_TXE  (1U << 7)    /* SR bit7：发送数据寄存器空 → 可以写下一个字节（uart_putc 轮询它） */
#define USART_SR_TC   (1U << 6)    /* SR bit6：发送完成（含移位寄存器已空） */
#define USART_SR_RXNE (1U << 5)    /* SR bit5：接收数据寄存器非空 → 有数据可读 */
#define USART_CR1_UE  (1U << 13)   /* CR1 bit13：USART 总使能 */
#define USART_CR1_TE  (1U << 3)    /* CR1 bit3：发送器使能 */
#define USART_CR1_RE  (1U << 2)    /* CR1 bit2：接收器使能 */

/* ================= CAN1 ================= */
/* 寄存器偏移严格按 RM0090 30.5 与 ST 官方 stm32f407xx.h 的 CAN_TypeDef：
 *   控制区   MCR 0x00 / MSR 0x04 / TSR 0x08 / RF0R 0x0C / RF1R 0x10 / IER 0x14 / ESR 0x18 / BTR 0x1C
 *   保留     0x020-0x17F = 88 个字（少写会让整个邮箱区前移！）
 *   发送邮箱 TIR/TDTR/TDLR/TDHR：T0 0x180 / T1 0x190 / T2 0x1A0
 *   接收邮箱 RIR/RDTR/RDLR/RDHR：F0 0x1B0 / F1 0x1C0
 *   保留     0x1D0-0x1FF = 12 个字
 *   过滤器   FMR 0x200 / FM1R 0x204 / (0x208) / FS1R 0x20C / (0x210) /
 *            FFA1R 0x214 / (0x218) / FA1R 0x21C / 0x220-0x23F 保留
 *   过滤体   sFilterRegister[i]：FR1 = 0x240 + 8i（ID）、FR2 = FR1 + 4（Mask）
 *            → F0R1=0x240、F0R2=0x244、F1R1=0x248、F1R2=0x24C
 * 注意：0x208/0x210/0x218 在 STM32F4 上是保留字（F1 系列的 FM2R/FS2R/FFA2R 等
 *       CAN2 专用寄存器在 F4/CAN1 不存在），故此处不命名、只占位。 */
typedef struct {
    volatile uint32_t MCR;      /* 0x00 */
    volatile uint32_t MSR;      /* 0x04 */
    volatile uint32_t TSR;      /* 0x08 */
    volatile uint32_t RF0R;     /* 0x0C */
    volatile uint32_t RF1R;     /* 0x10 */
    volatile uint32_t IER;      /* 0x14 */
    volatile uint32_t ESR;      /* 0x18 */
    volatile uint32_t BTR;      /* 0x1C */
    volatile uint32_t RESERVED0[88];  /* 0x20-0x17F：邮箱区前保留段，[88] 不可改小 */
    volatile uint32_t TX0R;     /* 0x180 TIR0 */
    volatile uint32_t TX0D0;    /* 0x184 TDTR0 (DLC) */
    volatile uint32_t TX0D1;    /* 0x188 TDLR0 (字节0-3) */
    volatile uint32_t TX0D2;    /* 0x18C TDHR0 (字节4-7) */
    volatile uint32_t TX1R;     /* 0x190 TIR1 */
    volatile uint32_t TX1D0;    /* 0x194 TDTR1 */
    volatile uint32_t TX1D1;    /* 0x198 TDLR1 */
    volatile uint32_t TX1D2;    /* 0x19C TDHR1 */
    volatile uint32_t TX2R;     /* 0x1A0 TIR2 */
    volatile uint32_t TX2D0;    /* 0x1A4 TDTR2 */
    volatile uint32_t TX2D1;    /* 0x1A8 TDLR2 */
    volatile uint32_t TX2D2;    /* 0x1AC TDHR2 */
    volatile uint32_t RX0R;     /* 0x1B0 RIR0 */
    volatile uint32_t RX0D0;    /* 0x1B4 RDTR0 (DLC) */
    volatile uint32_t RX0D1;    /* 0x1B8 RDLR0 (字节0-3) */
    volatile uint32_t RX0D2;    /* 0x1BC RDHR0 (字节4-7) */
    volatile uint32_t RX1R;     /* 0x1C0 RIR1 */
    volatile uint32_t RX1D0;    /* 0x1C4 RDTR1 */
    volatile uint32_t RX1D1;    /* 0x1C8 RDLR1 */
    volatile uint32_t RX1D2;    /* 0x1CC RDHR1 */
    volatile uint32_t RESERVED1[12];  /* 0x1D0-0x1FF：到 FMR(0x200) 前恰好 12 个字 */
    volatile uint32_t FMR;      /* 0x200 */
    volatile uint32_t FM1R;     /* 0x204 */
    volatile uint32_t RESERVED2;/* 0x208（F4 保留；F1 的 FM2R 在此不存在） */
    volatile uint32_t FS1R;     /* 0x20C */
    volatile uint32_t RESERVED3;/* 0x210（F4 保留） */
    volatile uint32_t FFA1R;    /* 0x214 */
    volatile uint32_t RESERVED4;/* 0x218（F4 保留） */
    volatile uint32_t FA1R;     /* 0x21C */
    volatile uint32_t RESERVED5[8];   /* 0x220-0x23F：过滤体前的保留段 */
    volatile uint32_t F0R1;     /* 0x240 过滤体0 ID  （sFilterRegister[0].FR1） */
    volatile uint32_t F0R2;     /* 0x244 过滤体0 Mask（sFilterRegister[0].FR2） */
    volatile uint32_t F1R1;     /* 0x248 */
    volatile uint32_t F1R2;     /* 0x24C */
} CAN_TypeDef;

#define CAN1 ((CAN_TypeDef *)CAN1_BASE)

/* CAN MCR（主控制）：初始化请求/睡眠/发送优先级/自动恢复等总开关 */
#define CAN_MCR_INRQ   (1U << 0)     /* 初始化请求：置 1 才能改 BTR/过滤器 */
#define CAN_MCR_SLEEP  (1U << 1)     /* 睡眠模式：进初始化态时硬件会自动置位，必须清掉 */
#define CAN_MCR_TXFP   (1U << 2)     /* 发送 FIFO 优先级：1=按请求顺序发送（本工程置 1，保证周期帧不乱序） */
#define CAN_MCR_RFLM   (1U << 3)     /* 接收 FIFO 锁定：1=满后覆盖最旧帧（本工程置 0，保留溢出标志） */
#define CAN_MCR_NART   (1U << 4)     /* 禁止自动重传：1=失败不重发（本工程置 0 = 自动重传） */
#define CAN_MCR_AWUM   (1U << 5)     /* 自动唤醒 */
#define CAN_MCR_ABOM   (1U << 6)     /* 自动离线恢复：Bus-Off 后硬件自动尝试回总线（本工程置 1） */
#define CAN_MCR_TTCM   (1U << 7)     /* 时间触发通信模式 */
#define CAN_MCR_RESET  (1U << 15)    /* 软件主复位 */
#define CAN_MCR_DBF    (1U << 16)    /* 调试冻结：调试器断点时冻结接收（排查"卡在 init 态"用） */
/* CAN MSR（主状态） */
#define CAN_MSR_INAK   (1U << 0)     /* 初始化确认：1=硬件真在初始化模式（与 INRQ 成对使用） */
#define CAN_MSR_SLAK   (1U << 1)     /* 睡眠确认 */
#define CAN_MSR_ERRI   (1U << 2)     /* 错误中断标志 */
#define CAN_MSR_TXM    (1U << 8)     /* 发送模式 */
#define CAN_MSR_RXM    (1U << 9)     /* 接收模式 */
#define CAN_MSR_SAMP   (1U << 10)    /* 上次采样点值 */
#define CAN_MSR_RX     (1U << 11)    /* 当前正在接收 */
/* CAN TSR
 * ⚠ 注意命名：本工程的 CAN_TSR_TME 语义 = "任一邮箱空"（TME0|TME1|TME2），
 *   与 ST 官方 stm32f407xx.h 同名字段的 bit25（"三个邮箱全空"）不一样！这是故意如此：
 *   Can_Send 用它判"有没有空邮箱可写"。若要"三箱全空"，用 CAN_TSR_TME_ALL(bit25)。 */
#define CAN_TSR_TME0   (1U << 26)    /* 发送邮箱 0 空 */
#define CAN_TSR_TME1   (1U << 27)    /* 发送邮箱 1 空 */
#define CAN_TSR_TME2   (1U << 28)    /* 发送邮箱 2 空 */
#define CAN_TSR_TME    (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)  /* 任一为空 */
#define CAN_TSR_TME_ALL (1U << 25)  /* RM0090 TSR bit25：三箱全空（= ST 官方的 TME） */
/* CAN RF0R（接收 FIFO0 状态/释放） */
#define CAN_RF0R_FMP0  (3U << 0)     /* [1:0] FIFO0 待读帧数（0~3，轮询时非 0 即有帧） */
#define CAN_RF0R_FULL0 (1U << 3)     /* FIFO0 满 */
#define CAN_RF0R_FOVR0 (1U << 4)     /* FIFO0 溢出：硬件置 1，写 1 清除（丢帧计数用） */
#define CAN_RF0R_RFOM0 (1U << 5)     /* 释放输出邮箱：写 1 才会让 FMP0 减 1（读完必须释放） */
/* CAN IER（中断使能）：本工程用轮询，此处只留最常用的 FIFO0 中断使能位 */
#define CAN_IER_FMPIE0 (1U << 1)     /* FIFO0 消息挂起中断使能 */
/* CAN ESR（错误状态） */
#define CAN_ESR_EWGF   (1U << 0)     /* 警告：TEC/REC ≥ 96 */
#define CAN_ESR_EPVF   (1U << 1)     /* 被动错误：TEC/REC ≥ 128 */
#define CAN_ESR_BOFF   (1U << 2)     /* 离线：TEC > 255，节点已脱离总线（Can_IsBusOff 读它） */
#define CAN_ESR_TEC    (0xFFU << 16) /* [23:16] 发送错误计数 */
#define CAN_ESR_REC    (0xFFU << 24) /* [31:24] 接收错误计数 */
/* CAN BTR 测试模式位（RM0090 30.9.2 CAN_BTR）
 *   LBKM(bit30): 环回模式——TX 内部接回 RX，可自发自收
 *   SILM(bit31): 静默模式——不驱动 CANTX、不接收总线，仅内部反馈
 *   两者同时置 1 = 静默环回(silent loopback)：不用收发器/对端/终端电阻即可自测
 *   （只置 SILM 是"只收不发"，无法自发自收，自测必须 LBKM|SILM） */
#define CAN_BTR_LBKM   (1U << 30)
#define CAN_BTR_SILM   (1U << 31)

/* CAN FMR */
#define CAN_FMR_FINIT  (1U << 0)
/* CAN FA1R: 过滤器激活 */

/* ================= 其他 ================= */
/* Flash 接口寄存器区（RM0090: 0x40023C00） */
#define FLASH_R_BASE 0x40023C00UL      /* Flash 接口寄存器基址（不是 Flash 存储区，别和 0x08000000 搞混） */
#define FLASH_ACR ((volatile uint32_t *)(FLASH_R_BASE + 0x00UL))   /* 访问控制：等待周期 + 缓存 */
#define FLASH_ACR_ICEN   (1U << 9)     /* 指令缓存使能 */
#define FLASH_ACR_DCEN   (1U << 10)    /* 数据缓存使能 */
#define FLASH_ACR_LATENCY_MASK (0x7U)  /* [2:0] 等待周期掩码（5WS → 5） */
#define FLASH_ACR_LATENCY_5WS  (5U << 0)  /* 168MHz 必须 5 个等待周期，否则取指出错 */

/* 内核 SCB 区（0xE000ED00）：这里只定义用得到的 ACTLR */
#define SCB_BASE 0xE000ED00UL          /* Cortex-M4 系统控制块基址（VTOR 在 +0x08） */
#define SCB_ACR  ((volatile uint32_t *)(SCB_BASE + 0x14UL))   /* 辅助控制寄存器 ACTLR */

/* SysTick（0xE000E010）：1ms 系统节拍，millis() 的唯一来源 */
#define SYSTICK_BASE 0xE000E010UL
typedef struct {
    volatile uint32_t CSR;        /* 0x00 控制/状态：ENABLE 启动、TICKINT 中断使能、CLKSOURCE 选时钟源 */
    volatile uint32_t RVR;        /* 0x04 重装值：每次数到 0 就重新装载（1ms = HCLK/1000 − 1） */
    volatile uint32_t CVR;        /* 0x08 当前值：写任何值都清零 */
    volatile uint32_t CALIB;      /* 0x0C 校准值（只读，本工程未用） */
} SysTick_TypeDef;
#define SysTick ((SysTick_TypeDef *)SYSTICK_BASE)
#define SYSTICK_CSR_ENABLE  (1U << 0)     /* 计数器使能 */
#define SYSTICK_CSR_TICKINT (1U << 1)     /* 数到 0 产生 SysTick 异常（进 SysTick_Handler） */
#define SYSTICK_CSR_CLKSOURCE (1U << 2)   /* 1 = 用处理器时钟 HCLK（168MHz），0 = 用 HCLK/8 */

#define __WFI() __asm volatile ("wfi")    /* 等待中断：进低功耗直到下一个中断（本工程未用） */

/* ================= SYSCFG（必须在 include guard 内：否则本头被包含两次时
 *  SYSCFG_TypeDef 会重定义报错）================= */
typedef struct
{
  __IO uint32_t MEMRMP;      /* 0x00 存储器重映射：把 Flash/SRAM 映射到 0x00000000 区 */
  __IO uint32_t PMC;         /* 0x04 外设模式配置：SWJ_CFG[26:24] 控制 JTAG/SWD 引脚占用 */
  __IO uint32_t EXTICR1;     /* 0x08 外部中断线 0~3 的端口映射（本工程未用） */
  __IO uint32_t EXTICR2;     /* 0x0C 外部中断线 4~7 的端口映射 */
  __IO uint32_t EXTICR3;     /* 0x10 外部中断线 8~11 的端口映射 */
  __IO uint32_t EXTICR4;     /* 0x14 外部中断线 12~15 的端口映射 */
  uint32_t RESERVED1[2];     /* 0x18-0x1C 保留（占位，保证 CMPCR 偏移正确） */
  __IO uint32_t CMPCR;       /* 0x20 补偿单元控制：内部电压补偿（本工程未用） */
} SYSCFG_TypeDef;

#define SYSCFG_BASE         0x40013800UL   /* SYSCFG 在 APB2 域 */
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define RCC_APB2ENR_SYSCFGEN (1U << 14)    /* APB2ENR bit14：SYSCFG 时钟使能（写 PMC 前必须先开） */

#endif /* STM32F407_REGS_H */

