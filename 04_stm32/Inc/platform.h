/**
 * platform.h - 平台抽象层（Platform Abstraction Layer）
 *
 * ═══════════ 这个头文件是"全局工具箱" ═══════════
 * 所有驱动文件（motor/servo/can/encoder）都 include 它，提供：
 *   1. 类型别名    : uint8/sint16/boolean...（和脚手架/AUTOSAR 风格一致）
 *   2. 板级常量    : LED 引脚、SYSTEM_CLOCK_HZ
 *   3. 基础服务    : SystemClock_Config / millis / delay_ms / uart_printf
 *   4. GPIO 助手   : gpio_enable_clock / gpio_mode / gpio_af / gpio_write
 *                    （寄存器级内联函数，见文件后段）
 *   5. 外设寄存器  : 来自 Core/Inc/stm32f407_regs.h
 *
 * 本版本说明（v2，适配 STM32F407ZGT6 最小系统板）：
 *   - 不再依赖 CubeMX / HAL：底层为寄存器级实现（Core/Inc/stm32f407_regs.h）
 *   - 保留脚手架的 AUTOSAR 风格类型别名（uint8/sint8/boolean...），
 *     上层业务代码写法不变
 *   - 提供时钟 / 延时 / 调试串口 / GPIO 助手
 *
 * 若日后要迁 AUTOSAR：业务代码不动，把下面这些接口换成 MCAL 实现即可。
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include "stm32f407_regs.h"   /* 寄存器级定义（替代 HAL/CubeMX，规则 §3） */
#include <stdint.h>           /* 固定宽度整数：uint8_t/int16_t... 用于下方 AUTOSAR 风格别名 */
#include <stdarg.h>           /* va_list：uart_printf 的可变参数（当前不做格式化，仅占位） */
// #include <stdio.h>

/* ===== 通用类型别名（与脚手架/AUTOSAR Std_Types.h 对齐） ===== */
typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int8_t   sint8;
typedef int16_t  sint16;
typedef int32_t  sint32;
typedef float    float32;
typedef double   float64;

#ifndef TRUE
#define TRUE  1u
#endif
#ifndef FALSE
#define FALSE 0u
#endif
typedef uint8 boolean;      /* 布尔用 1 字节无符号表示（TRUE=1/FALSE=0），与 AUTOSAR boolean 语义一致 */

/* ===== 常用宏 ===== */
#define UNUSED(x)  ((void)(x))   /* 显式标记"参数未使用"，避免 -Wextra 的 unused-parameter 警告 */

/* ===== 系统时钟频率 ===== */
#define SYSTEM_CLOCK_HZ  168000000u   /* HSE 8MHz -> PLL 168MHz */

/* ================================================================
 * 平台基础服务（寄存器级实现，定义在 Src/platform.c —— 平台层）
 * 应用层/调度层只调这些函数，不直接碰寄存器（.clinerules §4.1）
 * ================================================================ */

/**
 * @brief  上电早期初始化（main() 的第一句，必须早于任何外设配置）
 * @param  无
 * @return 无
 * @note   做两件事：
 *         ① 关 JTAG（SWJ_CFG=001）把 PB3/PB4 从调试口释放成普通 GPIO
 *            —— TB6612 的 STBY=PB3、BIN1=PB4 正好占这两个脚；
 *         ② 把向量表基址 SCB->VTOR 指回 0x08000000，防调试器/其它引导程序
 *            改过之后中断跳飞。
 */
void    Platform_EarlyInit(void);

/**
 * @brief  系统时钟配置：HSE 8MHz → PLL → SYSCLK 168MHz（APB1 42MHz / APB2 84MHz）
 * @param  无
 * @return 无
 * @note   内部先写 Flash 等待周期（5WS + 缓存使能）再提频，顺序不能反。
 *         时钟树已调通冻结（.clinerules §1）：改这里会连带改 CAN 位时序、
 *         USART 波特率、PWM 频率 —— 不要单独动。
 */
void    SystemClock_Config(void);

/**
 * @brief  启动 1ms 系统心跳（SysTick 重装值 168000-1，时钟源 = 内核时钟）
 * @param  无
 * @return 无
 * @note   millis() / delay_ms() 的时间基准都来自它，必须在 SystemClock_Config()
 *         之后调用（否则重装值按旧频率算，1ms 就不准了）。
 */
void    SysTick_Init(void);

/**
 * @brief  取"上电以来经过了多少毫秒"
 * @param  无
 * @return 毫秒计数（0 ~ 4294967295，约 49.7 天自然回绕一次）
 * @note   SysTick 中断里自增的计数器。调用方一律写成 (now - t0) >= 周期 的
 *         差值形式，天然抗回绕；这是调度层唯一允许的计时手段（禁止 delay_ms）。
 */
uint32  millis(void);

/**
 * @brief  阻塞忙等延时
 * @param  ms 延时长度，单位毫秒（传 0 = 立即返回）
 * @return 无
 * @note   ⚠ 禁止在 Task_10ms/20ms/100ms 内调用：会把 CAN 轮询挤掉 → 丢帧
 *         （规则 §3 历史事故）。只允许初始化阶段/一次性自检使用。
 */
void    delay_ms(uint32 ms);

/* ================================================================
 * 板载 LED（低电平点亮）：初始化 + 点亮/熄灭
 * 应用层/调度层只调这些接口，BSRR 写落在 platform.c 内部（§4：业务不碰寄存器）
 *   LED0 = PF9 (红)：RDK 蜂鸣快闪 / 刹停·HOLD 常亮；brake_v2 分支作 AEB 指示
 *   LED1 = PF10(绿)：heartbeat_v1 握手状态指示（STATIC 灭 / HANDSHAKE 快闪≈200ms /
 *                    CRUISE 慢闪≈1s / HS FAIL 常亮）—— 与 LED0 语义隔离，互不打架
 * ================================================================ */

/**
 * @brief  初始化 LED0（PF9）
 * @param  无
 * @return 无
 * @note   开 GPIOF 时钟 + PF9 配推挽输出 + 上电熄灭。
 */
void    Led_Init(void);

/**
 * @brief  点亮 / 熄灭 LED0（PF9，低电平点亮）
 * @param  on TRUE = 点亮（引脚输出低电平）；FALSE = 熄灭（输出高电平）
 * @return 无
 * @note   硬件是"GPIO 拉低才亮"，反相处理在 platform.c 内部，业务层不必关心。
 */
void    Led_Set(boolean on);

/**
 * @brief  初始化 LED1（PF10，板载绿）
 * @param  无
 * @return 无
 * @note   GPIOF 时钟重复使能是幂等的，可与 LED0 先后调用、顺序无关。
 */
void    Led1_Init(void);

/**
 * @brief  点亮 / 熄灭 LED1（PF10，低电平点亮）
 * @param  on TRUE = 点亮；FALSE = 熄灭
 * @return 无
 * @note   heartbeat_v1 里表示握手/链路状态，与 LED0（制动/蜂鸣语义）互不干扰，
 *         所以状态机可以同时用两盏灯表达两件事。
 */
void    Led1_Set(boolean on);

/* ================================================================
 * 调试串口 USART1 @ PA9(TX)/PA10(RX)，115200 8N1（板载 CH340 直连）
 * ================================================================ */

/**
 * @brief  初始化调试串口 USART1（115200 8N1，查询式收发，不占用中断）
 * @param  无
 * @return 无
 * @note   PA9/PA10 配 AF7；BRR 由 APB2=84MHz 推算（0x02D9，误差 0.023%）。
 *         时钟一改波特率就错 —— 对应 tools/timing_audit.py 的检查项。
 */
void    UART1_Init(void);

/**
 * @brief  发送一个字节（阻塞等 TXE，@115200 约 87us）
 * @param  c 要发送的字符（char）；换行要自己先发 '\r' 再发 '\n'
 * @return 无
 * @note   按 8 位数据透传；中文等多字节字符由调用方按字节传入。
 */
void    uart_putc(char c);

/**
 * @brief  发送以 '\0' 结尾的字符串
 * @param  s 字符串首地址（不可为 NULL；内部逐字节发到 '\0' 为止）
 * @return 无
 * @note   只做原始字节透传，不做编码转换、不自动加换行。
 */
void    uart_puts(const char *s);

/**
 * @brief  串口打印（⚠ 当前实现**不做格式化**：把 fmt 原样输出）
 * @param  fmt 要输出的文本；里面的 %d/%s 等**不会被替换**，会原样打出来
 * @param  ... 可变参数占位（当前实现完全忽略，只为保持接口形态）
 * @return 无
 * @note   原因：nano.specs 下 vsnprintf 会 HardFault（规则 §3）。
 *         要打数字请组合调用：uart_puts("pwm=") + uart_put_sint(pwm) + uart_puts("\r\n")。
 */
void    uart_printf(const char *fmt, ...);

/**
 * @brief  打印无符号十进制整数（自写 itoa，不依赖 printf）
 * @param  v 取值范围 0 ~ 4294967295
 * @return 无
 */
void    uart_put_uint(uint32 v);

/**
 * @brief  打印有符号十进制整数（负数自带 '-'）
 * @param  v 取值范围 -2147483648 ~ 2147483647
 * @return 无
 * @note   内部取绝对值后转无符号再打，INT32_MIN 也不会溢出。
 */
void    uart_put_sint(sint32 v);

/* ===== 板载 LED 引脚映射 =====
 * M144Z-M4（正点原子 STM32F407 Mini板）：LED0=PF9(红)、LED1=PF10(绿)，低电平点亮。
 * 代码使用 LED0(PF9) 与 LED1(PF10)。引脚级细节（LED_PORT/LED_PIN 与 BSRR 写）已下沉到
 * Src/platform.c；换板子改那里的 LED_PORT/LED_PIN/LED1_PORT/LED1_PIN，本头文件不再暴露寄存器宏。
 */

/* ================================================================
 * GPIO 助手（寄存器级）
 *   ⚠ 这里是唯一允许出现 GPIO 寄存器位操作的地方（平台层内部）；
 *     驱动层调这些助手，业务层只调驱动接口（.clinerules §4）
 *   ⚠ 五个都是 static inline：直接展开进调用者，无函数调用开销
 *   ⚠ 标准调用顺序：gpio_enable_clock() → gpio_mode()
 *      →（复用功能才需要）gpio_af() →（输出才需要）gpio_write()
 * ================================================================ */

/**
 * @brief  使能某个 GPIO 端口的 AHB1 时钟
 * @param  port 端口基址指针：GPIOA ~ GPIOG（本工程用到的范围）
 * @return 无
 * @note   **必须先开时钟**，否则后续对 MODER/AFR/BSRR 的写全部无效（外设时钟关闭
 *         时寄存器写被忽略）。重复调用幂等（内部是置位），同端口多个引脚分别初始化
 *         时每次都调也没问题。传 GPIOH/GPIOI 等未列出的端口 → 什么都不做（安全空操作）。
 */
static inline void gpio_enable_clock(GPIO_TypeDef *port)
{
    /* AHB1ENR 里每个端口一位：按端口指针选择对应使能位（开时钟后寄存器才可写） */
    if (port == GPIOA)      RCC->AHB1ENR |= RCC_GPIOAEN;
    else if (port == GPIOB) RCC->AHB1ENR |= RCC_GPIOBEN;
    else if (port == GPIOC) RCC->AHB1ENR |= RCC_GPIOCEN;
    else if (port == GPIOD) RCC->AHB1ENR |= RCC_GPIODEN;
    else if (port == GPIOE) RCC->AHB1ENR |= RCC_GPIOEEN;
    else if (port == GPIOF) RCC->AHB1ENR |= RCC_GPIOFEN;
    else if (port == GPIOG) RCC->AHB1ENR |= RCC_GPIOGEN;
    /* 其它端口（H/I）本工程未用到，故不列出；传进来时什么都不做（不会误写别的位置） */
}

/**
 * @brief  设置一个或多个引脚的工作模式（写 MODER，每脚占 2 位）
 * @param  port 端口基址：GPIOA~GPIOG（该类端口须已 gpio_enable_clock 开时钟）
 * @param  pin  引脚**位掩码，不是引脚号**！单脚如 GPIO_PIN_9；多脚用 | 合并，
 *              例如 GPIO_PIN_0 | GPIO_PIN_1 一次配两脚
 * @param  mode 模式编码：GPIO_MODE_INPUT(0) 输入 / GPIO_MODE_OUTPUT(1) 推挽输出 /
 *              GPIO_MODE_AF(2) 复用（交 TIM/USART/CAN 等外设）/
 *              GPIO_MODE_ANALOG(3) 模拟
 * @return 无
 * @note   读-改-写：先清该脚的 2 位再写新值，同端口其它引脚配置不受影响。
 *         掩码里为 0 的位被跳过，所以"配多脚"和"只配一脚"是同一套逻辑。
 */
static inline void gpio_mode(GPIO_TypeDef *port, uint16 pin, uint8 mode)
{
    uint32 pos = 0;          /* 当前考察的引脚号（从 0 开始，逐个右移 pin 掩码） */
    while (pin) {            /* pin 是位掩码：为 0 表示已经没有要处理的引脚了 */
        if (pin & 1U) {      /* 当前第 pos 位为 1 → 这个引脚需要配置 */
            port->MODER &= ~(3U << (pos * 2));                  /* 先清 MODER 的 2 位（读-改-写，不动别的引脚） */
            port->MODER |=  ((uint32)mode << (pos * 2));        /* 再写入模式：00 输入/01 输出/10 复用/11 模拟 */
        }
        pin >>= 1;           /* 掩码右移一位：下一个循环考察下一个引脚 */
        pos++;               /* 引脚号 +1 */
    }
}

/**
 * @brief  给一个引脚选择复用功能编号（写 AFRL/AFRH，每脚占 4 位）
 * @param  port 端口基址：GPIOA~GPIOG（须已开时钟，且该脚已设为 GPIO_MODE_AF）
 * @param  pin  引脚**位掩码**，本函数只接受**单个**引脚（如 GPIO_PIN_9）；
 *              传多脚掩码只会处理最低位那个脚
 * @param  af   复用功能号 0~15，取自数据手册复用表。本工程实测用到的：
 *              TIM1_CH1=1（PA8 舵机）、TIM3=2（PA6/PA7 PWM）、TIM4=2（PD12/PD13）、
 *              TIM8=3（PC6/PC7）、USART1=7（PA9/PA10）、CAN1=9（PA11/PA12）
 * @return 无
 * @note   引脚 0~7 走 AFRL、8~15 走 AFRH（本函数内部自动选）。
 *         ⚠ 复用号填错不会报错，只会"外设没波形/收不到数据"，排查时先核对这张表。
 */
static inline void gpio_af(GPIO_TypeDef *port, uint16 pin, uint8 af)
{
    uint32 pos = 0;                            /* 引脚号 */
    while (!(pin & (1U << pos))) pos++;        /* 找到掩码里第一个（也是唯一一个）为 1 的位 = 目标引脚号 */

    if (pos < 8) {                             /* 引脚 0~7 用 AFRL */
        port->AFRL &= ~(0xFU << (pos * 4));          /* 清该引脚的 4 位复用选择 */
        port->AFRL |=  ((uint32)af << (pos * 4));    /* 写入 AF 号（如 USART1=7、TIM3=2、CAN1=9） */
    } else {                                   /* 引脚 8~15 用 AFRH */
        pos -= 8;                                    /* AFRH 里重新从 0 开始编号 */
        port->AFRH &= ~(0xFU << (pos * 4));
        port->AFRH |=  ((uint32)af << (pos * 4));
    }
}

/**
 * @brief  设置引脚输出电平（写 BSRR，单次原子操作）
 * @param  port  端口基址：GPIOA~GPIOG（该脚须已设为推挽输出）
 * @param  pin   引脚**位掩码**（GPIO_PIN_x；多脚可 | 合并，一次写多脚）
 * @param  level 非 0 = 输出高电平；0 = 输出低电平
 * @return 无
 * @note   写 BSRR 不需要读-改-写，因此不会打断同端口其它引脚的输出
 *         （对比：先读 ODR 再改位会与中断里的写竞争）。
 *         ⚠ 本板 LED 是低电平点亮，用 Led_Set() 时 TRUE 反而写的是"清零位"。
 */
static inline void gpio_write(GPIO_TypeDef *port, uint16 pin, uint8 level)
{
    /* BSRR 低 16 位写 1 = 置高、高 16 位写 1 = 置低；单次写是原子操作，无需读-改-写 */
    if (level) port->BSRR = (uint32)pin;          /* 置 1 */
    else       port->BSRR = (uint32)pin << 16;    /* 清 0 */
}

/**
 * @brief  读取引脚当前电平（读 IDR）
 * @param  port 端口基址：GPIOA~GPIOG
 * @param  pin  引脚**位掩码**（GPIO_PIN_x）；多脚合并时含义变为"其中是否有高电平"
 * @return 1 = 该脚为高电平（多脚掩码：至少有 1 脚为高）；0 = 全为低
 * @note   读的是引脚**实际**电平（IDR），不是输出锁存值（ODR）——输出脚读回也有效。
 *         板载 LED 类低电平有效的器件，读到 0 才是"点亮"。
 */
static inline uint8 gpio_read(GPIO_TypeDef *port, uint16 pin)
{
    return (port->IDR & pin) ? 1 : 0;      /* IDR 是引脚实际电平；按掩码取位后归一化成 0/1（支持多脚掩码） */
}

#endif /* PLATFORM_H */
