/**
 * platform.c - 平台层实现（Platform Layer，对应 AUTOSAR CP 里 MCAL 的位置）
 *
 * 归属（.clinerules §4 四层框架）：
 *   平台层 = 时钟 / SysTick 时间基准 / 阻塞延时 / 调试串口 / GPIO 与 LED 基础抽象
 *   本文件只依赖 stm32f407_regs.h（经 platform.h 引入），不含任何业务逻辑；
 *   应用层与调度层只通过 platform.h 的声明调用，不直接操作寄存器。
 *
 * 迁移说明（本轮"缺层补层"）：
 *   这些实现原先内联在 Src/main.c（应用层文件）中，现**逐字**搬到本文件；
 *   寄存器赋值、位值、执行顺序、引脚全部未改动（§4.3：禁止为像 AUTOSAR
 *   而重排引脚或改外设初始化）。对外接口签名与 platform.h 声明完全一致，
 *   上层调用点不改；main() 的启动期寄存器块收拢为 Platform_EarlyInit()。
 *
 * 构建：Makefile 用 wildcard 收集 Src 下全部 .c，新增本文件无需改 Makefile。
 */
#include "platform.h"
#include <stddef.h>     /* _sbrk 的 ptrdiff_t */

/* ================================================================
 *  启动期寄存器收拢（原先内联在 main() 里 → 属平台层职责）
 * ================================================================ */
/**
 * @brief  启动期寄存器收拢：关 JTAG-DP（保留 SW-DP）+ 把中断向量表基址重新指向 Flash
 * @return 无
 * @note   ① 必须在本工程**任何 GPIO 配置与中断使能之前**执行：PB3=STBY、PB4=BIN1 上电
 *           默认是 JTAG 复用脚，不先关 JTAG 则 STBY 恒低 → TB6612 整片待机；
 *         ② 只开 SYSCFG 时钟（写 PMC 需要），不开其它外设时钟；
 *         ③ 上电只调一次（main 第一行）；重复调用无害（写的是固定值）。
 */
void Platform_EarlyInit(void)
{
    /* ===== 禁用 JTAG-DP，保留 SW-DP =====
     * PB3=JTDO、PB4=NJTRST 上电后默认是 JTAG 复用脚，即使软件把它们配成推挽
     * 输出，JTAG 外设仍会抢走引脚。本工程 PB3=STBY(TB6612 使能)、
     * PB4=BIN1(右轮方向)：不禁用 JTAG → STBY 永远为低 → TB6612 整片待机
     * → AO/BO 无输出。因此必须在所有 GPIO 配置之前执行。 */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;      /* 开启 SYSCFG 外设时钟：不开时钟，下一行写 PMC 会被硬件直接丢弃 */

    /* SWJ_CFG[26:24]：001 = 禁用 JTAG-DP、保留 SW-DP(PA13/PA14)，PB3/PB4 释放为普通 GPIO。
     * 注：历史注释写作 "010"，但写入的值一直是 001（010 会连 SW-DP 一起关掉，
     * 与"保留 SWD"的意图矛盾）；此处按实际写入值注为 001，代码未改。 */
    SYSCFG->PMC &= ~(0x7UL << 24);             /* 第一步：把这 3 位清 0（读-改-写，其它位保持原值） */
    SYSCFG->PMC |=  (0x1UL << 24);             /* 第二步：写入 001 —— 关 JTAG 留 SWD，PB3(STBY)/PB4(BIN1) 才归还 GPIO */

    /* ===== 中断向量表定位到 Flash（必须在任何中断使能之前） ===== */
    /* 0xE000ED08 = 内核 SCB->VTOR 寄存器（向量表基址）；0x08000000 = 链接脚本里 FLASH 的起始地址。
     * 上电默认值本来就是它，这里显式再写一次：防止调试器/其它引导程序改过 VTOR 后中断跳飞。 */
    *(volatile uint32_t *)0xE000ED08 = 0x08000000;
}

/* ================================================================
 *  时钟 / 系统心跳（寄存器级 168MHz）
 * ================================================================ */
/**
 * @brief  系统时钟配置：HSE 8MHz → PLL(M=8, N=336, P=2, Q=7) → SYSCLK 168MHz，
 *         并分出 AHB=168MHz / APB1=42MHz / APB2=84MHz
 * @return 无
 * @note   ① 所有等待都有界（1e6 次循环）：晶振坏/焊虚时不会死等卡住，便于 SWD 观察自救；
 *         ② APB1=42MHz 是 CAN 500kbps 与 TIM3 20kHz 的前提，APB2=84MHz 是 USART1
 *           115200、TIM1 1MHz 的前提 —— 改分频必须同步 tools/timing_audit.py 复核；
 *         ③ 必须早于 SysTick_Init()/UART1_Init()/Can_Init()：那些模块的分频与波特率
 *           都是按 168MHz 算出来的常量；
 *         ④ 第 5 步切换生效后，本函数后续语句已跑在 168MHz。
 */
void SystemClock_Config(void)
{
    /* 1. Flash 等待周期 5WS + 缓存使能（168MHz 必须 5WS） */
    /* 整字覆盖写 FLASH_ACR：5WS = 168MHz 下读 Flash 需等 5 个 HCLK；
     * ICEN/DCEN = 指令/数据缓存使能（提速，且与 5WS 配套）。必须先于提频写，否则取指错。 */
    *FLASH_ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

    /* 2. 打开 HSE 8MHz 外部晶振，等待就绪 */
    RCC->CR |= (1UL << 16);              /* HSEON：置 1 给外部晶振供电起振 */
    uint32 timeout = 1000000;            /* 有界等待计数：晶振坏/焊虚时约 6ms 就会退出，不会死等卡住 */
    while (!(RCC->CR & (1UL << 17))) {   /* HSERDY：等硬件把这一位置 1（表示 8MHz 已稳定） */
        if (--timeout == 0) break;       /* 超时也继续往下配（便于用 SWD 观察/自救，而不是卡死在这里） */
    }

    /* 3. 配 PLL：M=8 → 1MHz，N=336 → 336MHz，P=÷2 → 168MHz，Q=7 → 48MHz */
    /* 位段拆解：[22]=PLLSRC 取 HSE(不是内部 16MHz)、[5:0]=M=8、[14:6]=N=336、
     *          [17:16]=P=00(÷2)、[27:24]=Q=7。VCO = 8MHz/8×336 = 336MHz → /2 = 168MHz(SYSCLK)。 */
    RCC->PLLCFGR = (1UL << 22) | (8UL << 0) | (336UL << 6) | (0UL << 16) | (7UL << 24);

    /* 4. 打开 PLL，等待锁定 */
    RCC->CR |= (1UL << 24);              /* PLLON：给 PLL 模拟电路上电开始锁定 */
    timeout = 1000000;                   /* 重新装载超时计数（PLL 锁定通常几十 us） */
    while (!(RCC->CR & (1UL << 25))) {   /* PLLRDY：等锁定完成再切时钟，否则切过去会跑飞 */
        if (--timeout == 0) break;       /* 超时同样放行（后续 SWS 判断会暴露问题） */
    }

    /* 5. 总线分频 + 选 PLL：AHB=168(÷1)，APB1=42(÷4)，APB2=84(÷2)，SW=10(选PLL)
     *    ⚠ PPRE 编码：0xx=÷1, 100(4)=÷2, 101(5)=÷4, 110(6)=÷8, 111(7)=÷16
     *      历史上此处误写 (4UL << 10) 即 ÷2 → APB1 实为 84MHz，导致 CAN 位速率
     *      变成 1Mbps、TIM3 PWM 变成 40kHz（两者注释/文档都按 42MHz 设计）。
     *      离线交叉校验：python3 tools/timing_audit.py */
    /* 位段拆解：[4]HPRE=0 → AHB = SYSCLK÷1 = 168MHz；
     *          [10]PPRE1=101 → APB1 = 168÷4 = 42MHz（CAN/TIM4 挂这里，改它 CAN 波特率就变）；
     *          [13]PPRE2=100 → APB2 = 168÷2 = 84MHz（USART1/TIM1/TIM8 挂这里）；
     *          [0]SW=10 → 系统时钟源选 PLL（这一位生效时刻 = 跑完这行，之后的代码已跑在 168MHz）。 */
    RCC->CFGR = (0UL << 4) | (5UL << 10) | (4UL << 13) | (2UL << 0);

    /* 6. 等待 SWS=10（PLL 生效） */
    timeout = 1000000;                   /* 切换生效需要几个周期，给足有界等待 */
    while (((RCC->CFGR >> 2) & 0x3) != 0x2) {   /* SWS[3:2] = "当前实际时钟源"，10 = PLL */
        if (--timeout == 0) break;       /* 超时放行：说明没切成功，靠后续实测（timing_audit/串口）暴露 */
    }
}

/**
 * @brief  SysTick 1ms 周期中断初始化：为 millis()/delay_ms() 建立全系统时间基准
 * @return 无
 * @note   ① 重装值 168000−1 假设 HCLK=168MHz（SystemClock_Config() 已生效）；
 *         ② 运行期再次调用会把计数器清零，millis() 可能跳变/回退 → 只在启动期调一次；
 *         ③ 中断里只累加计数器，业务全部在任务层轮询（规则 §4）。
 */
void SysTick_Init(void)
{
    // SysTick->RVR = 16000UL - 1UL;   /* 168MHz / 168000 = 1ms */
    /* 重装值 = 每 1ms 需要的 HCLK 个数 − 1（硬件是从该值向下数到 0）：
     * 168MHz / 1000 = 168000。原注释写 16000 是旧时钟频率的残留值，已按 168MHz 修正。 */
    SysTick->RVR = 168000UL - 1UL;   /* 原 16000，168MHz / 168000 = 1ms */
    SysTick->CVR = 0;                /* 清当前计数值（写任何值都会清零），让下一个 1ms 从整点开始 */
    SysTick->CSR = SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE;
    /* CLKSOURCE=1 用 HCLK(168MHz)、TICKINT=1 数到 0 就产生 SysTick 中断、ENABLE=1 启动计数 */
}

static volatile uint32 g_ticks = 0;   /* ISR 与任务共享：必须 volatile（§3） */
/**
 * @brief  SysTick 中断服务程序：每 1ms 把全局毫秒计数 +1
 * @return 无
 * @note   由启动文件的中断向量表直接调用，**不要在应用代码里调用它**。
 *         只做一次自增（变量是 volatile，保证任务侧可见）；禁止在此加延时、打印或任何
 *         可能阻塞的逻辑 —— 它决定整个调度层的时基精度。
 */
void SysTick_Handler(void)
{
    g_ticks++;                   /* 每 1ms 进一次：这是全系统唯一的时间基准累加器 */
}

/**
 * @brief  读系统运行毫秒数
 * @return 上电以来经过的毫秒数，0 ~ 4294967295（约 49.7 天后回绕到 0）
 * @note   ① 读 32 位变量是单条指令，天然原子，无需关中断；
 *         ② 调用方一律写成 (now - t0) >= 周期 的差值形式，天然抗回绕；直接比大小会
 *           在回绕点出错；
 *         ③ 时基来自 SysTick 中断，长时间关中断会让时间“走慢”。
 */
uint32 millis(void)
{
    return g_ticks;              /* 读 32 位变量的单条指令是原子的；调用方一律用 (now - t0) 差值，天然抗溢出回绕 */
}

/**
 * @brief  阻塞忙等指定毫秒数（基于 millis() 差值，不受任务抖动影响）
 * @param  ms 要等待的毫秒数；0 表示立即返回
 * @return 无
 * @note   ⚠ **只允许在初始化阶段使用**（开机自检、上电指示）。在 Task_10ms/20ms/100ms
 *         里调用会挤掉 CAN 轮询、造成丢帧（.clinerules §3 历史事故）。
 *         需要“等一会”的周期行为请改用 millis() 做非阻塞时间片。
 */
void delay_ms(uint32 ms)
{
    uint32 start = millis();     /* 记下起点毫秒数 */
    while ((millis() - start) < ms) {}
    /* 阻塞忙等：仅允许初始化阶段（自检/上电指示）使用；Task_* 里调用会让 CAN 丢帧（规则 §3） */
}

/* ================================================================
 *  USART1 调试串口（PA9=TX, PA10=RX, 115200 8N1）
 * ================================================================ */
/**
 * @brief  调试串口 USART1 初始化：PA9=TX / PA10=RX（AF7），115200 8N1，轮询收发
 * @return 无
 * @note   ① BRR=0x02D9 对应实际 115226bps（误差 0.023%），前提是 APB2=84MHz；
 *         ② 只置 UE/TE/RE，其余位保持复位默认值 → 即 8 数据位/无校验/1 停止位；
 *         ③ 不用中断与 DMA：发送靠 uart_putc() 轮询 TXE，接收本工程未使用；
 *         ④ 板上经 CH340 引出，PC 端按 115200 8N1 打开即可。
 */
void UART1_Init(void)
{
    gpio_enable_clock(GPIOA);                                     /* 开 GPIOA 时钟（PA9/PA10 属 GPIOA） */
    gpio_mode(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_MODE_AF);     /* 两脚设为"复用功能"模式，交给片上外设驱动 */
    gpio_af(GPIOA, GPIO_PIN_9, 7);                                /* PA9 = TX，AF7 = USART1_TX（查数据手册复用表） */
    gpio_af(GPIOA, GPIO_PIN_10, 7);                               /* PA10 = RX，AF7 = USART1_RX */

    RCC->APB2ENR |= RCC_USART1EN;        /* 打开 USART1 外设时钟（USART1 挂在 APB2 = 84MHz） */

    /* 波特率 115200：USART1 时钟 = APB2 = 84MHz
    * USARTDIV = 84e6 / (16 × 115200) = 45.57 → BRR = 0x02D9 */
    /* BRR 拆解：高 12 位 = 整数部分 45、低 4 位 = 小数部分 9/16=0.5625 → 45.5625，
     * 实际波特率 = 84MHz/(16×45.5625) = 115226，误差 0.023%（远小于 2.5% 容限，通信稳定）。 */
    USART1->BRR = 0x02D9;



    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    /* UE=开 USART、TE=开发送器（TX）、RE=开接收器（RX）。
     * 其余位保持 0 即"8 数据位 + 无校验 + 1 停止位"(8N1) —— 这是复位默认值，所以无需另配。 */
}

/**
 * @brief  阻塞发送一个字节（等 TXE 后写 DR）
 * @param  c 待发送字符；只取低 8 位（DR 高 8 位保留）
 * @return 无
 * @note   ① 必须先调用 UART1_Init()，否则 TXE 不会按预期置位、可能在此死等；
 *         ② 115200 下一个字节约 87us，一行 60 字符 ≈ 5ms → 禁止在 10ms 任务里连续
 *           打长日志（规则 §3：任务内禁止长阻塞）；
 *         ③ 只保证“装进移位寄存器”，不等待发送完成（TC）。
 */
void uart_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) {}
    /* TXE=发送数据寄存器空：上一字节已搬进移位寄存器才会置 1，等它才不会覆盖未发完的数据 */
    USART1->DR = (uint32)(uint8)c;   /* 写 DR 即触发发送；DR 是 9 位寄存器，高 8 位保留，故只取低 8 位 */
}

/**
 * @brief  阻塞发送以 '\0' 结尾的字符串
 * @param  s 字符串首地址，**不可为 NULL**（空指针保护由调用方负责）
 * @return 无
 * @note   ① 逐字节调用 uart_putc()，因此同样受“禁止长阻塞”约束；
 *         ② 换行请用 "\r\n"（PC 串口助手默认不自动补 \r）；
 *         ③ 不做转义/格式化，原样输出。
 */
void uart_puts(const char *s)
{
    while (*s) {                 /* 逐字节直到字符串结束符 '\0'（'\0' 为 0，循环自然终止） */
        uart_putc(*s++);         /* 先发当前字符，再把指针后移一位 */
    }
}

/**
 * @brief  原样输出 fmt 字符串（**不做格式化**）
 * @param  fmt 要输出的字符串，不可为 NULL
 * @param  ... 可变参数，**被完全忽略**
 * @return 无
 * @note   ⚠ 名字虽叫 printf，但 nano.specs 下 vsnprintf 会 HardFault（规则 §3），所以
 *         这里只做 uart_puts(fmt)。写 uart_printf("v=%d", v) 会原样打出 "v=%d"。
 *         拼数字请用 uart_puts() + uart_put_uint()/uart_put_sint() 组合。
 */
void uart_printf(const char *fmt, ...)
{
    uart_puts(fmt);              /* 不做格式化：nano.specs 下 vsnprintf 会 HardFault（规则 §3），这里只原样输出 */
}

/* 自写 itoa：nano.specs 下 vsnprintf 会 HardFault，所以打数字一律走这里（规则 §3） */
/**
 * @brief  以十进制输出一个无符号 32 位整数（自写 itoa，不依赖 newlib 格式化）
 * @param  v 待输出的值，0 ~ 4294967295（最多 10 位十进制）
 * @return 无
 * @note   ① 内部缓冲区固定 10 字节，恰好容纳 uint32 的最大位数；
 *         ② 0 走特判分支（否则取模循环一次都不执行，会打出空串）；
 *         ③ 阻塞输出（逐字节 uart_putc），行宽请自行控制。
 */
void uart_put_uint(uint32 v)
{
    char  buf[10];      /* uint32 最大 4294967295 = 10 位 */
    uint8 i = 0;        /* 已写进 buf 的位数（也是逆序缓冲区的下标） */

    if (v == 0u) {              /* 特判 0：下面的取模循环一次都不会执行，会打出空串 */
        uart_putc('0');
        return;
    }
    while (v > 0u && i < 10) {              /* 反复取最低位 → 缓冲区里是"倒着"的数字 */
        buf[i] = (char)('0' + (uint8)(v % 10u));   /* 数字 0~9 加字符 '0' 的 ASCII 码即得对应字符 */
        i++;                                /* 缓冲区下标前进 */
        v /= 10u;                           /* 去掉已经取出的最低位（整除 10） */
    }
    while (i > 0u) {                        /* 逆序输出缓冲区 → 得到正序十进制字符串 */
        i--;
        uart_putc(buf[i]);
    }
}

/**
 * @brief  以十进制输出一个有符号 32 位整数（负数带 '-' 前缀）
 * @param  v 待输出的值，−2147483648 ~ 2147483647
 * @return 无
 * @note   负数先输出 '-' 再把绝对值交给 uart_put_uint()；用 (uint32)(-v) 的写法保证
 *         INT32_MIN 取负不溢出。正数不输出 '+' 号。
 */
void uart_put_sint(sint32 v)
{
    if (v < 0) {                        /* 负数：先打负号，再把绝对值交给无符号版本 */
        uart_putc('-');
        uart_put_uint((uint32)(-v));    /* -v 先转 uint32，避免 INT32_MIN 取负溢出 */
    } else {
        uart_put_uint((uint32)v);       /* 非负直接按无符号打印 */
    }
}

/* ================================================================
 *  LED: LED0=PF9（低电平点亮）—— 平台层 DIO 抽象
 *  引脚映射与 BSRR 写全部收在本文件内；上层只调 Led_Init()/Led_Set()
 *  （§4：应用层/调度层禁止直接操作寄存器，故不再对外提供 LED_ON/LED_OFF 寄存器宏）
 *  换板子：改这里的 LED_PORT/LED_PIN 即可（platform.h 不再暴露寄存器宏）。
 * ================================================================ */
#define LED_PORT GPIOF
#define LED_PIN  GPIO_PIN_9

/**
 * @brief  板载 LED0（PF9）初始化：推挽输出，上电默认熄灭
 * @return 无
 * @note   本板 LED 为“阳极接 VCC、阴极接 GPIO”接法 → **低电平点亮**（所以 Led_Set(TRUE)
 *         写的是 BSRR 高 16 位的清 0）。换板子只改本文件的 LED_PORT/LED_PIN。
 */
void Led_Init(void)
{
    gpio_enable_clock(LED_PORT);                          /* 开 LED 所在端口（GPIOF）时钟 */
    gpio_mode(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT);       /* PF9 设为推挽输出 */
    Led_Set(FALSE);                     /* 上电熄灭 */
}

/**
 * @brief  设置 LED0（PF9）亮灭
 * @param  on TRUE = 点亮（输出低电平）／FALSE = 熄灭（输出高电平）
 * @return 无
 * @note   用 BSRR 原子写（写 1 才动作，无需读-改-写），任务与中断里都可安全调用。
 *         LED0 的语义由上层决定（AEB：预警闪/制动常亮；heartbeat_v1：命令指示），
 *         平台层不解释语义。
 */
void Led_Set(boolean on)
{
    if (on) LED_PORT->BSRR = (uint32)LED_PIN << 16;   /* 输出低 -> 点亮 */
    else    LED_PORT->BSRR = (uint32)LED_PIN;         /* 输出高 -> 熄灭 */
    /* BSRR 是"写 1 才动作"的原子寄存器：低 16 位 = 置 1（输出高），高 16 位 = 清 0（输出低）。
     * 本板 LED 是"阴极接 GPIO"的接法，所以 GPIO 输出低电平才点亮 —— 故点亮写高 16 位。 */
}

/* ================================================================
 *  LED1: LED1=PF10（低电平点亮，板载绿）—— 同一套平台 DIO 抽象
 *  heartbeat_v1 握手状态指示专用：LED0 已被 RDK 蜂鸣/刹停语义占用，
 *  两个语义不共用一盏灯（避免"灯亮到底表示哪个"的歧义）。
 *  引脚映射与 BSRR 写仍全部收在 platform.c 内，上层只调 Led1_Set()。
 * ================================================================ */
#define LED1_PORT GPIOF
#define LED1_PIN  GPIO_PIN_10

/**
 * @brief  板载 LED1（PF10，绿色）初始化：推挽输出，上电默认熄灭
 * @return 无
 * @note   与 LED0 同一套平台 DIO 抽象、同一端口 GPIOF（重复开时钟是幂等的）。
 *         LED1 专用于 heartbeat_v1 的**链路状态**指示，与 LED0 的命令语义隔离。
 */
void Led1_Init(void)
{
    gpio_enable_clock(LED1_PORT);                         /* 与 LED0 同端口（GPIOF），重复开时钟是幂等的 */
    gpio_mode(LED1_PORT, LED1_PIN, GPIO_MODE_OUTPUT);     /* PF10 设为推挽输出 */
    Led1_Set(FALSE);                    /* 上电熄灭 */
}

/**
 * @brief  设置 LED1（PF10）亮灭
 * @param  on TRUE = 点亮（输出低电平）／FALSE = 熄灭（输出高电平）
 * @return 无
 * @note   同样走 BSRR 原子写。两盏灯语义隔离（LED0=命令/制动，LED1=链路状态），
 *         所以两个状态可以同时表达（例如 HOLD 时两灯都常亮）。
 */
void Led1_Set(boolean on)
{
    if (on) LED1_PORT->BSRR = (uint32)LED1_PIN << 16;   /* 输出低 -> 点亮（低电平点亮接法） */
    else    LED1_PORT->BSRR = (uint32)LED1_PIN;         /* 输出高 -> 熄灭 */
}

/* ================================================================
 *  newlib 堆支持（-specs=nano.specs 需要 _sbrk，勿删）
 *  ⚠ 工程不使用 malloc（只靠链接脚本的 minimum heap），保留桩函数仅为满足 newlib
 * ================================================================ */
extern char end;   /* 由链接脚本定义：.bss 段结束后的第一个地址 = 堆的起点 */
/**
 * @brief  newlib 堆分配桩（-specs=nano.specs 链接时必需，**勿删**）
 * @param  incr 本次请求的字节增量（ptrdiff_t，可为负；newlib 在 malloc 时只传正数）
 * @return 分配前的堆顶地址（即本次分配到的起点）
 * @note   ① 工程不使用 malloc，本函数仅为满足链接器/库需求；
 *         ② 首次调用时以链接脚本的 end 符号为堆起点（惰性初始化）；
 *         ③ **不做越界检查**（不与当前 SP 比较）：真去 malloc 超额会静默踩栈，
 *           因此业务代码禁止引入动态分配。
 */
void *_sbrk(ptrdiff_t incr)
{
    static char *heap_end = 0;      /* 堆当前末尾（首次调用时惰性初始化） */
    char *prev_heap_end;            /* 用来返回"本次分配到的起点" */

    if (heap_end == 0) {
        heap_end = &end;            /* 第一次调用：堆从链接脚本给的 end 符号处开始 */
    }
    prev_heap_end = heap_end;       /* 记下本次分配的起始地址 */
    heap_end += incr;               /* 堆顶向后推进 incr 字节 */
    return (void *)prev_heap_end;   /* 返回分配到的地址（newlib 需要这个旧值） */
}

