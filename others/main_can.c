// /**
//  * main.c - 主程序（普通 CAN 版，寄存器级）
//  *
//  * 适配硬件：M144Z-M4 / 正点原子 STM32F407 Mini 板（F407ZGT6, LQFP144）
//  *   CAN1  : PA11(RX)/PA12(TX)，500kbps，需外接 TJA1050 收发器
//  *   调试口 : USART1 115200（板载 CH340，USB 直连看日志）
//  *   LED   : LED0=PF9（低电平点亮）
//  *
//  * ────────────────────────────────────────────────────────────
//  * 与遥控版（edge_ai_setup_v2.sh 默认生成的 main.c）的区别：
//  *   ❌ 删除 0x100 遥控控制帧 / 0x102 AI心跳 / 0x101 状态帧 / 0x103 心跳
//  *   ❌ 删除 500ms 心跳超时刹车、g_control 控制执行
//  *   ❌ 删除电机(TB6612)/舵机(MG996R)/编码器(TIM4/TIM8) 全部外设与任务
//  *   ✅ 只保留：CAN1 收发 + USART1 日志 + LED
//  *
//  * 行为（普通 CAN 测试）：
//  *   1. 上电初始化 CAN1 @500kbps，串口打印 boot 信息
//  *   2. 每 500ms 自动发送一帧测试帧 0x111：{计数器,'C','A','N',...}
//  *   3. 收到总线上任意标准帧，串口打印 ID/长度/数据（可以自收自发验证）
//  *
//  * 编译：cd 04_stm32 && make       烧写：make flash（ST-Link）
//  * 本文件只替换 04_stm32/Src/main.c 即可，其余驱动文件原样保留
//  * （未引用的电机/舵机/编码器代码会被链接器 --gc-sections 丢弃）。
//  */

// #include "platform.h"
// #include "can_driver.h"
// #include <stddef.h>

// /* ===== 普通 CAN 测试参数（可自行修改） ===== */
// #define TX_TEST_ID       0x111u   /* 周期发送的测试帧 ID */
// #define TX_TEST_PERIOD_MS 500u    /* 发送周期 */

// static uint8 g_tx_counter = 0;

// /* ================================================================
//  *  时钟 / 平台基础服务（寄存器级，与遥控版完全一致）
//  * ================================================================ */
// void SystemClock_Config(void)
// {
//     /* 使能 HSE(8MHz) */
//     RCC->CR |= (1UL << 16);
//     uint32 timeout = 1000000;
//     while (!(RCC->CR & (1UL << 17))) {   /* HSERDY */
//         if (--timeout == 0) break;
//     }

//     /* 等待 PLL 就绪标志复位 */
//     while (RCC->CR & (1UL << 25)) {}

//     /* PLL: VCO = 8MHz / M(8) * N(336) = 336MHz
//      *      PLLCLK = 336 / P(2) = 168MHz
//      * PLLCFGR: PLLM=8 | PLLN=336<<6 | PLLP=0(÷2)<<16 | PLLQ=7<<24 | PLLSRC=HSE(1)<<22
//      */
//     RCC->PLLCFGR = (8UL) | (336UL << 6) | (0UL << 16) | (7UL << 24) | (1UL << 22);

//     /* 使能 PLL */
//     RCC->CR |= (1UL << 24);
//     timeout = 1000000;
//     while (!(RCC->CR & (1UL << 25))) {   /* PLLRDY */
//         if (--timeout == 0) break;
//     }

//     /* Flash 等待周期 5WS + 预取/缓存使能 */
//     *FLASH_ACR &= ~FLASH_ACR_LATENCY_MASK;
//     *FLASH_ACR |= FLASH_ACR_LATENCY_5WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

//     /* 总线分频：AHB=1, APB1=4(42MHz), APB2=2(84MHz)
//      * CFGR: HPRE=0(÷1) | PPRE1=101(÷4)<<10 | PPRE2=100(÷2)<<13 | SW=PLL(10)<<0
//      */
//     RCC->CFGR = (0UL) | (5UL << 10) | (4UL << 13);
//     RCC->CFGR = (0UL) | (5UL << 10) | (4UL << 13) | (2UL << 0);

//     /* 等待 PLL 切换完成 */
//     timeout = 1000000;
//     while (((RCC->CFGR >> 2) & 0x3) != 0x2) {
//         if (--timeout == 0) break;
//     }
// }

// void SysTick_Init(void)
// {
//     SysTick->RVR = 168000UL - 1UL;   /* 168MHz / 168000 = 1ms */
//     SysTick->CVR = 0;
//     SysTick->CSR = SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE;
// }

// static volatile uint32 g_ticks = 0;
// void SysTick_Handler(void)
// {
//     g_ticks++;
// }

// uint32 millis(void)
// {
//     return g_ticks;
// }

// void delay_ms(uint32 ms)
// {
//     uint32 start = millis();
//     while ((millis() - start) < ms) {}
// }

// /* ================================================================
//  *  USART1 调试串口（PA9=TX, PA10=RX, 115200 8N1）
//  * ================================================================ */
// void UART1_Init(void)
// {
//     gpio_enable_clock(GPIOA);
//     gpio_mode(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_MODE_AF);
//     gpio_af(GPIOA, GPIO_PIN_9, 7);
//     gpio_af(GPIOA, GPIO_PIN_10, 7);

//     RCC->APB2ENR |= RCC_USART1EN;

//     /* 波特率 115200: USARTDIV = 84MHz / 115200 = 729.167
//      * BRR = 729<<4 | 3 = 11667 = 0x2D93 */
//     USART1->BRR = 0x2D93;

//     USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
// }

// void uart_putc(char c)
// {
//     while (!(USART1->SR & USART_SR_TXE)) {}
//     USART1->DR = (uint32)(uint8)c;
// }

// void uart_puts(const char *s)
// {
//     while (*s) {
//         uart_putc(*s++);
//     }
// }

// void uart_printf(const char *fmt, ...)
// {
//     char buf[128];
//     va_list ap;
//     va_start(ap, fmt);
//     vsnprintf(buf, sizeof(buf), fmt, ap);
//     va_end(ap);
//     uart_puts(buf);
// }

// /* ================================================================
//  *  LED: LED0=PF9（M144Z-M4 Mini板，低电平点亮）
//  * ================================================================ */
// static void MX_LED_Init(void)
// {
//     gpio_enable_clock(GPIOF);
//     gpio_mode(GPIOF, GPIO_PIN_9, GPIO_MODE_OUTPUT);
//     LED_OFF();
// }

// /* ================================================================
//  *  CAN 接收回调（普通 CAN：收到什么打印什么）
//  * ================================================================ */
// static void App_CanRxCallback(uint16 can_id, const uint8* data, uint8 len)
// {
//     uint8 i;

//     uart_printf("\r\n[RX] id=0x%03X len=%u data=", can_id, len);
//     for (i = 0; i < len; i++) {
//         uart_printf("%02X ", data[i]);
//     }
//     uart_printf("\r\n");

//     /* 收到测试帧 ID 时 LED 闪一下，方便无串口时观察 */
//     if (can_id == TX_TEST_ID) {
//         LED_ON();
//         delay_ms(20);
//         LED_OFF();
//     }
// }

// /* ================================================================
//  *  普通 CAN 测试任务（周期发送测试帧）
//  * ================================================================ */
// static void PlainCan_TxTest(void)
// {
//     uint8 data[8];

//     data[0] = g_tx_counter++;   /* 计数器 */
//     data[1] = 'C';
//     data[2] = 'A';
//     data[3] = 'N';
//     data[4] = 0x00;
//     data[5] = 0x00;
//     data[6] = 0x00;
//     data[7] = 0x00;

//     if (Can_Send(TX_TEST_ID, data, 8)) {
//         uart_printf("[TX] id=0x%03X len=8 counter=%u\r\n",
//                     TX_TEST_ID, (uint32)(g_tx_counter - 1));
//         LED_ON();
//         delay_ms(10);
//         LED_OFF();
//     } else {
//         uart_printf("[WARN] TX mailbox busy (总线没接/没终端电阻?)\r\n");
//     }
// }

// /* ================================================================
//  *  newlib 堆支持（-specs=nano.specs 需要 _sbrk，勿删）
//  * ================================================================ */
// extern char end;   /* 由链接脚本定义 */
// void *_sbrk(ptrdiff_t incr)
// {
//     static char *heap_end = 0;
//     char *prev_heap_end;

//     if (heap_end == 0) {
//         heap_end = &end;
//     }
//     prev_heap_end = heap_end;
//     heap_end += incr;
//     return (void *)prev_heap_end;
// }

// /* ================================================================
//  *  main（普通 CAN 版：无遥控协议、无电机舵机，只做收发测试）
//  * ================================================================ */
// int main(void)
// {
//     /* ===== 平台基础 ===== */
//     SystemClock_Config();
//     SysTick_Init();
//     UART1_Init();
//     MX_LED_Init();

//     /* ===== CAN1 初始化（500kbps，PA11/PA12 + 外部 TJA1050） ===== */
//     if (!Can_Init()) {
//         uart_printf("[ERR] CAN1 init FAIL\r\n");
//         while (1) {
//             LED_ON(); delay_ms(200);
//             LED_OFF(); delay_ms(200);
//         }
//     }

//     /* ===== 注册接收回调（轮询方式，Can_ProcessRx 驱动） ===== */
//     Can_RegisterRxCallback(App_CanRxCallback);

//     uart_printf("\r\n=== edge_ai_car STM32 plain CAN demo (M144Z-M4) ===\r\n");
//     uart_printf("CAN1=500kbps | USART1=115200 | TX test id=0x%03X every %ums\r\n",
//                 TX_TEST_ID, (uint32)TX_TEST_PERIOD_MS);
//     uart_printf("boot OK @168MHz, waiting CAN frames...\r\n");

//     /* ===== 主循环：1ms 轮询接收 + 500ms 周期发送 =====
//      * 普通 CAN 无遥控任务，Can_ProcessRx() 轮询间隔短一点即可
//      */
//     uint32 last_poll = 0, last_tx = 0;

//     while (1)
//     {
//         uint32 now = millis();

//         /* 1ms 轮询收帧（FIFO0） */
//         if (now - last_poll >= 1) {
//             last_poll = now;
//             Can_ProcessRx();
//         }

//         /* 500ms 发一帧测试帧 */
//         if (now - last_tx >= TX_TEST_PERIOD_MS) {
//             last_tx = now;
//             PlainCan_TxTest();
//         }
//     }
// }
