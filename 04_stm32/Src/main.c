/**
 * main.c - 平台服务 + 调度层 + 应用层（AUTOSAR CP 四层，见 .clinerules §4）
 *          当前阶段：CAN 联调（§6.1 ①回环自测 → ②Normal 抓包 → ③与板端 AEB 联调）
 *
 * 适配硬件：M144Z-M4 / 正点原子 STM32F407 Mini 板（F407ZGT6, LQFP144）
 *  电机驱动 : TB6612FNG 双通道 H 桥
 *             A通道(左轮): PWMA=PA6(TIM3_CH1)  AIN1=PB0  AIN2=PB1
 *             B通道(右轮): PWMB=PA7(TIM3_CH2)  BIN1=PB4  BIN2=PB5
 *             STBY=PB3（必须拉高）
 *  电机     : JGB37-520 直流减速电机 ×2（后轮驱动）
 *  舵机     : MG996R 前轮转向，50Hz，PA8=TIM1_CH1，5~6V 独立供电
 *  调试口   : USART1 115200（板载 CH340，USB 直连看日志）
 *  LED      : LED0=PF9 + LED1=PF10（均低电平点亮；LED1=heartbeat_v1 握手状态指示）
 *
 * 当前行为：
 *   1) CAN1 初始化：默认 Normal（Can_Init，TJA1050 VCC=5V + 两端 120Ω + 显式共地）；
 *      自测镜像（make CAN_LOOPBACK=1）走静默回环（BTR.LBKM|SILM，无需收发器
 *      /对端/终端电阻），上电自发 0x111 并在 10ms 轮询里回读判 PASS/FAIL
 *   2) 应用层 AEB（§5）：收 0x123 制动等级 0解除/1预警/2部分制动/3急刹+锁存；
 *      收 0x124 障碍物距离 → 只记录并打印（不驱动动作）
 *   3) 调度层 Task_20ms：左右轮编码器测速(cm/s) → 平均 → ×0.036 = km/h → 发 0x125
 *      （Normal 模式发送；回环模式只打印，可先验证车速链路而无需 CAN 硬件）
 *   4) 失效保护（刹车优先、可恢复）：0x123 超时只告警并保持锁存，仅等级 0 解除
 *   5) RDK 行车链路（0x100/0x101/0x102，heartbeat_v1）：上电即 PWM=0 + 短路制动（STATIC），
 *      每 200ms 恒发 0x101 等握手（**无次数上限**，60s 未握手 → LOCK）；累计 2 帧有效
 *      0x102 应答 → CRUISE（唯一可驱动状态，软启动到巡航 PWM）。CRUISE 按有效 0x100 的
 *      byte0 执行：00 停车 PWM=0 / 01 巡航 / 02 预警减速 + LED0 慢闪 / 03 刹停 + LED0 快闪。
 *      判活**只看 0x100**：静默 ≥ hb_timeout_ms(300) → 本地刹停 → HOLD（0x102 在不在都一样）；HOLD 里
 *      唯一恢复条件 = 收到有效 0x100（60s 未恢复 → LOCK）；LOCK：PWM=0 + LED 常亮 + 停发
 *      0x101，需复位。byte3 与上帧相同的重复帧一律丢弃，且**不刷新**判活时间戳。
 *      参数全部集中在 Inc/rdk_config.h + Src/rdk_config.c，本文件不写死数字。
 *   运动 demo（MotionDemo_*）仍 #if 0 留档：其 delay_ms 阻塞会导致 CAN 丢帧
 *
 * 说明：
 *   - 舵机角度符号（+/-30°对应左/右）取决于安装方向，若转向方向反了，
 *     把 Servo_SetAngle 的参数取反即可。
 *   - Motor_SetSpeed 内部带 20ms 斜坡限幅，起步不会窜。
 *   - 替换 04_stm32/Src/main.c 后：cd 04_stm32 && make && make flash
 */

#include "platform.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "encoder.h"        /* 编码器抽象层：0x125 车速来源（两轮测速） */
#include "can_driver.h"     /* CAN 驱动抽象层（含 can_protocol.h） */
#include "rdk_config.h"     /* RDK 行车链路参数（config 集中定义，见 Src/rdk_config.c） */

/* ================================================================
 *  CAN 自测模式开关（规则 §6.1：先回环自测①，再切 Normal 联调②③）
 *  默认 = Normal：本文件不定义该宏，下面的 #ifndef 兜底给 0 → 联调镜像就是默认镜像。
 *  自测镜像**不改源码**，构建期注入（见 Makefile）：
 *      make clean && make CAN_LOOPBACK=1    # 等价 -DCAN_LOOPBACK_SELFTEST=1
 *      make clean && make                   # 默认 Normal（AEB 联调）
 *    1 = 静默回环（Can_InitLoopback：BTR.LBKM|SILM=1，无需收发器/对端）
 *        上电发 0x111 + Task_10ms 轮询回读 → 每秒打印 PASS/FAIL/TxErr/RxErr/BusOff
 *    0 = Normal  （Can_Init：接 TJA1050(VCC 5V) + 两端 120Ω + 共地，与板端联调）
 *  两种模式下 Task_20ms 都会测速并打印；只有 Normal 才真正发 0x125
 *  ⚠ SILM/LBKM 是 CAN_BTR[31]/[30]（RM0090 30.9.2）**不是 MCR**；只置 SILM 是
 *    "只收不发"，自发自收必须 LBKM|SILM（见 can_driver.c can_init_internal()）。
 * ================================================================ */
/* 临时想在源码里启用自测就放开下面一行（联调前记得收回，避免错烧自测镜像）： */
// #define CAN_LOOPBACK_SELFTEST   1
#ifndef CAN_LOOPBACK_SELFTEST
#define CAN_LOOPBACK_SELFTEST   0
#endif

/* ================================================================
 *  平台层实现已迁出（本轮分层补全）
 *  时钟 / SysTick / millis / delay_ms / USART1 / LED / _sbrk 现定义在
 *  Src/platform.c —— AUTOSAR CP 四层框架的"平台层"（.clinerules §4）。
 *  本文件只保留：外设 PWM 初始化（MX_TIM3/MX_TIM1，本轮按零时序变化暂留
 *  主流程）、应用段（AEB）、调度段（Task_*）、main() 装配；
 *  以上均不再直接操作寄存器。
 * ================================================================ */

/* ================================================================
 *  TIM3: 双通道 PWM 20kHz（TB6612 PWMA/PWMB）
 *   APB1=42MHz, TIM3 时钟=84MHz, PSC=83 -> 1MHz, ARR=49 -> 20kHz
 * ================================================================ */
/**
 * @brief  TIM3 双通道 PWM 初始化（20kHz，TB6612 的 PWMA/PWMB）
 * @return 无
 * @note   ① PA6→AF2=TIM3_CH1(PWMA/左轮)、PA7→AF2=TIM3_CH2(PWMB/右轮)；
 *         ② PSC=83/ARR=49 对应 20kHz：前提是 APB1=42MHz（timing_audit 有契约项），
 *           改频率会同时影响 CCR 换算（speed_to_ccr 按 ARR 现场算，所以不会饱和）；
 *         ③ 本函数只配波形，**不使能电机**：CCR1/CCR2=0，方向脚由 Motor_Init() 管；
 *         ④ 属外设装配，按“零时序变化”原则暂留 main.c，由 main() 调用。
 */
static void MX_TIM3_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_TIM3EN;      /* TIM3 挂在 APB1；开时钟后才能写它的寄存器 */

    gpio_enable_clock(GPIOA);                                       /* 开 GPIOA 时钟 */
    gpio_mode(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF);         /* PA6/PA7 设复用（PWM 输出脚） */
    gpio_af(GPIOA, GPIO_PIN_6, 2);                                  /* PA6 → AF2 = TIM3_CH1 = PWMA(左轮) */
    gpio_af(GPIOA, GPIO_PIN_7, 2);                                  /* PA7 → AF2 = TIM3_CH2 = PWMB(右轮) */

    TIM3->PSC = 83;        /* 预分频：84MHz/(83+1) = 1MHz 计数时钟（TIM3 定时器时钟 = APB1×2 = 84MHz） */
    TIM3->ARR = 49;        /* 自动重装：1MHz/(49+1) = 20kHz PWM 频率（TB6612 支持、听不见啸叫） */

    // TIM3->CCMR1 = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE |   /* CH1: PWM mode 1 */
    //               TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE;    /* CH2: PWM mode 1 */

    TIM3->CCMR1 = (0x6UL << 4) | TIM_CCMR1_OC1PE |     /* CH1: OC1M[6:4]=110 */
                  (0x6UL << 12) | TIM_CCMR1_OC2PE;     /* CH2: OC2M[14:12]=110 */
    /* OCxM=110 = PWM 模式 1：计数值 < CCRx 时输出高电平（占空比 = CCRx/(ARR+1)）；
     * OCxPE=预装载使能：CCR 的修改在下一个周期才生效，避免改占空比时出现半周期毛刺 */


    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;   /* 打开两个通道的输出使能（不开则引脚不出波形） */
    TIM3->CCR1 = 0;                               /* 左轮占空比 0 = 停转（上电安全态） */
    TIM3->CCR2 = 0;                               /* 右轮占空比 0 */
    TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;       /* CEN=启动计数；ARPE=ARR 预装载（改频率不会半周期跳变） */
}

/* ================================================================
 *  TIM1: 舵机 PWM 50Hz（PA8=TIM1_CH1, AF1）
 *   APB2=84MHz, TIM1 时钟=168MHz, PSC=167 -> 1MHz, ARR=19999 -> 50Hz
 *   高级定时器必须置 BDTR.MOE
 * ================================================================ */
/**
 * @brief  TIM1_CH1 舵机 PWM 初始化（50Hz / 20ms，PA8=AF1）
 * @return 无
 * @note   ① PSC=167 → 计数时钟 1MHz（1 计数 = 1us），ARR=19999 → 周期 20000us=50Hz，
 *           所以 CCR1 直接就是脉宽微秒数，上电 CCR1=1500（中位）；
 *         ② TIM1 是高级定时器：**必须置 BDTR.MOE=1**，否则输出被强制为无效电平、
 *           引脚一直为低、舵机不动（经典坑）；
 *         ③ 只初始化 PWM 通道，不改舵机姿态；角度由 Servo_SetAngle()/Servo_SetPulse()
 *           改 CCR1 决定。
 */
static void MX_TIM1_PWM_Init(void)
{
    RCC->APB2ENR |= RCC_TIM1EN;      /* TIM1 挂在 APB2（高级定时器） */

    gpio_enable_clock(GPIOA);                                  /* 开 GPIOA 时钟 */
    gpio_mode(GPIOA, GPIO_PIN_8, GPIO_MODE_AF);                /* PA8 设复用功能 */
    gpio_af(GPIOA, GPIO_PIN_8, 1);                             /* PA8 → AF1 = TIM1_CH1（舵机信号线） */

    TIM1->PSC = 167;       /* 预分频：168MHz/(167+1) = 1MHz → 1 个计数 = 1us（CCR 直接当微秒数用） */
    TIM1->ARR = 19999;     /* 自动重装：1MHz/(19999+1) = 50Hz（舵机标准重复频率） */

    // TIM1->CCMR1 = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 = (0x6UL << 4) | TIM_CCMR1_OC1PE;    /* OC1M[6:4]=110 → PWM mode 1 */

    TIM1->CCER = TIM_CCER_CC1E;        /* 打开 CH1 输出使能 */
    TIM1->CCR1 = 1500;                 /* 中位 */
    /* 1500 计数 = 1500us 高电平 = 舵机中位（0°），上电即打正、不会乱打方向 */
    TIM1->BDTR = TIM_BDTR_MOE;         /* 主输出使能，必须有 */
    /* 高级定时器特有：MOE=0 时所有输出通道被强制为无效电平 → 引脚一直是低，舵机不动（经典坑） */
    TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;   /* 启动计数 + ARR 预装载 */
}

/* ================================================================
 *  demo 动作序列（每个动作：串口提示 + 执行 + 停顿）
 *  ⚠ CAN 联调阶段禁用：内含 27 处 delay_ms 阻塞，
 *    会挤掉 10ms 轮询导致 CAN 丢帧（规则 §3 历史事故）。
 *    后续要用请先改造为非阻塞任务再启用。
 * ================================================================ */
#if 0
/**
 * @brief  运动 demo 的“停车”动作（PWM=0 + LED 闪一下）—— 禁用·留档（不参与编译）
 * @return 无
 * @note   与整个运动 demo 一起放在 `#if 0` 里：内含 delay_ms 阻塞，与 CAN 联调
 *         （10ms 轮询）冲突。要恢复请先改造成非阻塞任务（规则 §3），再放开宏。
 */
static void MotionDemo_Stop(void)
{
    Motor_SetSpeed(0);
    Led_Set(TRUE);
    delay_ms(600);
    Led_Set(FALSE);
}

/**
 * @brief  运动 demo 主循环（前进/刹车/后退/转向/原地转圈/舵机摆角）—— 禁用·留档（不参与编译）
 * @return 无（原实现是 while(1) 永不返回）
 * @note   同样在 `#if 0` 内：内含 27 处 delay_ms 阻塞，会挤掉 CAN 轮询造成丢帧。
 *         联调阶段禁用；若要单独验机械，建议烧一个独立镜像并拔掉 CAN 线。
 */
static void MotionDemo_Run(void)
{
    uart_puts("\r\n=== edge_ai_car 运动 demo (无 CAN) ===\r\n");
    uart_puts("电机 PWM=20kHz(TB6612) | 舵机=50Hz(MG996R)\r\n\r\n");

    while (1) {
        /* ---------- 舵机摆角测试（左右极限 + 回中） ---------- */
        Servo_SetAngle(15);  delay_ms(1200);
        Servo_SetAngle(0);   delay_ms(1200);
        Servo_SetAngle(-15); delay_ms(1200);
        Servo_SetAngle(0);   delay_ms(1200);
        /* ---------- 1. 直线前进 / 刹车 / 后退 ---------- */
        demo_forward(300);
        delay_ms(1500);

        Motor_EmergencyBrake();
        Led_Set(TRUE); delay_ms(1500); Led_Set(FALSE);
        Motor_ReleaseBrake();

        demo_reverse(-300);
        delay_ms(1500);

        Motor_EmergencyBrake();
        Led_Set(TRUE); delay_ms(1500); Led_Set(FALSE);
        Motor_ReleaseBrake();
        delay_ms(1500);
    }

    while (1)
    {
        /* ---------- 1. 直线前进 / 刹车 / 后退 ---------- */
        uart_puts("[1] 前进 60% x2s\r\n");
        Motor_SetSpeed(600);
        delay_ms(2000);

        uart_puts("[2] 刹车(短路制动)\r\n");
        Motor_EmergencyBrake();
        Led_Set(TRUE); delay_ms(800); Led_Set(FALSE);
        Motor_ReleaseBrake();

        uart_puts("[3] 后退 60% x2s\r\n");
        Motor_SetSpeed(-600);
        delay_ms(2000);

        uart_puts("[4] 停车\r\n");
        MotionDemo_Stop();

        /* ---------- 2. 前轮转向（舵机打角 + 前进，阿克曼式转弯） ---------- */
        uart_puts("[5] 左转弯：舵机左打 + 前进 40% x1.5s\r\n");
        Servo_SetAngle(-30);                 /* 若方向反了，改成 +30 */
        delay_ms(500);                        /* 等舵机转到位 */
        Motor_SetSpeed(400);
        delay_ms(1500);
        Motor_SetSpeed(0);
        Servo_SetAngle(0);                    /* 回中 */
        MotionDemo_Stop();

        uart_puts("[6] 右转弯：舵机右打 + 前进 40% x1.5s\r\n");
        Servo_SetAngle(30);                  /* 若方向反了，改成 -30 */
        delay_ms(500);
        Motor_SetSpeed(400);
        delay_ms(1500);
        Motor_SetSpeed(0);
        Servo_SetAngle(0);
        MotionDemo_Stop();

        /* ---------- 3. 差速原地转圈（左右轮反向，不依赖舵机） ---------- */
        uart_puts("[7] 原地左转：左轮反转/右轮正转 x1.2s\r\n");
        Motor_SetSpeedLR(-400, 400);
        delay_ms(1200);
        Motor_SetSpeedLR(0, 0);
        MotionDemo_Stop();

        uart_puts("[8] 原地右转：左轮正转/右轮反转 x1.2s\r\n");
        Motor_SetSpeedLR(400, -400);
        delay_ms(1200);
        Motor_SetSpeedLR(0, 0);
        MotionDemo_Stop();

        /* ---------- 4. 舵机摆角测试（左右极限 + 回中） ---------- */
        uart_puts("[9] 舵机摆角：+30 -> -30 -> 0\r\n");
        Servo_SetAngle(30);  delay_ms(800);
        Servo_SetAngle(-30); delay_ms(800);
        Servo_SetAngle(0);   delay_ms(500);

        uart_puts("\r\n==== 一轮演示完成，重新开始 ====\r\n\r\n");
    }
}
#endif /* 运动 demo 留档（CAN 联调期间禁用） */

/* ================================================================
 *  应用层：AEB 业务流程（规则 §4 应用层 / §5 协议行为）
 *  只调驱动抽象接口，不碰寄存器；状态在"回调 ↔ 调度任务"间共享 → 全部 volatile
 * ================================================================ */

/* ---- 回环自测态（仅 CAN_LOOPBACK_SELFTEST=1 时使用）---- */
#if CAN_LOOPBACK_SELFTEST
static volatile uint8   g_can_test_tx_counter = 0;    /* 待发自测帧计数（= 下一次要发的 counter） */
static volatile uint8   g_can_test_tx_last    = 0;    /* 最近一次真正发出去的 counter，供回读比对 */
static volatile uint32  g_can_test_rx_count   = 0;    /* 回读到的合法自测帧数（内容+计数均匹配） */
static volatile uint32  g_can_test_echo_bad   = 0;    /* 0x111 但内容/计数不符 → 脏帧计数 */
static volatile boolean g_can_test_pass       = FALSE;/* 本判定窗口内收到过 → PASS */

/* 发一帧自测帧（0x111）：先把"最近发出的 counter"记下来再发，
 * 这样回调里能用它判定"收到的确实是刚发出去的那一帧"。
 * 用法：先设置 g_can_test_tx_counter，再调用本函数。 */
/**
 * @brief  回环自测：发一帧 0x111 自测帧，并记下“最近发出的 counter”
 * @return 无
 * @note   ① 发之前先把 g_can_test_tx_counter 抄到 g_can_test_tx_last，回调里据此
 *           判定“收到的确实是刚发出去的那一帧”（防残留帧/噪声帧误判 PASS）；
 *         ② 调用前必须先设置 g_can_test_tx_counter（上电首帧为 1，之后每秒 +1）；
 *         ③ 仅 CAN_LOOPBACK_SELFTEST 镜像编译。
 */
static void CanTest_SendFrame(void)
{
    g_can_test_tx_last = g_can_test_tx_counter;
    Can_SendSelfTest(g_can_test_tx_counter);
}

#if CAN_PROTOCOL_BRAKE_V2
/* ---- ⑤ 回环 AEB 自测态（自发 0x123/0x124/0x125 → 回读断言解析/映射/失效保护）----
 * 一轮 10 步，每步 = Task_100ms 的 2 个拍（AEB_LB_STEP_MS）；步内"先发帧、下一拍再断言"，
 * 留出 Can_ProcessRx(10ms) + Aeb_ServicePartialBrake(20ms) 的执行时间。 */
#define AEB_LB_STEP_COUNT    10u   /* 一轮步数（见 Aeb_LbSendStepFrame） */
#define AEB_LB_LOST_STEP      8u   /* 失效保护步：发 level3 后静默 AEB_LB_LOST_WAIT_MS */
#define AEB_LB_LOST_TICKS     ((AEB_LB_LOST_WAIT_MS + AEB_LB_STEP_MS - 1u) / AEB_LB_STEP_MS)
#define AEB_LB_TEST_DIST_CM  123u  /* 自测载荷：0x124 距离 cm */
#define AEB_LB_TEST_KMH      123u  /* 自测载荷：0x125 车速 km/h */
#define AEB_LB_BAD_LEVEL       4u  /* 非法等级（>3，必须被解析层拒绝） */

static volatile uint8   g_lb_step            = 0;    /* 0 = 空闲/轮间；1..AEB_LB_STEP_COUNT */
static volatile uint8   g_lb_wait            = 0;    /* 步内静默等待拍数（失效保护步用） */
static volatile uint8   g_lb_tick            = 0;    /* Task_100ms 分频：每 2 拍推进 1 步 */
static volatile uint32  g_lb_round           = 0;    /* 已完成轮数 */
static volatile uint32  g_lb_round_ms        = 0;    /* 本轮计时起点 */
static volatile uint8   g_lb_round_fail      = 0;    /* 本轮失败步数 */
static volatile uint8   g_lb_last_round_fail = 0xFFu;/* 上一轮失败步数（0xFF = 还没跑完一轮）*/
static volatile uint32  g_lb_fail            = 0;    /* 累计失败步数 */
static volatile uint8   g_lb_fail_step       = 0;    /* 最近失败步号（0 = 无） */
static volatile sint32  g_lb_fail_exp        = -1;   /* 最近失败期望值 */
static volatile sint32  g_lb_fail_act        = -1;   /* 最近失败实际值 */
static volatile uint16  g_lb_speed_echo      = 0;    /* 0x125 回读还原值（小端） */
static volatile boolean g_lb_speed_echo_ok   = FALSE;/* 0x125 回读 DLC=8 且已收到 */
#endif /* CAN_PROTOCOL_BRAKE_V2 */

#endif /* CAN_LOOPBACK_SELFTEST */

/* ---- 共享：轮速观测（brake_v2 的 0x125 融合与 heartbeat_v1 的滑行距离积分都用）----
 * 由 Task_20ms 每拍写入（只在那里消费一次 Encoder_GetSpeed*）；各协议分支只读。*/
static volatile sint32  g_wheel_l_cmps    = 0;                 /* 左轮 cm/s（日志观测） */
static volatile sint32  g_wheel_r_cmps    = 0;                 /* 右轮 cm/s（日志观测） */

/* ---- AEB 状态（brake_v2，§5）----
 * ⚠ 仅当 CAN_PROTOCOL_BRAKE_V2=1 时编译（heartbeat_v1 分支不含 AEB）。*/
#if CAN_PROTOCOL_BRAKE_V2
static volatile uint8   g_brake_level     = BRAKE_LEVEL_NONE;  /* 最近一次生效的等级 */
static volatile boolean g_brake_latched   = FALSE;             /* 刹停锁存：仅等级 0 解除 */
static volatile boolean g_brake_rx_seen   = FALSE;             /* 是否收到过 0x123 */
static volatile uint32  g_brake_rx_ms     = 0;                 /* 最近一帧有效 0x123 时刻 */
static volatile uint16  g_obstacle_cm     = AEB_DIST_INVALID;  /* 0x124 最近距离 */
static volatile uint32  g_dist_rx_count   = 0;                 /* 0x124 合法帧累计 */
static volatile uint16  g_speed_kmh       = AEB_SPEED_INVALID; /* 0x125 上报值 */
static volatile boolean g_speed_valid     = FALSE;
static volatile boolean g_wheel_diff_warn = FALSE;             /* 左右轮一致性告警 */

/* 单轮读数合理性上限：10 m/s = 36km/h（远超小车极速）；超出或 NaN(float 比较恒假)
 * 视为该轮无效 → 退化为"用有效那轮"，两轮都无效才上报 0xFFFF */
#define AEB_WHEEL_CMPS_MAX   1000.0f
/**
 * @brief  单轮线速度读数合理性判定（挡掉异常值与 NaN）
 * @param  v 某一路编码器算出的线速度，单位 cm/s（float32）
 * @return TRUE = 读数可用（落在 ±1000 cm/s 内，即 ±10m/s ≈ ±36km/h）；
 *         FALSE = 不可用（越界或 NaN）
 * @note   用"区间内"比较而不是"是否等于 NaN"：NaN 与任何数比较都为假，
 *         所以这一个条件就能同时挡掉 NaN 与物理上不可能的读数。
 *         判定失败的轮子被排除，两轮都失败才把 0x125 上报成 0xFFFF。
 */
static boolean wheel_reading_valid(float32 v)
{
    /* 落在 (−1000, +1000) 内才算有效：这个比较同时挡掉 NaN（NaN 与任何数比较都为假） */
    return ((v > -AEB_WHEEL_CMPS_MAX) && (v < AEB_WHEEL_CMPS_MAX)) ? TRUE : FALSE;
}

/* ---- 制动等级 → 执行机构（§5 映射）----
 * M1：日志改为**边沿触发**（等级变化才打印）。板端可能 20ms 发一次同等级 0x123，
 * 若每帧都打印，115200 下一行 ≈4ms → 会挤占调度时间；持续观测交给 Task_1000ms 状态行。*/
/**
 * @brief  按 AEB 制动等级驱动执行机构（brake_v2；应用层映射唯一入口）
 * @param  level 制动等级 0~3（调用方已由驱动层保证合法性）：
 *               0 = 正常/解除（唯一解锁路径）／1 = 预警（只打日志）
 *               2 = 部分制动（锁存，由 20ms 服务逐拍减速）／3 = 紧急制动（立即短路制动）
 * @return 无
 * @note   ① 刹车优先：等级 2/3 会置 g_brake_latched，之后即使 0x123 丢失也**不自动解除**，
 *           只有收到等级 0 才清锁存（§5 失效保护）；
 *         ② 等级 3 走 Motor_EmergencyBrake() 立即短路，不走斜坡；
 *         ③ 日志为边沿触发（等级变化才打印），防 115200 被周期帧刷爆；
 *         ④ 每次调用都刷新 g_brake_rx_ms（Task_100ms 的 0x123 超时基准）。
 */
static void Aeb_ApplyBrakeLevel(uint8 level)
{
    boolean changed = (level != g_brake_level) ? TRUE : FALSE;   /* 等级变化 = 日志边沿 */

    if (level == BRAKE_LEVEL_EMERGENCY) {          /* 3：立即短路急刹 + 锁存 */
        Motor_EmergencyBrake();                    /* 立即短路制动（IN1=IN2=H），不等 20ms 斜坡 */
        g_brake_latched = TRUE;                    /* 锁存：只有收到 level 0 才解除（刹车优先） */
        Led_Set(TRUE);                             /* LED0 常亮表示制动态 */
        if (changed) uart_puts("[AEB] level=3 EMERGENCY (latched, only level0 releases)\r\n");
    } else if (level == BRAKE_LEVEL_PARTIAL) {     /* 2：部分制动（Task_20ms 逐拍减速，阈值 A1）*/
        g_brake_latched = TRUE;                    /* 同样锁存：超时丢失也不自动解除 */
        Led_Set(TRUE);                     /* 与 README 一致：等级 2/3 常亮 */
        if (changed) uart_puts("[AEB] level=2 PARTIAL (decelerate->coast, latched)\r\n");
    } else if (level == BRAKE_LEVEL_WARN) {        /* 1：预警，不制动，也不改锁存 */
        if (changed) uart_puts("[AEB] level=1 WARN (no brake)\r\n");
    } else {                                       /* 0：解除制动（唯一解锁路径）*/
        g_brake_latched = FALSE;                   /* 清锁存 */
        Motor_ReleaseBrake();                      /* 短路制动 → 滑行（驱动层已修）*/
        Motor_SetSpeed(0);                         /* 目标速度归零，确保不再输出 PWM */
        Led_Set(FALSE);                            /* 熄灭制动指示 */
        if (changed) uart_puts("[AEB] level=0 RELEASE\r\n");
    }

    g_brake_level   = level;       /* 记录本次生效等级（供 20ms 服务/状态行） */
    g_brake_rx_seen = TRUE;        /* 标记"收过 0x123"，失效保护以此为前置条件 */
    g_brake_rx_ms   = millis();    /* 刷新超时时间戳（Task_100ms 判 0x123 是否丢失） */
}

/* ---- 等级 2 部分制动服务（20ms 拍）----
 * 借 Motor_SetSpeed 内部斜坡（RAMP_STEP=80/次）把输出逐步降到 0：
 * 递减速率 = 80/20ms = 4000 单位/s（±1000 满速约 0.25s 降到 0），到底后滑行。
 * 注意：这里**不**用短路制动（那是等级 3 的语义）；速率若要改，先确认阈值。 */
/**
 * @brief  等级 2（部分制动）的 20ms 服务：借斜坡把输出逐步降到 0
 * @return 无
 * @note   ① 只服务“等级 2 且已锁存”的状态，其余情况立即返回（幂等）；
 *         ② **不用短路制动**（那是等级 3 的语义），只把目标速度置 0，靠
 *           Motor_SetSpeed 的斜坡（80/20ms ≈ 0.25s 到 0）平滑减速后滑行；
 *         ③ 已处于 MOTOR_STATE_BRAKE 或速度已为 0 时不再干预；
 *         ④ 减速速率若要改，先确认 AEB 阈值（规则 §5 要求先确认再动）。
 */
static void Aeb_ServicePartialBrake(void)
{
    if (g_brake_level != BRAKE_LEVEL_PARTIAL) return;    /* 只服务等级 2 */
    if (!g_brake_latched) return;                        /* 未锁存 → 没有待执行的制动请求 */
    if (Motor_GetState() == MOTOR_STATE_BRAKE) return;   /* 已短路制动，不再干预 */
    if (Motor_GetSpeed() == 0) return;                   /* 已减速到位 */
    Motor_SetSpeed(0);                                   /* 目标置 0：驱动层斜坡每拍降 80，逐步减速到滑行 */
}

/* ---- 非法 0x123 的日志（M2：1s 限流，防刷屏；首次必打）---- */
/**
 * @brief  非法 0x123 帧的限流日志（1s 最多一条，首帧立即打印）
 * @return 无
 * @note   仅打印告警，**不改变任何制动状态**（无效帧一律丢弃）；限流是为了避免
 *         板端错帧率时把 115200 串口刷满、反过来拖慢 10ms 轮询。
 */
static void Aeb_LogMalformedFrame(void)
{
    static uint32 last_ms = 0;      /* 上次打印时刻（0 = 还没打过 → 首帧立即打印） */
    uint32 now = millis();

    if ((last_ms == 0u) || ((now - last_ms) >= 1000u)) {
        last_ms = now;                                                   /* 刷新限流起点 */
        uart_puts("[AEB] 0x123 malformed frame ignored (DLC!=8 or level>3)\r\n");
    }
}
#endif /* CAN_PROTOCOL_BRAKE_V2 */

/* ================================================================
 *  应用层：heartbeat_v1 行车链路状态机（0x100 / 0x101 / 0x102；.clinerules §5）
 *  ------------------------------------------------------------------
 *  仅当 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译（与 brake_v2 编译期二选一）。
 *  只调驱动抽象接口（Can_SendRdkHsReq / Can_ParseRdk*）与执行机构抽象接口
 *  （Motor_ / Led_(LED0) / Led1_(LED1 链路指示)），不碰寄存器（.clinerules §4.1）；
 *  参数全部读 g_rdk_cfg（Inc/rdk_config.h，定义在 Src/rdk_config.c）。
 *
 *  回环自测镜像（CAN_LOOPBACK_SELFTEST）里**整体不编译**：
 *    ① 避免多发 0x101 干扰回环判定；② 不与回环自测争用电机。
 *
 *  状态迁移（与规格一一对应）：
 *    STATIC  上电/复位：PWM=0；每 200ms 发 0x101（无次数上限）；**忽略 0x100**；
 *            有效应答（DLC=8 && byte0=0x55 && byte1=0x01 && byte2 递增）累计 2 帧 → CRUISE；
 *            60s 未完成握手 → LOCK。
 *    CRUISE  唯一可驱动状态；每 200ms 发 0x101；按有效 0x100 的 byte0 执行
 *            （00→PWM=0 / 01→巡航软启动 / 02→减速+LED 慢闪 1Hz / 03→刹停+LED 快闪 5Hz）；
 *            **判活只看 0x100**：静默 ≥ hb_timeout_ms(300) → 本地刹停进 HOLD。
 *    HOLD    PWM=0 保持；每 200ms 发 0x101 重握手；收到有效 0x100（**唯一恢复条件**）
 *            → 回 CRUISE，按该帧 byte0 执行；60s 未恢复 → LOCK。
 *    LOCK    PWM=0、LED 常亮、串口打印 LOCK、**停发 0x101**；需复位恢复。
 *
 *  ⚠ 判活铁律：0x102 **不参与** CRUISE/HOLD 的任何判活/恢复/超时（只在 STATIC 阶段
 *    累计握手应答，其余时候只作日志）。
 *  ⚠ 帧有效性：DLC=8 && byte0∈{0x00..0x03} && byte3 与上一帧不同（去重）；
 *    无效帧丢弃且**不刷新判活时间戳**（防"一帧卡住"把车一直开着）。
 * ================================================================ */
#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST

/* ---- 状态码（日志里直接打数字） ---- */
#define RDK_ST_STATIC      0u   /* 上电/复位：PWM=0，发 0x101@200ms，忽略 0x100 */
#define RDK_ST_CRUISE      1u   /* 唯一可驱动状态（已握手） */
#define RDK_ST_HOLD        2u   /* 0x100 超时 → PWM=0 保持，等有效 0x100 恢复 */
#define RDK_ST_LOCK        3u   /* PWM=0 + LED 常亮 + 停发 0x101，需复位 */

/* ---- 共享状态（回调 ↔ 20ms/10ms 服务，全部 volatile） ---- */
static volatile uint8   g_rdk_state      = RDK_ST_STATIC;
static volatile uint8   g_rdk_tx_seq     = 0u;     /* 0x101 的 byte1 seq（每帧 +1） */
static volatile uint32  g_rdk_req_count  = 0u;     /* 已发 0x101 帧数（无上限） */
static volatile uint32  g_rdk_req_ms     = 0u;     /* 最近一次发 0x101 的时刻 */
static volatile uint32  g_rdk_state_ms   = 0u;     /* 进入 STATIC/HOLD 的时刻（60s 窗口计） */

static volatile uint8   g_rdk_ack_count  = 0u;     /* STATIC 阶段累计有效 0x102 应答 */
static volatile uint8   g_rdk_last_ack_seq = 0u;   /* 上一帧有效应答的 byte2（递增判定） */
static volatile boolean g_rdk_ack_seq_seen = FALSE;

static volatile uint8   g_rdk_cmd        = 0xFFu;  /* 最近一帧有效 0x100 的 cmd（0xFF=无） */
static volatile uint8   g_rdk_last_cnt   = 0u;     /* 上一帧有效 0x100 的 byte3（去重基准） */
static volatile boolean g_rdk_cnt_seen   = FALSE;
static volatile uint32  g_rdk_hb_ms      = 0u;     /* 最近一帧**有效** 0x100 时刻（唯一判活源） */
static volatile uint32  g_rdk_last_rx_100= 0u;     /* == hb_ms（回归脚本/SWD 观测符号，同步更新） */
static volatile uint32  g_rdk_last_rx_102= 0u;     /* 最近一帧有效 0x102 时刻（仅日志，不判活） */
static volatile Rdk_HbFrame_t g_rdk_hb;            /* 最近一帧有效 0x100 内容（留存打印） */
static volatile uint32  g_rdk_hb_rx      = 0u;     /* 有效 0x100 累计 */
static volatile uint32  g_rdk_hb_dup     = 0u;     /* byte3 相同被去重丢弃累计 */
static volatile uint32  g_rdk_hb_bad     = 0u;     /* DLC≠8 / cmd 越界 被拒累计 */

static volatile sint16  g_rdk_pwm_out    = 0;      /* 当前目标 PWM（含方向） */
static volatile sint32  g_rdk_v_cmps     = 0;      /* 最近一拍轮速（两轮绝对平均 cm/s，日志） */
static volatile boolean g_rdk_buzz_on    = FALSE;  /* cmd==02 时 TRUE（LED0 慢闪） */

/* ---- 状态迁移（统一打印，串口可见完整链路） ---- */
/**
 * @brief  统一的状态迁移函数（改状态 + 打印完整链路，串口可见）
 * @param  next 目标状态编号：RDK_ST_STATIC / HANDSHAKE / CRUISE / HOLD / LOCK
 * @return 无
 * @note   状态未变时立即返回、不打印（防周期刷屏）。状态变量是 volatile 共享量
 *         （接收回调与各任务都读写），必须经由本函数修改，不能就地赋值。
 */
static void Rdk_SetState(uint8 next)
{
    if (next == g_rdk_state) return;       /* 状态没变就不打印（避免周期刷屏） */

    uart_puts("[RDK] STATE ");             /* 打印迁移：旧状态 -> 新状态 */
    uart_put_uint(g_rdk_state);
    uart_puts(" -> ");
    uart_put_uint(next);
    uart_puts("\r\n");
    g_rdk_state = next;                    /* 真正落地新状态（volatile：回调/任务共享） */
}

/* ---- 设目标 PWM 并立即下发（Motor_SetSpeed 内部 20ms 斜坡限幅 → "软启动"）---- */
/**
 * @brief  记录并下发目标 PWM（软启动：真正爬坡在驱动层的 20ms 斜坡里完成）
 * @param  pwm 目标 PWM，±1000 且**含方向**（正=前进 / 负=后退 / 0=停）；
 *             典型取值来自 g_rdk_cfg.cruise_pwm × cruise_dir，或 warn_pwm × cruise_dir
 * @return 无
 * @note   必须"记录 + 下发"两步都做：只下发不记录，20ms 服务下一拍就无法把斜坡
 *         继续推向目标（斜坡需要每拍重新喂同一个目标值）。
 *         禁止直接写定时器寄存器 —— 业务层只走驱动抽象接口（规则 §4.1）。
 */
static void Rdk_SetPwm(sint16 pwm)
{
    g_rdk_pwm_out = pwm;           /* 记下目标值：20ms 服务每拍用它重发，斜坡才能持续推进 */
    Motor_SetSpeed(pwm);           /* 下发到驱动层（驱动层内部每拍最多走 80，禁止瞬间全速） */
}

/* ---- 制动输出（上电 / 进 HOLD / 进 LOCK / cmd=03 时调用）---- */
/**
 * @brief  制动输出：PWM 目标归零，并按配置决定“短路制动”还是“滑行停车”
 * @return 无
 * @note   ① 上电（Rdk_Init）、进 HOLD、进 LOCK、收到 cmd=03 都走这里，是唯一的
 *           “停车出口”，避免各处逻辑不一致；
 *         ② g_rdk_cfg.stop_short_brake 决定是否调 Motor_EmergencyBrake()；否则只
 *           Motor_SetSpeed(0)（滑行）；
 *         ③ 同步把 g_rdk_pwm_out 置 0 —— 否则 20ms 服务的斜坡会把速度又推回原值。
 */
static void Rdk_StopOutput(void)
{
    if (g_rdk_cfg.stop_short_brake) {
        Motor_EmergencyBrake();     /* PWM=0 + 短路制动（TB6612 IN1=IN2=H） */
    } else {
        Motor_SetSpeed(0);          /* 只把 PWM 拉到 0（滑行停车） */
    }
    g_rdk_pwm_out = 0;              /* 目标也归零：避免 20ms 服务又把斜坡推回原速度 */
}

/* ---- 异常 0x100 计数（DLC≠8 或命令码越界）：实时打印会刷屏 → 1s 限流一条，
 *      累计值可在 Task_1000ms 的 [RDK] 状态行里查（bad=）---- */
/**
 * @brief  累计并限流打印“非法 0x100 帧”（DLC≠8 或命令码越界）
 * @return 无
 * @note   ① 计数 g_rdk_hb_bad 无条件累加（日志被限流也不丢统计，状态行 bad= 可查）；
 *         ② 打印限流 1s 一条、首条立即打，防止错帧把串口刷满；
 *         ③ 非法帧**不刷新判活时间戳**，也不会被当成有效命令执行。
 */
static void Rdk_NoteBadFrame(void)
{
    static uint32 last_ms = 0;      /* 上次打印时刻（0 = 首条立即打） */
    uint32 now = millis();

    g_rdk_hb_bad++;                 /* 无条件累计：即便日志被限流，计数也要准 */
    if ((last_ms == 0u) || ((now - last_ms) >= 1000u)) {
        last_ms = now;              /* 刷新限流起点 */
        uart_puts("[RDK] 0x100 invalid frame ignored (DLC!=8 or cmd>3)\r\n");
    }
}

/* ---- 执行一条有效 0x100 命令（cmd 已由驱动层保证 ∈{0x00..0x03}）---- */
/**
 * @brief  执行一条有效 0x100 命令（heartbeat_v1 唯一执行入口）
 * @param  cmd 命令码 0x00~0x03（合法性由驱动层 Can_ParseRdkHeartbeat 保证）：
 *             0x00 停车（PWM 目标归零，不短路制动）／0x01 巡航（软启动到 cruise_pwm）／
 *             0x02 预警（降到 warn_pwm，斜坡减速 + LED0 慢闪）／
 *             0x03 制动（PWM=0 + 短路刹停 + LED0 快闪）
 * @return 无
 * @note   ① 先记 g_rdk_cmd 与预警闪烁标志，LED 服务/状态行随后读它；
 *         ② 速度只通过 Rdk_SetPwm()/g_rdk_pwm_out 表达，实际爬坡在 20ms 服务的斜坡里；
 *         ③ 命令值不会越界（驱动层已拒），default 分支仅为防御；
 *         ④ 本函数不判断状态机 —— 何时允许执行由 Rdk_OnHeartbeat() 决定。
 */
static void Rdk_ApplyCmd(uint8 cmd)
{
    g_rdk_cmd     = cmd;                                          /* 记录当前命令（LED 服务/状态行都读它） */
    g_rdk_buzz_on = (cmd == RDK_CMD_ALERT) ? TRUE : FALSE;         /* 02 预警：LED0 慢闪标志 */

    switch (cmd) {
    case RDK_CMD_PARK:                                  /* 00 停车：PWM=0（20ms 服务拉低） */
        g_rdk_pwm_out = 0;                              /* 目标归零；不直接短路制动（语义是"停车"而非"刹停"） */
        break;
    case RDK_CMD_CRUISE:                                /* 01 巡航：软启动到巡航速度 */
        Rdk_SetPwm((sint16)(g_rdk_cfg.cruise_pwm * g_rdk_cfg.cruise_dir));
        /* 巡航 PWM × 方向系数：方向装反时只改 config 里的 cruise_dir，不动接线 */
        break;
    case RDK_CMD_ALERT:                                 /* 02 预警：减速（LED 慢闪见 LED 服务） */
        Rdk_SetPwm((sint16)(g_rdk_cfg.warn_pwm * g_rdk_cfg.cruise_dir));
        /* 用更小的 warn_pwm（约巡航一半）→ 通过斜坡逐步减速，不是急刹 */
        break;
    case RDK_CMD_STOP:                                  /* 03 制动：PWM=0 + 短路制动 */
        g_rdk_pwm_out = 0;                              /* 目标先归零 */
        Rdk_StopOutput();                               /* 立即刹停（短路制动，PWM=0） */
        break;
    default:
        break;                                          /* 不可达（驱动层已校验） */
    }
}

/* ---- 进巡航（握手成功：STATIC → CRUISE）：按 config 输出速度/方向（斜坡由 Motor_SetSpeed 内部 20ms 限幅）---- */
/**
 * @brief  进入 CRUISE（握手成功 STATIC/HOLD → CRUISE），按配置输出巡航速度
 * @return 无
 * @note   ① 顺序有讲究：先置 g_rdk_cmd=CRUISE、清预警标志、下发巡航 PWM，再打日志，
 *           最后重起判活时间戳（g_rdk_hb_ms / g_rdk_last_rx_100）并切状态；
 *         ② 判活时间戳必须在此重新起算：否则“进巡航前那段静默”会被算进
 *           hb_timeout_ms，一进巡航就误判超时、立刻掉回 HOLD；
 *         ③ 速度只经 Rdk_SetPwm() 表达，实际爬坡由 20ms 服务的斜坡完成（软启动）。
 */
static void Rdk_EnterCruise(void)
{
    uint32 now = millis();         /* 统一取一次时刻，后面几个时间戳都用它 */

    g_rdk_cmd     = RDK_CMD_CRUISE;                                  /* 握手成功即视为收到"巡航"语义 */
    g_rdk_buzz_on = FALSE;                                           /* 清预警闪烁标志 */
    Rdk_SetPwm((sint16)(g_rdk_cfg.cruise_pwm * g_rdk_cfg.cruise_dir));   /* 软启动到巡航速度 */

    uart_puts("[RDK] HS OK (ack=");
    uart_put_uint(g_rdk_ack_count);        /* 打印累计到的有效应答帧数（应为 2） */
    uart_puts(") -> CRUISE pwm=");
    uart_put_sint(g_rdk_pwm_out);
    uart_puts("\r\n");

    /* 判活看门狗从进巡航时刻起算（否则"进巡航前那段静默"会被算进 hb_timeout_ms，一进巡航就误判超时） */
    g_rdk_hb_ms       = now;
    g_rdk_last_rx_100 = now;

    Rdk_SetState(RDK_ST_CRUISE);           /* STATIC/HOLD → CRUISE（串联的 LED1 慢闪随之生效） */
}

/* ---- 发一帧 0x101（[0xAA, seq]）：恒发、无次数上限；打印限流（第 1 帧 + 每 5s）---- */
/**
 * @brief  发一帧 0x101 握手请求，并维护周期/计数/seq 与限流日志
 * @return 无
 * @note   ① 顺序：先发（驱动层组帧 {0xAA, seq}）、再记 g_rdk_req_ms、再计数、最后 seq++；
 *           20ms 服务靠 req_ms 算 200ms 周期；
 *         ② 无次数上限、无阶段停发（LOCK 阶段由调用方整体不再调用）；
 *         ③ 日志限流：第 1 帧 + 每 25 帧（25×200ms=5s）打一条，避免占满 115200。
 */
static void Rdk_SendHsReq(void)
{
    Can_SendRdkHsReq(g_rdk_tx_seq);     /* 组帧 + 发送全在驱动层，应用层只给 seq */
    g_rdk_req_ms = millis();            /* 记录发送时刻：20ms 服务据此算 200ms 周期 */
    g_rdk_req_count++;                  /* 累计已发帧数（无上限） */
    g_rdk_tx_seq++;                     /* seq 自增：板端据此判重/判活 */

    if ((g_rdk_req_count == 1u) || ((g_rdk_req_count % 25u) == 0u)) {
        /* 打印限流：首帧必打 + 每 25 帧（25×200ms = 5s）打一条，避免 115200 被日志占满 */
        uart_puts("[RDK] 0x101 seq=");
        uart_put_uint(g_rdk_tx_seq);
        uart_puts(" req=");
        uart_put_uint(g_rdk_req_count);
        uart_puts(" st=");
        uart_put_uint(g_rdk_state);
        uart_puts("\r\n");
    }
}

/* ---- 0x102 应答/心跳：**仅 STATIC 阶段**参与握手（累计 2 帧有效应答）；其余时候只记时刻 ----
 * ⚠ 0x102 绝不参与 CRUISE/HOLD 的判活/恢复/超时（判活只看 0x100）。 */
/**
 * @brief  处理一帧 0x102（应答/心跳）：**仅 STATIC 阶段**计数握手，其它状态只记时刻
 * @param  kind 帧类型：0x01 = 应答（参与握手）／0x02 = 心跳（只记时刻，不计数）
 * @param  seq  byte2 序号 0~255；要求**递增**，与上一次相同则视为重复帧丢弃
 * @return 无
 * @note   ① 判活铁律：0x102 绝不参与 CRUISE/HOLD 的判活、恢复、超时，只作日志；
 *         ② 累计到 g_rdk_cfg.hs_ack_need（默认 2）帧有效应答即触发 STATIC → CRUISE；
 *         ③ 计数饱和到 255，防长时间误收导致回绕。
 */
static void Rdk_OnHsAck(uint8 kind, uint8 seq)
{
    g_rdk_last_rx_102 = millis();              /* 无论什么类型：先记时刻（仅供日志 age102 观测） */

    if (kind != RDK_HS_ACK_KIND_ACK) return;   /* 0x02 心跳：只记时刻，不计数 */
    if (g_rdk_state != RDK_ST_STATIC) return;  /* 仅 STATIC 阶段参与握手判定 */

    if (g_rdk_ack_seq_seen && (seq == g_rdk_last_ack_seq)) return;  /* byte2 未递增 → 丢弃 */
    /* 要求 seq 递增：防止同一帧被重复计数（假握手） */
    g_rdk_ack_seq_seen = TRUE;                 /* 记住"已见过 seq"，后续才能比较递增 */
    g_rdk_last_ack_seq = seq;                  /* 保存本帧 seq 作为下次比较基准 */
    if (g_rdk_ack_count < 0xFFu) g_rdk_ack_count++;   /* 累计有效应答（饱和到 255，防溢出回绕） */
}

/* ---- 0x100 心跳/命令：去重（byte3）→ 记账 → 按状态执行 ----
 * 去重铁律：byte3 与上一帧相同 → 丢弃，且**不刷新判活时间戳**（防"一帧卡住"把车一直开着）。 */
/**
 * @brief  处理一帧已解析的 0x100（去重 → 记账 → 按状态执行）
 * @param  f 解析后的 0x100 内容指针，不可为 NULL（cmd/dist_cm/conf/cnt 四字段）
 * @return 无
 * @note   ① 去重铁律：byte3(cnt) 与上一帧相同 → 计 dup 并**直接返回**，
 *           不刷新判活时间戳（防"一帧卡住"把车一直开着）；
 *         ② 判活唯一来源：只有本函数刷新 g_rdk_hb_ms / g_rdk_last_rx_100；
 *         ③ STATIC 阶段忽略 0x100（不执行、不判活）；LOCK 阶段一概忽略（等复位）；
 *         ④ CRUISE/HOLD 收到有效帧 → 回/留在 CRUISE 并按 cmd 执行 Rdk_ApplyCmd()。
 */
static void Rdk_OnHeartbeat(const Rdk_HbFrame_t* f)
{
    static uint8 last_cmd_printed = 0xFFu;   /* 日志边沿触发：同命令不刷屏（0x100 是周期帧）*/

    if (g_rdk_cnt_seen && (f->cnt == g_rdk_last_cnt)) {   /* 计数未变：重复帧，丢弃 */
        g_rdk_hb_dup++;                                   /* 累计去重数（状态行 dup= 可查） */
        return;                                           /* 关键：直接返回，不刷新 hb_ms */
    }
    g_rdk_cnt_seen = TRUE;                    /* 已建立去重基准 */
    g_rdk_last_cnt = f->cnt;                  /* 更新去重基准为本帧计数 */

    g_rdk_hb          = *f;                   /* 留存整帧内容（状态行/调试用） */
    g_rdk_hb_ms       = millis();             /* 刷新判活时间戳（唯一判活源） */
    g_rdk_last_rx_100 = g_rdk_hb_ms;              /* 判活只看 0x100（与 hb_ms 同义，同步更新） */
    g_rdk_hb_rx++;                            /* 累计有效 0x100 帧数 */

    if (g_rdk_state == RDK_ST_STATIC) return;     /* STATIC：忽略 0x100（不执行、不判活） */
    if (g_rdk_state == RDK_ST_LOCK)   return;     /* LOCK：忽略一切（等复位） */

    if (g_rdk_state == RDK_ST_HOLD) {             /* HOLD：有效 0x100 是唯一恢复条件 */
        uart_puts("[RDK] HOLD -> CRUISE (valid 0x100)\r\n");
        Rdk_SetState(RDK_ST_CRUISE);              /* 先回 CRUISE，再按本帧命令执行 */
    }

    Rdk_ApplyCmd(f->cmd);                         /* CRUISE（含刚从 HOLD 恢复）：执行命令 */

    if (f->cmd != last_cmd_printed) {             /* 命令变化才打印（边沿触发）*/
        last_cmd_printed = f->cmd;                /* 更新边沿基准 */
        uart_puts("[RDK] HB cmd=");
        uart_put_uint(f->cmd);                    /* byte0：命令 0~3 */
        uart_puts(" cnt=");
        uart_put_uint(f->cnt);                    /* byte3：计数（去重用） */
        uart_puts(" dist=");
        uart_put_uint(f->dist_cm);                /* byte1：距离 cm */
        uart_puts("cm conf=");
        uart_put_uint(f->conf);                   /* byte2：置信度 */
        uart_puts(" pwm=");
        uart_put_sint(g_rdk_pwm_out);             /* 当前目标 PWM（含方向符号） */
        uart_puts("\r\n");
    }
}

/* ================================================================
 *  LED 服务（LED0=PF9 命令指示，LED1=PF10 链路指示；均低电平点亮）—— 非阻塞状态映射
 *  ------------------------------------------------------------------
 *  LED0（对应板端"LED 指示"约定）：
 *      HOLD/LOCK            → 常亮（制动保持 / 锁死）
 *      cmd=02 预警           → 慢闪 1Hz（半周期 500ms）
 *      cmd=03 制动           → 快闪 5Hz（半周期 100ms）
 *      其余（STATIC / 00 / 01）→ 灭
 *  LED1（链路状态）：
 *      STATIC → 快闪 ≈200ms（等握手）；CRUISE → 慢闪 ≈1s；HOLD/LOCK → 常亮
 *  ⚠ 用 millis() 相位生成闪烁（无共享计数器，对任务抖动不敏感）；
 *    由 Task_10ms 每拍写一次，全程非阻塞、无 delay_ms（规则 §3）。
 * ================================================================ */
#define RDK_LED_SLOW_HALF_MS  500u   /* 慢闪半周期：500ms 亮 + 500ms 灭 = 1Hz */
#define RDK_LED_FAST_HALF_MS  100u   /* 快闪半周期：100ms 亮 + 100ms 灭 = 5Hz */

/**
 * @brief  两盏 LED 的显示服务（10ms 一拍调用，纯相位计算、无阻塞、无共享计数器）
 * @param  now 当前时刻（millis() 的值，调用方统一取一次传入，保证同拍两灯用同一时基）
 * @return 无
 * @note   ① LED0 语义 = 命令/制动：HOLD/LOCK 常亮；0x02 预警慢闪 1Hz；
 *           0x03 制动快闪 5Hz；其余灭；
 *         ② LED1 语义 = 链路：STATIC 快闪（等握手）、CRUISE 慢闪 ≈1s、
 *           HOLD/LOCK 常亮 —— 与 LED0 语义隔离，两件事可同时表达；
 *         ③ 闪烁用 (now / 半周期) & 1 生成相位：对任务抖动不敏感，也不用额外状态变量；
 *         ④ 严禁 delay_ms（规则 §3）。
 */
static void Rdk_LedService(uint32 now)
{
    boolean on;        /* 本拍 LED0 应有的状态（TRUE=亮） */

    /* ---- LED0：命令映射 ---- */
    if ((g_rdk_state == RDK_ST_HOLD) || (g_rdk_state == RDK_ST_LOCK)) {
        on = TRUE;                                              /* 制动保持 / 锁死：常亮 */
    } else if (g_rdk_cmd == RDK_CMD_ALERT) {
        on = (((now / RDK_LED_SLOW_HALF_MS) & 1u) != 0u);       /* 02 预警：慢闪 1Hz */
        /* 用 millis() 的商取奇偶做相位：每 500ms 翻转一次，无状态、对任务抖动不敏感 */
    } else if (g_rdk_cmd == RDK_CMD_STOP) {
        on = (((now / RDK_LED_FAST_HALF_MS) & 1u) != 0u);       /* 03 制动：快闪 5Hz */
    } else {
        on = FALSE;                                             /* 00/01/STATIC：灭 */
    }
    Led_Set(on);                                                /* 写 LED0（BSRR 原子操作） */

    /* ---- LED1：链路状态 ---- */
    if ((g_rdk_state == RDK_ST_HOLD) || (g_rdk_state == RDK_ST_LOCK)) {
        Led1_Set(TRUE);                                         /* 常亮 */
    } else if (g_rdk_state == RDK_ST_STATIC) {
        Led1_Set(((now / RDK_LED_FAST_HALF_MS) & 1u) != 0u);    /* 等握手：快闪 ≈200ms */
    } else {
        Led1_Set(((now / RDK_LED_SLOW_HALF_MS) & 1u) != 0u);    /* 已握手：慢闪 ≈1s */
    }
}

/* ---- 进 HOLD：PWM=0 + 制动保持（0x100 超时 / 本地刹停的唯一入口）---- */
/**
 * @brief  进入 HOLD：本地刹停 + PWM 保持 0（0x100 超时的唯一入口）
 * @return 无
 * @note   ① 先 Rdk_StopOutput() 真正停住车，再记 60s 窗口起点、打日志、切状态；
 *         ② 判活只看 0x100：本函数由 Rdk_Service20ms() 在
 *           (now - g_rdk_hb_ms) >= hb_timeout_ms 时调用，0x102 在不在都一样；
 *         ③ HOLD 里继续发 0x101 重握手；恢复只认“有效 0x100”（见 Rdk_OnHeartbeat）。
 */
static void Rdk_EnterHold(void)
{
    Rdk_StopOutput();               /* 本地刹停（PWM=0 + 按 config 决定是否短路制动） */
    g_rdk_state_ms = millis();     /* 60s 未恢复 → LOCK 的窗口起点 */
    uart_puts("[RDK] HOLD (pwm=0, wait valid 0x100)\r\n");
    Rdk_SetState(RDK_ST_HOLD);     /* 迁移到 HOLD（LED0/LED1 随之常亮） */
}

/* ---- 进 LOCK：PWM=0 + 停发 0x101（需复位恢复）---- */
/**
 * @brief  进入 LOCK：PWM=0 保持 + 停发 0x101（只能靠复位退出）
 * @return 无
 * @note   ① 进入前一定先停住车（Rdk_StopOutput），不能带着速度锁死；
 *         ② 触发条件：STATIC 阶段 60s 未握手成功，或 HOLD 阶段 60s 未恢复；
 *         ③ LOCK 下 20ms 服务不再发 0x101、忽略 0x100；LED0/LED1 常亮，串口打印 LOCK。
 */
static void Rdk_EnterLock(void)
{
    Rdk_StopOutput();              /* 保持刹停，不能带着速度进锁死态 */
    g_rdk_state_ms = millis();     /* 记录进入时刻（本状态不再计时，仅保持一致） */
    uart_puts("[RDK] LOCK (pwm=0, LED on, 0x101 stopped) - reset required\r\n");
    Rdk_SetState(RDK_ST_LOCK);     /* 迁移到 LOCK；20ms 服务在该状态不再发 0x101 */
}

/* ================================================================
 *  20ms 服务：0x101 恒发（STATIC / CRUISE / HOLD）/ 60s 锁定窗口 /
 *  判活看门狗（**只看 0x100**）/ 巡航斜坡推进
 *  全程非阻塞（规则 §3：任务内禁止 delay_ms），在 Task_20ms 末尾调用。
 * ================================================================ */
/**
 * @brief  heartbeat_v1 的 20ms 服务：0x101 恒发 + 判活看门狗 + 斜坡推进 + 60s 锁定窗口
 * @return 无
 * @note   ① 先由 Task_20ms 已算好的 g_wheel_l/r_cmps 取绝对平均发布成 g_rdk_v_cmps；
 *           **不在这里调 Encoder_GetSpeed*()**：那是有状态接口，二次消费会把车速读成 0；
 *         ② STATIC：累计 ≥hs_ack_need 帧应答 → CRUISE；60s 未握手 → LOCK；否则每 200ms 发 0x101；
 *         ③ CRUISE：静默 ≥hb_timeout_ms → 本地刹停进 HOLD（**只判 0x100**）；
 *           只有 cmd∈{01,02} 每拍重发目标让斜坡推进，cmd=00 让斜坡自然收敛到 0，
 *           cmd=03 不重发（否则 Motor_SetSpeed(0) 会把短路制动改回滑行）；
 *         ④ HOLD：保持 PWM=0、发 0x101、60s 未恢复 → LOCK；恢复在回调里做；
 *         ⑤ 全程非阻塞，无 delay_ms（规则 §3），在 Task_20ms 末尾调用。
 */
static void Rdk_Service20ms(void)
{
    uint32 now = millis();
    sint32 vl, vr, v;

    /* 轮速（cm/s，两轮绝对平均）：直接读 Task_20ms 已算好的观测值。
     * 不在这里调 Encoder_GetSpeed*()：那是**有状态**接口（消费 20ms 增量），
     * 二次消费会把车速观测读成 0。 */
    vl = (g_wheel_l_cmps < 0) ? -g_wheel_l_cmps : g_wheel_l_cmps;   /* 左轮速度取绝对值（倒车也算速度幅值） */
    vr = (g_wheel_r_cmps < 0) ? -g_wheel_r_cmps : g_wheel_r_cmps;   /* 右轮速度取绝对值 */
    v  = (vl + vr) / 2;                                             /* 两轮平均 = 车体纵向速度（cm/s） */
    g_rdk_v_cmps = v;                                               /* 发布给状态行日志（只读观测，不参与控制） */

    switch (g_rdk_state) {
    case RDK_ST_STATIC:
        /* 上电/复位：PWM=0 保持；忽略 0x100（由回调 return 实现）；发 0x101 等握手应答 */
        if (g_rdk_ack_count >= g_rdk_cfg.hs_ack_need) {
            Rdk_EnterCruise();                                    /* 累计 2 帧有效应答 → CRUISE */
        } else if ((now - g_rdk_state_ms) >= g_rdk_cfg.lock_timeout_ms) {
            Rdk_EnterLock();                                      /* 60s 未握手 → LOCK */
        } else if ((now - g_rdk_req_ms) >= g_rdk_cfg.hs_period_ms) {
            Rdk_SendHsReq();                                      /* 200ms 恒发 0x101 */
        }
        break;

    case RDK_ST_CRUISE:
        /* 判活铁律：**只看 0x100**。静默 ≥ hb_timeout_ms(300) → 本地刹停进 HOLD。
         * （0x102 在不在都一样，不参与此判定。） */
        if ((now - g_rdk_hb_ms) >= g_rdk_cfg.hb_timeout_ms) {
            uart_puts("[RDK] 0x100 TIMEOUT -> local stop (HOLD)\r\n");
            Rdk_EnterHold();
            break;
        }
        /* 斜坡推进：只有"可驱动"命令（01/02）每拍重发目标；00 停车让斜坡自然收敛到 0（滑行）；
         * 03 已在回调里短路制动，这里不重发（避免 Motor_SetSpeed(0) 把短路制动改回滑行）。 */
        if ((g_rdk_cmd == RDK_CMD_CRUISE) || (g_rdk_cmd == RDK_CMD_ALERT)) {
            Motor_SetSpeed(g_rdk_pwm_out);   /* 每 20ms 重发一次目标 → 驱动层斜坡持续逼近目标（软启动/软减速） */
        } else if (g_rdk_cmd == RDK_CMD_PARK) {
            Motor_SetSpeed(0);               /* 停车：目标 0，斜坡把速度平滑收敛到 0 后滑行 */
        }
        if ((now - g_rdk_req_ms) >= g_rdk_cfg.hs_period_ms) {
            Rdk_SendHsReq();                                      /* 0x101 恒发 */
        }
        break;

    case RDK_ST_HOLD:
        /* PWM=0 保持；发 0x101 重握手；恢复只认"有效 0x100"（在回调 Rdk_OnHeartbeat 处理） */
        if ((now - g_rdk_state_ms) >= g_rdk_cfg.lock_timeout_ms) {
            Rdk_EnterLock();                                      /* 60s 未恢复 → LOCK */
        } else if ((now - g_rdk_req_ms) >= g_rdk_cfg.hs_period_ms) {
            Rdk_SendHsReq();                                      /* 200ms 恒发 0x101 */
        }
        break;

    case RDK_ST_LOCK:
    default:
        /* PWM=0 + LED 常亮 + 停发 0x101；需复位恢复（Rdk_Init） */
        break;
    }
}

/* ---- 上电初始化：PWM=0 静止（行为 1）→ 发第 1 帧 0x101 → STATIC（行为 2）---- */
/**
 * @brief  上电初始化：清零全部状态 → PWM=0 静止（行为 1）→ 发第 1 帧 0x101（行为 2）
 * @return 无
 * @note   ① 把 g_rdk_state 复位为 STATIC、cmd=0xFF、各种计数/时间戳归零；
 *           Rdk_SelfTestParser() 可能已动过 g_rdk_hb_* 与 g_rdk_cnt_* 系列变量，这里统一复位；
 *         ② g_rdk_req_ms 置 0 使首拍立即发 0x101（不等 200ms），并显式置
 *           g_rdk_req_count=1 与 seq 计数语义对齐；
 *         ③ 判活时间戳置 0，进巡航时再统一起算（见 Rdk_EnterCruise）；
 *         ④ 必须在 Can_RegisterRxCallback() 之后、进入调度循环之前调用。
 */
static void Rdk_Init(void)
{
    /* 清零：Rdk_SelfTestParser() 可能已动过 g_rdk_hb_* / g_rdk_cnt_* （这里统一复位） */
    g_rdk_state        = RDK_ST_STATIC;    /* 上电永远从 STATIC 开始（PWM=0、忽略 0x100） */
    g_rdk_tx_seq       = 0u;               /* 0x101 的 seq 从 0 起 */
    g_rdk_req_count    = 0u;               /* 已发 0x101 帧数清零 */
    g_rdk_ack_count    = 0u;               /* 握手应答计数清零 */
    g_rdk_last_ack_seq = 0u;               /* seq 递增判定的基准清零 */
    g_rdk_ack_seq_seen = FALSE;            /* 还没见过任何应答 seq */
    g_rdk_cmd          = 0xFFu;            /* 无有效命令（0xFF = 尚未收到） */
    g_rdk_last_cnt     = 0u;               /* 去重基准清零 */
    g_rdk_cnt_seen     = FALSE;            /* 还没见过有效 0x100 */
    g_rdk_hb_rx        = 0u;               /* 有效 0x100 计数清零 */
    g_rdk_hb_dup       = 0u;               /* 去重丢弃计数清零 */
    g_rdk_hb_bad       = 0u;               /* 非法帧计数清零 */
    g_rdk_buzz_on      = FALSE;            /* 预警闪烁标志清零 */
    g_rdk_pwm_out      = 0;                /* 目标 PWM 归零（上电静止） */
    g_rdk_v_cmps       = 0;                /* 轮速观测归零 */

    /* 判活时间戳归 0：进巡航（Rdk_EnterCruise）时统一重新起算，避免"上电静默"被算进 hb_timeout_ms */
    g_rdk_hb_ms       = 0u;
    g_rdk_last_rx_100 = 0u;
    g_rdk_last_rx_102 = 0u;
    g_rdk_req_ms      = 0u;            /* 0 = 首拍就会立即发一帧 0x101（不等 200ms） */
    g_rdk_state_ms    = millis();      /* 60s 锁定窗口起点（STATIC 未握手 / HOLD 未恢复共用） */

    uart_puts("[RDK] boot: PWM=0 static, 0x101@");
    uart_put_uint(g_rdk_cfg.hs_period_ms);
    uart_puts("ms, need ");
    uart_put_uint(g_rdk_cfg.hs_ack_need);
    uart_puts("x0x102-ack, lock after ");
    uart_put_uint(g_rdk_cfg.lock_timeout_ms / 1000u);
    uart_puts("s\r\n");

    /* 判活铁律（串口可见，便于联调核对）：只看 0x100；0x102 仅 STATIC 阶段握手计数 */
    uart_puts("[RDK] link: live only if 0x100 within ");
    uart_put_uint(g_rdk_cfg.hb_timeout_ms);
    uart_puts("ms -> HOLD; 0x102 = handshake only\r\n");

    Rdk_StopOutput();          /* 行为 1：上电 PWM=0 + 制动（静止，PWM 恒 0） */
    g_rdk_req_count = 1u;
    Rdk_SendHsReq();           /* 行为 2：立即发出第 1 帧 0x101（无次数上限） */
}

/* ---- RDK 解析器自测（纯软件、零硬件依赖，Normal 镜像开机跑一次）
 * 与 AEB 的 ⑤-A 同风格：直接构造字节数组调驱动层解析接口，覆盖
 *   0x100：cmd=0/1/2/3 合法解析（cmd/dist/conf/cnt 四字段）、cmd=4 被拒、
 *          DLC=1 被拒（§5 DLC 契约对 RDK 帧同样生效）
 *   0x102：合法应答 [0x55,0x01]、合法心跳 [0x55,0x02]、tag0 错被拒、kind 错被拒、DLC=1 被拒
 *   去重：byte3(cnt) 与上一帧相同的第二帧必须被丢弃（应用层去重，§5）
 * 不发帧、不动执行机构。 */
#define RDK_PARSER_CASES  12u

/**
 * @brief  RDK 解析器自测：12 个用例（纯软件、零硬件依赖，Normal 镜像开机跑一次）
 * @return 无（结果打印为 "[RDK-TEST] parser PASS (12/12)" 或逐条 FAIL）
 * @note   ① 覆盖：0x100 cmd=0~3 四字段解析、cmd=4 被拒、DLC=1 被拒；0x102 应答/心跳
 *           合法解析、tag0 错被拒、kind 错被拒、DLC=1 被拒；byte3 去重（第二帧必须丢）；
 *         ② 只调驱动层解析接口 + 应用层 Rdk_OnHeartbeat()，**不发帧、不动执行机构**；
 *         ③ 调用时机必须在 Rdk_Init() 之前（此时状态是 STATIC，回调只记账不驱动电机），
 *           Rdk_Init() 会把本函数动过的计数统一复位。
 */
static void Rdk_SelfTestParser(void)
{
    uint8         frame[8];
    uint8         i;
    uint8         k;
    uint8         pass = 0u;
    uint8         kind = 0u;
    uint8         seq  = 0u;
    Rdk_HbFrame_t hb;

    for (k = 0u; k <= 3u; k++) {                   /* case 1~4：0x100 cmd=0/1/2/3 合法 */
        for (i = 0u; i < 8u; i++) frame[i] = 0u;
        frame[0] = k;
        frame[1] = 42u;                            /* dist_cm */
        frame[2] = 7u;                             /* conf */
        frame[3] = 5u;                             /* cnt */
        hb.cmd = 0xFFu; hb.dist_cm = 0u; hb.conf = 0u; hb.cnt = 0u;
        if (Can_ParseRdkHeartbeat(frame, 8u, &hb) && (hb.cmd == k) &&
            (hb.dist_cm == 42u) && (hb.conf == 7u) && (hb.cnt == 5u)) {
            pass++;
        } else {
            uart_puts("[RDK-TEST] FAIL 0x100 cmd=");
            uart_put_uint(k);
            uart_puts(" not parsed\r\n");
        }
    }

    for (i = 0u; i < 8u; i++) frame[i] = 0u;       /* case 5：cmd=4（越界）必须被拒 */
    frame[0] = 4u;
    if (!Can_ParseRdkHeartbeat(frame, 8u, &hb)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x100 cmd=4 accepted (must reject)\r\n");
    }

    frame[0] = RDK_CMD_CRUISE;                     /* case 6：DLC=1 必须整帧拒绝 */
    if (!Can_ParseRdkHeartbeat(frame, 1u, &hb)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x100 len=1 accepted (must reject)\r\n");
    }

    for (i = 0u; i < 8u; i++) frame[i] = 0u;       /* case 7：0x102 [0x55,0x01,seq] 应答合法 */
    frame[0] = RDK_HS_ACK_TAG0;
    frame[1] = RDK_HS_ACK_KIND_ACK;
    frame[2] = 0x21u;
    kind = 0u; seq = 0u;
    if (Can_ParseRdkHsAck(frame, 8u, &kind, &seq) &&
        (kind == RDK_HS_ACK_KIND_ACK) && (seq == 0x21u)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x102 [0x55,0x01,0x21] rejected\r\n");
    }

    frame[1] = RDK_HS_ACK_KIND_HB;                 /* case 8：0x102 [0x55,0x02] 心跳合法 */
    kind = 0u;
    if (Can_ParseRdkHsAck(frame, 8u, &kind, &seq) && (kind == RDK_HS_ACK_KIND_HB)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x102 [0x55,0x02] rejected\r\n");
    }

    frame[0] = 0x54u;                              /* case 9：tag0 错必须被拒 */
    if (!Can_ParseRdkHsAck(frame, 8u, &kind, &seq)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x102 tag0=0x54 accepted\r\n");
    }

    frame[0] = RDK_HS_ACK_TAG0;                    /* case 10：kind 错必须被拒 */
    frame[1] = 0x00u;
    if (!Can_ParseRdkHsAck(frame, 8u, &kind, &seq)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x102 kind=0x00 accepted\r\n");
    }

    frame[1] = RDK_HS_ACK_KIND_ACK;                /* case 11：0x102 DLC=1 必须被拒（§5 DLC==8）*/
    if (!Can_ParseRdkHsAck(frame, 1u, &kind, &seq)) {
        pass++;
    } else {
        uart_puts("[RDK-TEST] FAIL 0x102 len=1 accepted (must reject)\r\n");
    }

    /* case 12：0x100 byte3 去重 —— 与上一帧 cnt 相同的第二帧必须被丢弃（判活时间戳不刷新）。
     * 此刻状态是 STATIC（Rdk_Init 还没跑）→ 回调只记账、不驱动电机，安全。 */
    for (i = 0u; i < 8u; i++) frame[i] = 0u;
    frame[0] = RDK_CMD_CRUISE;
    frame[3] = 9u;                                 /* cnt 非 0，避免与初值混淆 */
    if (Can_ParseRdkHeartbeat(frame, 8u, &hb)) {
        Rdk_OnHeartbeat(&hb);                      /* 第 1 帧：有效 */
        Rdk_OnHeartbeat(&hb);                      /* 第 2 帧：同 cnt 必须被去重丢弃 */
        if ((g_rdk_hb_rx == 1u) && (g_rdk_hb_dup == 1u)) {
            pass++;
        } else {
            uart_puts("[RDK-TEST] FAIL dedup rx=");
            uart_put_uint(g_rdk_hb_rx);
            uart_puts(" dup=");
            uart_put_uint(g_rdk_hb_dup);
            uart_puts(" (expect 1/1)\r\n");
        }
    } else {
        uart_puts("[RDK-TEST] FAIL dedup frame rejected\r\n");
    }

    uart_puts("[RDK-TEST] parser ");
    if (pass == RDK_PARSER_CASES) {
        uart_puts("PASS (");
        uart_put_uint(RDK_PARSER_CASES);
        uart_puts("/");
        uart_put_uint(RDK_PARSER_CASES);
        uart_puts(")\r\n");
    } else {
        uart_puts("FAIL (");
        uart_put_uint(pass);
        uart_puts("/");
        uart_put_uint(RDK_PARSER_CASES);
        uart_puts(")\r\n");
    }
}

#endif /* !CAN_LOOPBACK_SELFTEST —— RDK 行车链路状态机定义结束 */

/* ---- 无 heartbeat_v1 状态机时的"空访问点"宏：任务/日志共用一处引用，避免到处 #if ----
 * heartbeat_v1 状态机在 brake_v2 分支与回环镜像里都不编译 → 状态恒 0
 * （LED0/LED1 此时完全归 AEB 域，heartbeat_v1 的 LED 服务不参与编译）。*/
#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST
#define RDK_STATE_VAR      (g_rdk_state)
#else
#define RDK_STATE_VAR      (0u)      /* 无 RDK 状态机：状态恒 0 */
#endif

/* ================================================================
 *  应用层自测（§6.1：① 回环链路自测已就绪，⑤ 本轮补 AEB 解析/映射/失效保护自测）
 *  自测只调驱动抽象接口（Can_ParseXxx / Can_SendXxx）与执行机构抽象接口（Motor_Xxx），不碰寄存器。
 * ================================================================ */
#if CAN_PROTOCOL_BRAKE_V2
#define AEB_PARSER_CASES  8u   /* 见 Aeb_SelfTestParser：case 1~8（含 0x123/0x124 的 DLC≠8 拒绝）*/

/* ---- ⑤-A 解析器自测：纯软件、零硬件依赖、**两种镜像都跑**（开机一次）----
 * 直接构造 §5 字节数组调驱动层解析接口，覆盖：合法等级 0~3、非法等级 4 被拒、
 * 0x124 小端 (0x34,0x12 -> 0x1234)、0x124 DLC 过短被拒、0x123 DLC 过短被拒。
 * 不发帧、不动执行机构。 */
/**
 * @brief  AEB 解析器自测：8 个用例（纯软件、零硬件依赖，开机跑一次）
 * @return 无（结果打印为 "[AEB-TEST] parser PASS (8/8)" 或逐条 FAIL）
 * @note   ① 覆盖：0x123 等级 0~3 接受、等级 4 被拒、0x123 DLC=1 被拒；
 *           0x124 小端 (0x34,0x12→0x1234)、0x124 DLC=1 被拒；
 *         ② 只调 Can_Parse* 接口，不发帧、不动电机；
 *         ③ 仅 CAN_PROTOCOL_BRAKE_V2=1 时编译；0x123 短帧那一例是补真机首跑的
 *           历史缺陷（DLC=1 曾把等级从 1 改写回 3）。
 */
static void Aeb_SelfTestParser(void)
{
    uint8  frame[8];
    uint8  level;
    uint16 cm;
    uint8  i;
    uint8  k;
    uint8  pass = 0u;

    for (k = 0u; k < 4u; k++) {                    /* case 1~4：0x123 level=k 必须被接受 */
        for (i = 0u; i < 8u; i++) frame[i] = 0u;
        frame[0] = k;
        level    = 0xFFu;
        if (Can_ParseBrakeCmd(frame, 8u, &level) && (level == k)) {
            pass++;
        } else {
            uart_puts("[AEB-TEST] FAIL 0x123 level=");
            uart_put_uint(k);
            uart_puts(" not parsed\r\n");
        }
    }

    for (i = 0u; i < 8u; i++) frame[i] = 0u;       /* case 5：0x123 level=4 必须被拒 */
    frame[0] = 4u;
    level    = 0u;
    if (!Can_ParseBrakeCmd(frame, 8u, &level)) {
        pass++;
    } else {
        uart_puts("[AEB-TEST] FAIL 0x123 level=4 accepted (must reject)\r\n");
    }

    for (i = 0u; i < 8u; i++) frame[i] = 0u;       /* case 6：0x124 小端 0x34,0x12 = 4660cm */
    frame[0] = 0x34u;
    frame[1] = 0x12u;
    cm       = 0u;
    if (Can_ParseObstacleDist(frame, 8u, &cm) && (cm == 0x1234u)) {
        pass++;
    } else {
        uart_puts("[AEB-TEST] FAIL 0x124 LE 0x34,0x12 -> ");
        uart_put_uint(cm);
        uart_puts(" (expect 4660)\r\n");
    }

    if (!Can_ParseObstacleDist(frame, 1u, &cm)) {  /* case 7：0x124 DLC=1 必须被拒 */
        pass++;
    } else {
        uart_puts("[AEB-TEST] FAIL 0x124 len=1 accepted (must reject)\r\n");
    }

    /* case 8：0x123 DLC=1（等级本身合法）必须被拒 —— §5 数据段固定 8 字节。
     * 补这一例的原因：真机首跑时 `123#03`（DLC=1）被解析层放行，把等级从 1 改写回 3，
     * 而当时自测只覆盖了 0x124 的短帧，漏掉了 0x123 这一侧。 */
    for (i = 0u; i < 8u; i++) frame[i] = 0u;
    frame[0] = BRAKE_LEVEL_WARN;                   /* 等级合法，非法的是 DLC */
    level    = 0xFFu;
    if (!Can_ParseBrakeCmd(frame, 1u, &level) && (level == 0xFFu)) {
        pass++;
    } else {
        uart_puts("[AEB-TEST] FAIL 0x123 len=1 accepted (must reject)\r\n");
    }

    uart_puts("[AEB-TEST] parser ");
    if (pass == AEB_PARSER_CASES) {
        uart_puts("PASS (");
        uart_put_uint(AEB_PARSER_CASES);
        uart_puts("/");
        uart_put_uint(AEB_PARSER_CASES);
        uart_puts(")\r\n");
    } else {
        uart_puts("FAIL (");
        uart_put_uint(pass);
        uart_puts("/");
        uart_put_uint(AEB_PARSER_CASES);
        uart_puts(")\r\n");
    }
}
#endif /* CAN_PROTOCOL_BRAKE_V2 */

/* ---- ⑤-B 回环 AEB 自测：自发 0x123/0x124/0x125 → 回读断言"解析 + 制动映射 + 失效保护"
 *      （§6.1 步骤①，不接硬件、全程非阻塞）。步进由 Task_100ms 每 2 拍推进 1 步（200ms），
 *      一轮 10 步 = 14 个 200ms 拍 ≈ 2.8s（步 8 多静默 3 拍），间隔 AEB_LB_ROUND_MS 后自动重跑；
 *      断言只看抽象接口
 *      （g_brake_level / g_brake_latched / Motor_GetState），不碰寄存器。---- */
#if CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2
/* 发第 step 步的测试帧（只发帧，不判定） */
/**
 * @brief  回环自测：发出第 step 步的测试帧（只发帧，不作判定）
 * @param  step 步号 1~10（越界不做任何事）：
 *              1 = 0x124 距离 123cm／2 = 0x123 等级 1／3 = 等级 2／4 = 等级 3／
 *              5 = 等级 0／6 = 等级 4（非法，须被拒）／7 = 0x124 距离 0xFFFF／
 *              8 = 等级 3（之后静默，验证失效保护保持制动）／
 *              9 = 0x125 车速 123／10 = 0x125 车速 0xFFFF
 * @return 无
 * @note   仅 `CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2` 镜像编译；
 *         步 9/10 发送前会清空回读变量，因此"没收到回显"能可靠判为失败。
 */
static void Aeb_LbSendStepFrame(uint8 step)
{
    switch (step) {
    case 1u:  Can_SendObstacleDistTest(AEB_LB_TEST_DIST_CM);  break;  /* 123cm */
    case 2u:  Can_SendBrakeCmdTest(BRAKE_LEVEL_WARN);         break;  /* 1 预警 */
    case 3u:  Can_SendBrakeCmdTest(BRAKE_LEVEL_PARTIAL);      break;  /* 2 部分制动 */
    case 4u:  Can_SendBrakeCmdTest(BRAKE_LEVEL_EMERGENCY);    break;  /* 3 急刹 */
    case 5u:  Can_SendBrakeCmdTest(BRAKE_LEVEL_NONE);         break;  /* 0 解除 */
    case 6u:  Can_SendBrakeCmdTest(AEB_LB_BAD_LEVEL);         break;  /* 4 非法，须被拒 */
    case 7u:  Can_SendObstacleDistTest(AEB_DIST_INVALID);     break;  /* 0xFFFF */
    case 8u:  Can_SendBrakeCmdTest(BRAKE_LEVEL_EMERGENCY);    break;  /* 之后静默→④ 失效保护 */
    case 9u:  g_lb_speed_echo    = 0u;                                /* 清回读：缺帧即失败 */
              g_lb_speed_echo_ok = FALSE;
              Can_SendVehicleSpeed(AEB_LB_TEST_KMH);          break;  /* 123 km/h */
    case 10u: g_lb_speed_echo    = 0u;
              g_lb_speed_echo_ok = FALSE;
              Can_SendVehicleSpeed(AEB_SPEED_INVALID);        break;  /* 0xFFFF 占位 */
    default:  break;
    }
}

/* 断言第 step 步（TRUE = 通过）；失败累计并打印 exp/act，作为 §5 契约的可观测判据 */
/**
 * @brief  回环自测：断言第 step 步的结果（只看抽象接口状态，不碰寄存器）
 * @param  step 步号 1~10（语义同 Aeb_LbSendStepFrame；越界返回 FALSE）
 * @return TRUE = 该步断言通过；FALSE = 不符预期（已累计到 g_lb_round_fail 并打印 exp/act）
 * @note   判定依据：g_obstacle_cm / g_dist_rx_count / g_brake_level / g_brake_latched /
 *         Motor_GetState() / 0x125 回读值 —— 全是应用层可见状态，
 *         这样断言的是"§5 契约"而不是实现细节。
 */
static boolean Aeb_LbCheckStep(uint8 step)
{
    boolean ok  = FALSE;
    sint32  exp = -1;
    sint32  act = -1;

    switch (step) {
    case 1u:                                    /* 0x124 → 距离被解析并记录 */
        exp = (sint32)AEB_LB_TEST_DIST_CM;
        act = (sint32)g_obstacle_cm;
        ok  = ((g_obstacle_cm == AEB_LB_TEST_DIST_CM) && (g_dist_rx_count != 0u)) ? TRUE : FALSE;
        break;
    case 2u:                                    /* level 1 预警：不锁存、不短路制动 */
        exp = (sint32)BRAKE_LEVEL_WARN;
        act = (sint32)g_brake_level;
        ok  = ((g_brake_level == BRAKE_LEVEL_WARN) && (g_brake_latched == FALSE) &&
               (Motor_GetState() != MOTOR_STATE_BRAKE)) ? TRUE : FALSE;
        break;
    case 3u:                                    /* level 2 部分制动：锁存（减速由 20ms 服务做）*/
        exp = (sint32)BRAKE_LEVEL_PARTIAL;
        act = (sint32)g_brake_level;
        ok  = ((g_brake_level == BRAKE_LEVEL_PARTIAL) && (g_brake_latched == TRUE)) ? TRUE : FALSE;
        break;
    case 4u:                                    /* level 3 急刹：立即短路制动 + 锁存 */
        exp = (sint32)MOTOR_STATE_BRAKE;
        act = (sint32)Motor_GetState();
        ok  = ((Motor_GetState() == MOTOR_STATE_BRAKE) && (g_brake_latched == TRUE)) ? TRUE : FALSE;
        break;
    case 5u:                                    /* level 0：唯一解锁路径 */
        exp = (sint32)BRAKE_LEVEL_NONE;
        act = (sint32)g_brake_level;
        ok  = ((g_brake_level == BRAKE_LEVEL_NONE) && (g_brake_latched == FALSE) &&
               (Motor_GetState() != MOTOR_STATE_BRAKE)) ? TRUE : FALSE;
        break;
    case 6u:                                    /* 非法 level 4：整帧被拒，状态不变 */
        exp = (sint32)BRAKE_LEVEL_NONE;
        act = (sint32)g_brake_level;
        ok  = ((g_brake_level == BRAKE_LEVEL_NONE) && (g_brake_latched == FALSE)) ? TRUE : FALSE;
        break;
    case 7u:                                    /* 0xFFFF = 无有效目标 */
        exp = (sint32)AEB_DIST_INVALID;
        act = (sint32)g_obstacle_cm;
        ok  = (g_obstacle_cm == AEB_DIST_INVALID) ? TRUE : FALSE;
        break;
    case 8u:                                    /* ④ 失效保护：静默 >500ms 仍保持制动 */
        exp = (sint32)TRUE;
        act = (sint32)g_brake_latched;
        ok  = ((g_brake_latched == TRUE) && (Motor_GetState() == MOTOR_STATE_BRAKE)) ? TRUE : FALSE;
        break;
    case 9u:                                    /* 0x125 小端回读 = 123 */
        exp = (sint32)AEB_LB_TEST_KMH;
        act = (sint32)g_lb_speed_echo;
        ok  = ((g_lb_speed_echo_ok == TRUE) && (g_lb_speed_echo == AEB_LB_TEST_KMH)) ? TRUE : FALSE;
        break;
    case 10u:                                   /* 0x125 0xFFFF 占位回读 */
        exp = (sint32)AEB_SPEED_INVALID;
        act = (sint32)g_lb_speed_echo;
        ok  = ((g_lb_speed_echo_ok == TRUE) && (g_lb_speed_echo == AEB_SPEED_INVALID)) ? TRUE : FALSE;
        break;
    default:
        break;
    }

    if (ok != TRUE) {
        g_lb_fail++;
        g_lb_round_fail++;
        g_lb_fail_step = step;
        g_lb_fail_exp  = exp;
        g_lb_fail_act  = act;
        uart_puts("[CAN-LB] AEB SELFTEST FAIL step=");
        uart_put_uint(step);
        uart_puts(" exp=");
        uart_put_sint(exp);
        uart_puts(" act=");
        uart_put_sint(act);
        uart_puts("\r\n");
    }
    return ok;
}

/* 发第 step 步帧并记住"断言前需静默的拍数"（失效保护步要等够 AEB_LB_LOST_WAIT_MS） */
/**
 * @brief  回环自测：发第 step 步的帧并登记"断言前需静默的拍数"，推进一步
 * @param  step 步号 1~10（先发帧，再把 g_lb_step 置为该步）
 * @return 无
 * @note   只有失效保护步（AEB_LB_LOST_STEP）会设置静默拍数 AEB_LB_LOST_TICKS，
 *         用来等够超时窗口、确认"帧丢失后仍保持制动"；其它步静默拍数为 0。
 */
static void Aeb_LbBeginStep(uint8 step)
{
    Aeb_LbSendStepFrame(step);
    g_lb_step = step;
    g_lb_wait = (step == AEB_LB_LOST_STEP) ? AEB_LB_LOST_TICKS : 0u;
}

/* 自测状态机：Task_100ms 每 2 拍调用一次（200ms/步；无 delay_ms，不阻塞调度） */
/**
 * @brief  回环 AEB 自测状态机：按 200ms/步推进 10 步，发帧→等落定→断言→收尾
 * @return 无
 * @note   ① 由 Task_100ms 每 2 拍调用一次（g_lb_tick 分频），**无 delay_ms、不阻塞调度**；
 *         ② 空闲态按 AEB_LB_FIRST_DELAY_MS/AEB_LB_ROUND_MS 控制轮间隔，自动反复跑；
 *         ③ 失效保护步会先静默 AEB_LB_LOST_TICKS 拍，等够超时窗口再断言“仍保持制动”；
 *         ④ 每轮结束强制发 level 0 解锁，绝不让车停在制动态；
 *         ⑤ 仅 `CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2` 镜像编译。
 */
static void Aeb_LbSelfTest_Service(void)
{
    uint32 gap;

    if (g_lb_step == 0u) {                          /* 空闲：等首轮延迟 / 轮间隔 */
        gap = (g_lb_round == 0u) ? AEB_LB_FIRST_DELAY_MS : AEB_LB_ROUND_MS;
        if ((millis() - g_lb_round_ms) < gap) return;
        g_lb_round++;
        g_lb_round_fail = 0u;
        Aeb_LbBeginStep(1u);
        return;
    }

    if (g_lb_wait > 0u) {                           /* 静默观察拍（失效保护步用） */
        g_lb_wait--;
        return;
    }

    Aeb_LbCheckStep(g_lb_step);                     /* 距发帧 ≥200ms：收帧 + 20ms 服务都生效了 */

    if (g_lb_step >= AEB_LB_STEP_COUNT) {           /* 轮末：安全收尾 + 起新计时 */
        Can_SendBrakeCmdTest(BRAKE_LEVEL_NONE);     /* 强制 level 0 解锁，不把车留在制动态 */
        g_lb_last_round_fail = g_lb_round_fail;
        g_lb_step            = 0u;
        g_lb_round_ms        = millis();
        return;
    }

    Aeb_LbBeginStep((uint8)(g_lb_step + 1u));       /* 继续下一步 */
}
#endif /* CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2 */

/* ---- CAN 接收回调（按编译期选中的协议分派；回环模式兼做 0x111 自测判定）----
 * heartbeat_v1：0x102 握手应答 / 0x100 心跳命令
 * brake_v2    ：0x123 制动等级 / 0x124 障碍物距离
 * 两种协议编译期二选一，本分派体随宏只保留选中协议那一段。 */
/**
 * @brief  CAN 接收回调：按编译期选中的协议把帧分派给对应处理函数
 * @param  can_id 标准帧 ID（11 位）：0x100 / 0x101 / 0x102（heartbeat_v1）或
 *                0x123 / 0x124 / 0x125（brake_v2）；回环自测另认 0x111
 * @param  data   数据段指针，不可为 NULL；仅在本次调用内有效（栈上缓冲）
 * @param  len    帧 DLC，0~8；DLC≠8 的协议帧在这里被拒并计入异常统计
 * @return 无
 * @note   ① 在 Can_ProcessRx() 里**同步**执行（不在中断里），所以必须短小、禁止延时；
 *         ② 两种协议编译期二选一，未选中协议的分支整段不参与编译；
 *         ③ 0x125/0x111 是"自发自收的回读"，只在回环自测镜像里处理；
 *         ④ 本函数只做"解析 + 记账 + 调用应用层处理"，不直接写寄存器。
 */
static void App_CanRxCallback(uint16 can_id, const uint8* data, uint8 len)
{
#if CAN_PROTOCOL_BRAKE_V2
    uint8  level;
    uint16 cm;
#endif

#if CAN_LOOPBACK_SELFTEST
    /* 回环自测：只认"自己刚发出去的那一帧" —— 三个条件同时成立才算 PASS：
     *   (a) DLC == 8；(b) byte1..3 == 'C','A','N'（帧内容签名）；
     *   (c) byte0(counter) 落在已发出范围内（1..g_can_test_tx_last）。
     * 任一不满足计 bad：避免把残留/总线噪声帧当成自测成功（规则 §6.1 步骤①）。 */
    if (can_id == CAN_ID_SELFTEST) {
        if ((len == 8) && (data[1] == 'C') && (data[2] == 'A') && (data[3] == 'N')
            && (data[0] >= 1u) && (data[0] <= g_can_test_tx_last)) {
            g_can_test_rx_count++;
            g_can_test_pass = TRUE;
        } else {
            g_can_test_echo_bad++;
        }
        return;
    }

#if CAN_PROTOCOL_BRAKE_V2
    /* 回环下 0x125（STM32→板端）同样是自发自收：回读并按小端还原，验证 ③ 的字节契约
     * （123 km/h 与 0xFFFF 两种载荷各一步）。仅自测镜像 + brake_v2 编译，DLC 必须 = 8。 */
    if (can_id == CAN_ID_VEHICLE_SPEED) {
        g_lb_speed_echo_ok = FALSE;
        if (len == 8u) {
            g_lb_speed_echo    = (uint16)(((uint16)data[1] << 8) | (uint16)data[0]);
            g_lb_speed_echo_ok = TRUE;
        }
        return;
    }
#endif /* CAN_PROTOCOL_BRAKE_V2 */
#endif /* CAN_LOOPBACK_SELFTEST */

    /* ---- RDK 行车链路接收侧（0x102 握手应答 / 0x100 心跳命令）----
     * 回调内只做"解析 + 记账 + 边沿打印"：重活（判活超时、状态迁移、斜坡推进、
     * LED 映射）全部留给 20ms/10ms 服务，保持 Can_ProcessRx() 短（规则 §3）。
     * 回环自测镜像里整段不编译（RDK 状态机不参与 03 的判定窗口）。 */
#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST
    if (can_id == CAN_ID_RDK_HS_ACK) {                  /* 0x102 应答 / 心跳 */
        uint8 kind = 0u;
        uint8 seq  = 0u;

        if (Can_ParseRdkHsAck(data, len, &kind, &seq)) {
            Rdk_OnHsAck(kind, seq);                     /* kind/seq 在应用层分角色记账 */
        }
        return;
    }

    if (can_id == CAN_ID_RDK_HEARTBEAT) {               /* 0x100 心跳/命令 */
        Rdk_HbFrame_t f;

        if (Can_ParseRdkHeartbeat(data, len, &f)) {
            Rdk_OnHeartbeat(&f);
        } else {
            Rdk_NoteBadFrame();
        }
        return;
    }
#endif

#if CAN_PROTOCOL_BRAKE_V2
    if (can_id == CAN_ID_BRAKE_CMD) {                   /* 0x123 制动指令 */
        level = BRAKE_LEVEL_NONE;
        if (Can_ParseBrakeCmd(data, len, &level)) {
            Aeb_ApplyBrakeLevel(level);
        } else {
            Aeb_LogMalformedFrame();                    /* M2：1s 限流打印，防刷屏 */
        }
    } else if (can_id == CAN_ID_OBSTACLE_DIST) {        /* 0x124 障碍物距离 */
        cm = AEB_DIST_INVALID;
        if (Can_ParseObstacleDist(data, len, &cm)) {
            g_obstacle_cm = cm;                         /* 仅记录/打印，不驱动动作 */
            g_dist_rx_count++;
        }
    }
#endif /* CAN_PROTOCOL_BRAKE_V2 */
}

/* ================================================================
 *  调度层：非阻塞时间片（规则 §4；任务内禁止 delay_ms —— 规则 §3 历史事故）
 * ================================================================ */
#define TASK_10MS_MS    10u
#define TASK_20MS_MS    20u     /* 编码器测速窗口：必须 = ENCODER_CONTROL_PERIOD_MS(20) */
#define TASK_100MS_MS   100u
#define TASK_1000MS_MS  1000u

/* 回环自测步进 = Task_100ms 每 2 拍；若有人改了任务周期而没同步自测常量，直接编译报错，
 * 避免"步进周期与调度周期静默失配"（契约见 Inc/can_protocol.h 的 AEB_LB_STEP_MS）。
 * ⚠ 该自测只属 brake_v2；heartbeat_v1 分支不引用 AEB_LB_STEP_MS。 */
#if CAN_PROTOCOL_BRAKE_V2
#if (TASK_100MS_MS * 2u) != AEB_LB_STEP_MS
#error "AEB_LB_STEP_MS 必须 = TASK_100MS_MS * 2（自测步进与调度周期失配）"
#endif
#endif /* CAN_PROTOCOL_BRAKE_V2 */

/* 10ms：轮询收帧（FIFO0 整批排空；回环模式下就是"自己发的帧回到自己"）
 *      + heartbeat_v1 的 LED 服务（LED0 命令指示 / LED1 链路指示，非阻塞相位闪烁）*/
/**
 * @brief  10ms 时间片任务：CAN 接收轮询 + heartbeat_v1 的 LED 服务
 * @return 无
 * @note   ① Can_ProcessRx() 一次把 FIFO0 排空并同步回调（回调里禁止长日志/延时）；
 *         ② LED 服务用 millis() 相位闪烁，10ms 拍足以看清 5Hz 快闪；
 *         ③ **禁止在本函数里 delay_ms**（规则 §3，历史丢帧事故）。
 */
static void Task_10ms(void)
{
    Can_ProcessRx();               /* 轮询收帧：一次把 FIFO0 排空并分发给回调查找 */

#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST
    Rdk_LedService(millis());      /* 非阻塞（无 delay_ms），10ms 拍刷新即可看清 5Hz 快闪 */
#endif
}

/* 20ms：0x125 车速采样与上报 + 部分制动服务
 * ⚠ 测速窗口必须保持 20ms（Encoder_GetSpeed* 内部按 20ms 折算）；
 *    左右轮各只调用一次（该接口有状态，会消费本窗口增量）。 */
/**
 * @brief  20ms 时间片任务：编码器测速 → 0x125 车速上报 / 部分制动 / heartbeat_v1 服务
 * @return 无
 * @note   ① 测速窗口必须严格 20ms（Encoder_GetSpeed* 内部按 20ms 折算），且左右轮
 *           各只调一次 —— 该接口有状态，会消费本窗口增量；
 *         ② brake_v2：单轮读数越界/NaN 时退化为用有效那轮，两轮都无效才报 0xFFFF；
 *           Bus-Off 期间同样上报 0xFFFF，不让板端拿旧值做 AEB 决策；
 *         ③ heartbeat_v1：末尾调 Rdk_Service20ms()（0x101 恒发 + 判活 + 斜坡推进）；
 *         ④ 禁止 delay_ms。
 */
static void Task_20ms(void)
{
    float32 vl, vr;
#if CAN_PROTOCOL_BRAKE_V2
    float32 avg, diff, lim;
    boolean vl_ok, vr_ok;
#endif

    if (Encoder_SelfTest()) {
        vl = Encoder_GetSpeedCmPerS();          /* 左轮 cm/s（消费 20ms 增量）*/
        vr = Encoder_GetSpeedCmPerSRight();     /* 右轮 cm/s */
        g_wheel_l_cmps = (sint32)vl;            /* 发布左轮观测值（heartbeat_v1 的 20ms 服务也读它） */
        g_wheel_r_cmps = (sint32)vr;            /* 发布右轮观测值 */

#if CAN_PROTOCOL_BRAKE_V2
        vl_ok = wheel_reading_valid(vl);        /* 左轮读数是否在合理范围（挡掉 NaN/异常大值） */
        vr_ok = wheel_reading_valid(vr);        /* 右轮同理 */

        if (vl_ok && vr_ok) {
            avg  = (vl + vr) * 0.5f;            /* 两轮平均 = 车体中心纵向速度 */
            diff = (vl > vr) ? (vl - vr) : (vr - vl);   /* 两轮速度差（取正） */

            /* 一致性判据：相对 30% 与绝对 0.5km/h(13.9cm/s) 取大者；
             * 超阈只告警（打滑/卡死观测），上报值仍是融合值（P4′ 选项 1） */
            lim = avg * ((float32)AEB_WHEEL_DIFF_PCT / 100.0f);   /* 相对阈值：平均速度的 30% */
            if (lim < AEB_WHEEL_DIFF_CMPS_MIN) lim = AEB_WHEEL_DIFF_CMPS_MIN;   /* 低速时用绝对下限兜底 */
            g_wheel_diff_warn = ((diff > lim) ? TRUE : FALSE);    /* 超阈 → 置告警（Task_100ms 打印）
                                                                   * 注意告警不影响上报值，只做观测 */
        } else if (vl_ok) {                     /* 单轮无效 → 用有效那轮 */
            avg = vl;
            g_wheel_diff_warn = FALSE;          /* 只有一个读数可比，不判一致性 */
        } else if (vr_ok) {
            avg = vr;
            g_wheel_diff_warn = FALSE;
        } else {                                /* 两轮都无效 */
            avg = 0.0f;
            g_wheel_diff_warn = TRUE;           /* 两轮都没数据也可视为异常，置告警便于排查 */
        }

        if (vl_ok || vr_ok) {
            if (avg < 0.0f) avg = -avg;          /* 0x125 是 uint16：倒车按幅值上报 */
            if (avg > 65000.0f) avg = 65000.0f;  /* 防 km/h 溢出（65000×0.036 = 2340）*/
            g_speed_kmh   = (uint16)(avg * 0.036f);  /* §5：cm/s × 0.036 = km/h */
            g_speed_valid = TRUE;                /* 标记本次速度可信（Bus-Off 时仍会被改成 0xFFFF） */
        } else {
            g_speed_kmh   = AEB_SPEED_INVALID;   /* 无有效轮速 → 0xFFFF 占位 */
            g_speed_valid = FALSE;
        }
#endif /* CAN_PROTOCOL_BRAKE_V2 */
    } else {
#if CAN_PROTOCOL_BRAKE_V2
        g_speed_kmh       = AEB_SPEED_INVALID;  /* 编码器自检不过 → 无有效速度 */
        g_speed_valid     = FALSE;
        g_wheel_diff_warn = FALSE;              /* 没有读数就不告警（避免开机刷屏） */
#endif
    }

#if CAN_PROTOCOL_BRAKE_V2 && !CAN_LOOPBACK_SELFTEST
    /* Normal：20ms 周期上报（必须 < 板端 recv_timeout 100ms）。
     * Bus-Off 期间同样上报 0xFFFF：链路恢复前不让板端拿旧值做 AEB 决策 */
    Can_SendVehicleSpeed((g_speed_valid && !Can_IsBusOff()) ? g_speed_kmh
                                                            : AEB_SPEED_INVALID);
#endif

#if CAN_PROTOCOL_BRAKE_V2
    Aeb_ServicePartialBrake();
#endif

#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST
    /* heartbeat_v1 行车链路 20ms 服务：0x101 恒发 / 0x100 判活看门狗 /
     * 巡航斜坡推进 / 60s 锁定窗口（非阻塞，参数取自 g_rdk_cfg） */
    Rdk_Service20ms();
#endif
}

/* 100ms：0x123 失效保护观测 + 左右轮差告警 + 0x124 距离打印 + 等级 1 预警闪烁
 * ⚠ 本任务整体只在 brake_v2 分支有内容；heartbeat_v1 的 100ms 观测在 Task_1000ms 状态行。*/
/**
 * @brief  100ms 时间片任务（仅 brake_v2 有实体内容）：失效保护观测 + 预警闪烁
 * @return 无
 * @note   ① 0x123 超时只告警、**保持锁存不自动解除**（§5 刹车优先，只有等级 0 解除）；
 *         ② 左右轮差告警、0x124 距离变化打印（都只观测，不驱动动作）；
 *         ③ 等级 1 预警时 LED0 闪烁（等级 2/3 常亮、0 熄灭由 Rdk/AEB 其它分支决定）；
 *         ④ 回环自测镜像下每 2 拍推进一步 Aeb_LbSelfTest_Service()；
 *         ⑤ heartbeat_v1 分支本函数为空（其观测在 Task_1000ms 状态行）。
 */
static void Task_100ms(void)
{
#if CAN_PROTOCOL_BRAKE_V2
    static uint16  last_dist        = AEB_DIST_INVALID;
    static boolean timeout_reported = FALSE;
    static boolean diff_reported    = FALSE;
    static uint8   blink            = 0;

    /* 失效保护（§5 刹车优先）：0x123 超时只告警并保持锁存，绝不自动解除。
     * M3：等级 1（预警，本就无制动动作）丢失时单独打一条日志，解锁语义不变
     *     —— 只有收到 level 0 才解除，超时永远不解除。 */
    if (g_brake_rx_seen && (g_brake_level != BRAKE_LEVEL_NONE) &&
        ((millis() - g_brake_rx_ms) >= AEB_BRAKE_TIMEOUT_MS)) {
        if (!timeout_reported) {
            timeout_reported = TRUE;
            if (g_brake_latched) {
                uart_puts("[AEB] 0x123 LOST -> keep brake latched (no auto release)\r\n");
            } else {
                uart_puts("[AEB] 0x123 LOST (warn only, no brake)\r\n");
            }
        }
    } else {
        timeout_reported = FALSE;
    }

    /* 左右轮不一致告警（只告警，不影响上报值） */
    if (g_wheel_diff_warn) {
        if (!diff_reported) {
            diff_reported = TRUE;
            uart_puts("[AEB] WHEEL-DIFF warn: vL=");
            uart_put_sint(g_wheel_l_cmps);
            uart_puts("cm/s vR=");
            uart_put_sint(g_wheel_r_cmps);
            uart_puts("cm/s\r\n");
        }
    } else {
        diff_reported = FALSE;
    }

    /* 0x124 距离：值变化时才打印（联调观察用，不驱动动作） */
    if (g_obstacle_cm != last_dist) {
        last_dist = g_obstacle_cm;
        uart_puts("[AEB] dist=");
        if (g_obstacle_cm == AEB_DIST_INVALID) {
            uart_puts("none\r\n");
        } else {
            uart_put_uint(g_obstacle_cm);
            uart_puts("cm\r\n");
        }
    }

    /* 等级 1 预警：LED 闪烁（不制动）；等级 2/3 由应用层常亮、等级 0 熄灭。
     * ⚠ 本任务体整体属 brake_v2；heartbeat_v1 的 LED0/LED1 由 Rdk_LedService() 独占
     *    （Task_10ms），两协议编译期二选一 → 不存在两个域争同一个灯的情况。 */
    if (!g_brake_latched && (g_brake_level == BRAKE_LEVEL_WARN)) {
        blink ^= 1u;
        if (blink) { Led_Set(TRUE); } else { Led_Set(FALSE); }
    }

#if CAN_LOOPBACK_SELFTEST
    /* ⑤-B 回环 AEB 自测：每 2 拍（= AEB_LB_STEP_MS = 200ms）推进一步，非阻塞 */
    g_lb_tick ^= 1u;
    if (g_lb_tick == 0u) Aeb_LbSelfTest_Service();
#endif
#endif /* CAN_PROTOCOL_BRAKE_V2 —— heartbeat_v1 本任务为空，观测在 Task_1000ms */
}

/* 1000ms：链路诊断（TxErr/RxErr/BusOff/FIFO 溢出）+ AEB 观测
 * 两种模式都打印，便于在回环阶段先验证车速链路（不依赖 CAN 硬件） */
/**
 * @brief  1s 时间片任务：CAN 总线自愈 + 链路诊断状态行（含 AEB/RDK 观测）
 * @return 无
 * @note   ① 先 Can_RecoverIfStuck()：每秒把卡在 init 态的节点拉回总线，成功日志限流
 *           （第 1 次 + 每 10 次）；
 *         ② 打印 TxErr/RxErr/BusOff/Ovf/Rec 以及协议相关状态（AEB：lvl/latched/kmh/
 *           vL/vR/dist；RDK：st/cmd/pwm/ack/req/hb/dup/bad/age100/age102）；
 *         ③ 回环镜像下打印 [CAN-LB] 判定并每秒补发一帧 0x111；
 *         ④ 这是唯一的周期日志出口，调试信息尽量加在这里而不是 10ms 任务里。
 */
static void Task_1000ms(void)
{
    /* CAN 自愈（§6.4）：板端掉电/线松/缺 120Ω 终端时，CAN 可能被硬件停在初始化态
     * （INAK=1/INRQ=0、3 邮箱永久占满），硬件 ABOM 出不来 → 本层每秒拉一次，
     * 恢复动作全在 can_driver 内部（有界 INRQ 往返，不阻塞）。
     * 只有"总线已恢复合格"时才会成功；成功日志限流：第 1 次 + 每 10 次。 */
    if (Can_RecoverIfStuck()) {
        uint16 rc = Can_GetRecoverCount();
        if ((rc == 1u) || ((rc % 10u) == 0u)) {
            uart_puts("[CAN] back on bus after init-mode stall, total=");
            uart_put_uint(rc);
            uart_puts("\r\n");
        }
    }

#if CAN_LOOPBACK_SELFTEST
    uart_puts("[CAN-LB] ");
    uart_puts(g_can_test_pass ? "PASS" : "FAIL");
    uart_puts("  tx=");
    uart_put_uint(g_can_test_tx_counter);
    uart_puts("  rx=");
    uart_put_uint(g_can_test_rx_count);
    uart_puts("  bad=");
    uart_put_uint(g_can_test_echo_bad);
#if CAN_PROTOCOL_BRAKE_V2
    uart_puts("  round=");
    uart_put_uint(g_lb_round);
    uart_puts("  AEBtest=");
    if (g_lb_step != 0u) {
        uart_puts("running/");
        uart_put_uint(g_lb_step);
    } else if (g_lb_last_round_fail == 0u) {
        uart_puts("PASS (");
        uart_put_uint(AEB_LB_STEP_COUNT);
        uart_puts("/");
        uart_put_uint(AEB_LB_STEP_COUNT);
        uart_puts(")");
    } else {
        uart_puts("FAIL step=");
        uart_put_uint(g_lb_fail_step);
        uart_puts(" exp=");
        uart_put_sint(g_lb_fail_exp);
        uart_puts(" act=");
        uart_put_sint(g_lb_fail_act);
    }
#else
    uart_puts("  AEBtest=n/a (heartbeat_v1)");
#endif
#else
    uart_puts("[CAN] ");
#endif
    uart_puts("  TxErr=");
    uart_put_uint(Can_GetTxErrorCount());
    uart_puts("  RxErr=");
    uart_put_uint(Can_GetRxErrorCount());
    uart_puts("  BusOff=");
    uart_put_uint(Can_IsBusOff());
    uart_puts("  Ovf=");
    uart_put_uint(Can_GetRxOverrunCount());
    uart_puts("  Rec=");
    uart_put_uint(Can_GetRecoverCount());
    uart_puts("\r\n");

#if CAN_PROTOCOL_BRAKE_V2
    uart_puts("[AEB] lvl=");
    uart_put_uint(g_brake_level);
    uart_puts(" latched=");
    uart_put_uint(g_brake_latched);
    uart_puts(" kmh=");
    if (g_speed_valid) { uart_put_uint(g_speed_kmh); } else { uart_puts("inv"); }
    uart_puts(" vL=");
    uart_put_sint(g_wheel_l_cmps);
    uart_puts(" vR=");
    uart_put_sint(g_wheel_r_cmps);
    uart_puts(" dist=");
    if (g_obstacle_cm == AEB_DIST_INVALID) { uart_puts("none"); }
    else { uart_put_uint(g_obstacle_cm); }
    uart_puts(" dist_rx=");
    uart_put_uint(g_dist_rx_count);
    uart_puts("\r\n");
#endif /* CAN_PROTOCOL_BRAKE_V2 */

#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST
    /* heartbeat_v1 行车链路状态行（st：0 STATIC / 1 CRUISE / 2 HOLD / 3 LOCK）
     * cmd = 最近一帧有效 0x100 的命令（0xFF=无）；v = 最近一拍轮速 cm/s；
     * age100 = 距最近一帧**有效** 0x100 的毫秒数（≥hb_timeout_ms 即应已进 HOLD）；
     * age102 仅观测 0x102（**不参与判活**）。 */
    uart_puts("[RDK] st=");
    uart_put_uint(RDK_STATE_VAR);
    uart_puts(" cmd=");
    uart_put_uint(g_rdk_cmd);
    uart_puts(" pwm=");
    uart_put_sint(g_rdk_pwm_out);
    uart_puts(" ack=");
    uart_put_uint(g_rdk_ack_count);
    uart_puts(" req=");
    uart_put_uint(g_rdk_req_count);
    uart_puts(" hb=");
    uart_put_uint(g_rdk_hb_rx);
    uart_puts(" dup=");
    uart_put_uint(g_rdk_hb_dup);
    uart_puts(" bad=");
    uart_put_uint(g_rdk_hb_bad);
    uart_puts(" v=");
    uart_put_sint(g_rdk_v_cmps);
    uart_puts("cm/s buzz=");
    uart_put_uint(g_rdk_buzz_on);
    uart_puts(" age100=");
    uart_put_uint(millis() - g_rdk_last_rx_100);
    uart_puts(" age102=");
    uart_put_uint(millis() - g_rdk_last_rx_102);
    uart_puts("\r\n");
#endif

#if CAN_LOOPBACK_SELFTEST
    g_can_test_pass = FALSE;                            /* 开新判定窗口 */
    g_can_test_tx_counter++;
    CanTest_SendFrame();                                /* 每秒 1 帧 0x111 */
#endif
}

/* ================================================================
 *  main（上电：平台 → 外设 → 驱动 → CAN 回环自测 → 10ms/1000ms 调度）
 * ================================================================ */
/**
 * @brief  上电装配与主循环：平台 → 外设 → 驱动 → 自检 → CAN → 10/20/100/1000ms 调度
 * @return 不会返回（while(1) 调度；末尾的 return 0 仅为满足编译器）
 * @note   ① 顺序不可随意调换：Platform_EarlyInit()（关 JTAG 释放 PB3/PB4）→ 时钟 →
 *           SysTick/UART/LED → TIM3/TIM1 → 编码器/电机/舵机 → 舵机自检 → 注册 CAN
 *           回调 → Can_Init/Can_InitLoopback → 协议自测 → Rdk_Init → 调度；
 *         ② 协议分支由编译期宏 CAN_PROTOCOL_HEARTBEAT_V1 / CAN_PROTOCOL_BRAKE_V2 二选一，
 *           必须与板端 can.protocol 一致（规则 §5）；
 *         ③ 调度用累加式时间基准钉住周期，落后超一拍就跳拍（不做追帧突发，避免污染
 *           20ms 测速窗口）；任务内一律不 delay_ms；
 *         ④ 用 CAN1 时禁止插板载 USB SLAVE 口（PA11/PA12 与 OTG_FS 冲突）。
 */
int main(void)
{
    /* ===== 平台层启动期初始化 =====
     * 关 JTAG 释放 PB3(STBY)/PB4(BIN1) + 中断向量表定位到 Flash；
     * 必须在一切 GPIO 配置与中断使能之前执行（原先内联在此，属越层，已收拢） */
    Platform_EarlyInit();

    /* ===== 平台基础 ===== */
    SystemClock_Config();
    SysTick_Init();
    UART1_Init();
    Led_Init();
    Led1_Init();            /* LED1=PF10：heartbeat_v1 握手状态指示（LED0 已被 RDK/AEB 占用） */

    /* ===== 外设：电机 PWM + 舵机 PWM ===== */
    MX_TIM3_PWM_Init();     /* TB6612 PWMA/PWMB @20kHz */
    MX_TIM1_PWM_Init();     /* MG996R 舵机 @50Hz */

    /* ===== 驱动层 ===== */
    Encoder_Init();         /* 左右轮正交计数（TIM4/TIM8，自由计数 0..0xFFFF） */
    Motor_Init();             /* STBY=1, 初始停止(滑行) */
    Servo_Init();         /* 舵机中位 1500us */

    /* 编码器自检（只读检查，不转电机）：0x125 车速链路的前置条件 */
    uart_puts("[ENC] self-test = ");
    uart_put_uint(Encoder_SelfTest());
    uart_puts("\r\n");

    uart_puts("\r\nedge_ai_car boot OK @168MHz\r\n");
#if CAN_LOOPBACK_SELFTEST
    uart_puts("[CAN-LB] mode = silent loopback (BTR.LBKM|SILM), 500kbps\r\n");
#else
    uart_puts("[CAN] mode = normal, 500kbps (TJA1050 VCC=5V + 120ohm x2)\r\n");
#endif

    /* 编译期选中的协议（与板端 can.protocol 必须一致，见 .clinerules §5） */
    uart_puts("[PROTO] ");
#if CAN_PROTOCOL_HEARTBEAT_V1
    uart_puts("heartbeat_v1 (0x100/0x101/0x102)\r\n");
#else
    uart_puts("brake_v2 (0x123/0x124/0x125)\r\n");
#endif

    /* ===== 舵机自检：左右摆动一次 ===== */
    Servo_SelfTest();

    //demo
    // MotionDemo_Run();

    /* ===== CAN：先注册接收回调，再按开关初始化（回环自测 / Normal 联调） =====
     * 注意：用 CAN1 时禁止插板载 USB SLAVE 口（PA11/PA12 与 OTG_FS 冲突） */
#if CAN_LOOPBACK_SELFTEST
    Can_RegisterRxCallback(App_CanRxCallback);

    if (!Can_InitLoopback()) {
        uart_puts("[CAN-LB] init FAIL (loopback)\r\n");
    } else {
        uart_puts("[CAN-LB] init OK: BTR=0xC04E0003 (BRP=4,TS1=15,TS2=5,SJW=1,LBKM=1,SILM=1)\r\n");
        uart_puts("[CAN-LB] boot frame 0x111 sent: {1,'C','A','N',0,0,0,0}\r\n");
    }

    /* 上电自动发一帧自测帧（counter=1），之后每秒补发一帧 */
    g_can_test_tx_counter = 1;
    CanTest_SendFrame();
#else
    Can_RegisterRxCallback(App_CanRxCallback);   /* 0x123 制动等级 / 0x124 距离 */

    if (!Can_Init()) {
        uart_puts("[CAN] init FAIL (normal)\r\n");
    } else {
        uart_puts("[CAN] init OK: BTR=0x004E0003 (BRP=4,TS1=15,TS2=5,SJW=1) 500kbps\r\n");
    }
#endif

#if CAN_PROTOCOL_BRAKE_V2
    /* ===== ⑤-A 解析器自测（brake_v2）：零硬件依赖、开机跑一次
     * 验证 §5 的字节契约（等级 0~3 合法 / 4 被拒 / 0x124 小端 / 0x123·0x124 的 DLC≠8 被拒），
     * 不发帧、不动执行机构；失败会逐条打印，最后给 "parser PASS (8/8)"。===== */
    Aeb_SelfTestParser();
#endif

#if CAN_PROTOCOL_HEARTBEAT_V1 && !CAN_LOOPBACK_SELFTEST
    /* ===== heartbeat_v1 行车链路（0x100/0x101/0x102，仅 Normal 镜像）=====
     * 顺序要紧：先跑纯软件解析器自测（不自发帧、不动执行机构），
     * 再 Rdk_Init() 真正"上电 PWM=0 静止锁死 + 发第 1 帧 0x101"进握手。 */
    Rdk_SelfTestParser();
    Rdk_Init();
#endif

    /* ===== 调度层：10/20/100/1000ms 非阻塞时间片（永不返回，任务内无 delay_ms）
     * 时间基准用累加式（last += 周期）把周期漂移钉住；若某任务耗时导致落后
     * 超过一个周期，则跳拍到当前时刻（不做"追帧突发"—— 20ms 是编码器测速窗口，
     * 窗口抖动会按比例污染 0x125 车速，见 tools/timing_audit.py 契约项 10） ===== */
    uint32 last_10ms   = millis();
    uint32 last_20ms   = last_10ms;
    uint32 last_100ms  = last_10ms;
    uint32 last_1000ms = last_10ms;

    while (1) {
        uint32 now = millis();      /* 只取一次时刻：四个任务用同一基准判超时 */

        if ((now - last_10ms) >= TASK_10MS_MS) {
            last_10ms += TASK_10MS_MS;      /* 累加式推进：钉住周期、不积累漂移 */
            if ((now - last_10ms) >= TASK_10MS_MS) last_10ms = now;   /* 已落后超过一个周期 → 跳拍到当前，不做追帧突发 */
            Task_10ms();                     /* 收帧 + LED 服务 */
        }
        if ((now - last_20ms) >= TASK_20MS_MS) {
            last_20ms += TASK_20MS_MS;      /* 20ms 是编码器测速窗口，必须稳定（见 timing_audit 契约） */
            if ((now - last_20ms) >= TASK_20MS_MS) last_20ms = now;
            Task_20ms();                     /* 测速 + 0x125 上报 / 0x101 恒发 + 判活 */
        }
        if ((now - last_100ms) >= TASK_100MS_MS) {
            last_100ms += TASK_100MS_MS;
            if ((now - last_100ms) >= TASK_100MS_MS) last_100ms = now;
            Task_100ms();                    /* 失效保护观测 / 距离打印 / 预警闪烁 */
        }
        if ((now - last_1000ms) >= TASK_1000MS_MS) {
            last_1000ms += TASK_1000MS_MS;
            if ((now - last_1000ms) >= TASK_1000MS_MS) last_1000ms = now;
            Task_1000ms();                   /* CAN 自愈 + 链路诊断日志 */
        }
    }

    return 0;   /* 不会到这里 */
}
