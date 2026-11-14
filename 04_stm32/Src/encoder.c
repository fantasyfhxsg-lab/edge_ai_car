/**
 * encoder.c - 编码器实现（双路，寄存器级）
 *
 * ═══════════ 编码器测速原理 ═══════════
 * JGB37-520 电机尾部霍尔编码器：电机轴每转 11 个脉冲（11PPR）。
 * 减速比 19:1 -> 输出轴每转 = 11×19 = 209 脉冲。
 * TIM 编码器模式 3（SMS=011）对 A/B 两相正交信号 4 倍频：
 *   CNT 每转计数 = 209 × 4 = 836 = ENCODER_PPR
 *
 * 测速公式（20ms 采样一次增量）：
 *   RPM = delta(脉冲数) / (836 脉冲/转) / 0.02s × 60s
 *       = delta × 3000 / 836
 *   （代码里 3000.0f 就是 60/0.02 的合并常数，见 Encoder_GetSpeedRPM）
 *
 * 引脚：
 *   左轮: TIM4 编码器模式 PD12(TIM4_CH1)/PD13(TIM4_CH2)  AF2
 *   右轮: TIM8 编码器模式 PC6 (TIM8_CH1)/PC7 (TIM8_CH2)  AF3
 *   （注意：M144Z-M4 上 PD12/13 同时连 TFTLCD 的 FSMC_A11/A12、
 *    PC6/7 连摄像头 DCMI_D0/D1——小车不插这两个模块就无冲突）
 *
 * 编码器模式 3（SMS=011）：A/B 正交 4 倍频
 * RPM = delta(20ms) × 3000 / 836
 */
#include "encoder.h"

static sint16 g_last_left  = 0;    /* 左轮上一拍的 CNT 快照（算增量用） */
static sint16 g_last_right = 0;    /* 右轮上一拍的 CNT 快照 */

/* ===== 初始化 ===== */
/**
 * @brief  双路编码器初始化：TIM4(左轮 PD12/PD13, AF2) + TIM8(右轮 PC6/PC7, AF3)，
 *         均配成编码器模式 3（A/B 正交 4 倍频），自由计数 0~0xFFFF
 * @return 无
 * @note   ① PSC=0、ARR=0xFFFF、CEN=1 —— tools/timing_audit.py 有契约项检查这三条，别乱改；
 *         ② 同时把软件增量基准 g_last_left/g_last_right 清零，否则第一次 GetDelta()
 *           会算出巨大假增量；
 *         ③ 引脚冲突提示：PD12/13 与 TFTLCD 的 FSMC_A11/A12 复用、PC6/7 与摄像头
 *           DCMI_D0/D1 复用 —— 小车不插这两个模块才不冲突；
 *         ④ 运行期重复调用会把 CNT 清零（相当于丢弃已累计行程）。
 */
void Encoder_Init(void)
{
    /* ---- 左轮 TIM4：PD12=CH1, PD13=CH2 ---- */
    gpio_enable_clock(GPIOD);                                    /* 开 GPIOD 时钟 */
    gpio_mode(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_MODE_AF);    /* 两个编码器脚设为复用功能 */
    gpio_af(GPIOD, GPIO_PIN_12, 2);                              /* PD12 → AF2 = TIM4_CH1 */
    gpio_af(GPIOD, GPIO_PIN_13, 2);                              /* PD13 → AF2 = TIM4_CH2 */

    RCC->APB1ENR |= RCC_TIM4EN;                                  /* TIM4 挂在 APB1 总线上，使能它的时钟 */

    TIM4->CCMR1 = (1U << 0) | (1U << 8);   /* CC1S=01, CC2S=01: TI1/TI2 输入 */
    /* bit[1:0]=01 → 通道1 输入源选 TI1；bit[9:8]=01 → 通道2 输入源选 TI2；
     * 即把 A/B 两相直接接到捕获单元，交给下面的编码器模式解读 */
    TIM4->SMCR = TIM_SMCR_SMS;             /* 编码器模式 3（4倍频） */
    /* SMS=011：A、B 两相都计数（双沿都算）→ 一个正交周期计 4 次，分辨率最高 */
    TIM4->CCER = 0;                        /* 默认极性，捕获使能由编码器模式自动 */
    /* 全 0 = 不反相、上升沿有效；编码器模式下捕获使能由硬件自动接管，无需手动置 CCxE */
    TIM4->PSC = 0;                         /* 不分频：每个边沿都计数（timing_audit 契约项要求 PSC=0） */
    TIM4->ARR = 0xFFFF;                    /* 16 位满量程自动重装：计数跨越 65535 也不丢（增量按有符号差值算） */
    TIM4->CNT = 0;                         /* 计数清零，保证上电从 0 开始 */
    TIM4->CR1 = TIM_CR1_CEN;               /* CEN=1 启动计数器（自检函数会检查这一位） */

    /* ---- 右轮 TIM8：PC6=CH1, PC7=CH2 ---- */
    gpio_enable_clock(GPIOC);                                    /* 开 GPIOC 时钟 */
    gpio_mode(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF);      /* PC6/PC7 设为复用功能 */
    gpio_af(GPIOC, GPIO_PIN_6, 3);                               /* PC6 → AF3 = TIM8_CH1 */
    gpio_af(GPIOC, GPIO_PIN_7, 3);                               /* PC7 → AF3 = TIM8_CH2 */

    RCC->APB2ENR |= RCC_TIM8EN;                                  /* TIM8 挂在 APB2 总线上，使能它的时钟 */

    TIM8->CCMR1 = (1U << 0) | (1U << 8);   /* 与 TIM4 同理：CH1→TI1、CH2→TI2 */
    TIM8->SMCR = TIM_SMCR_SMS;             /* 编码器模式 3（4 倍频） */
    TIM8->CCER = 0;                        /* 不反相、上升沿有效 */
    TIM8->PSC = 0;                         /* 不分频 */
    TIM8->ARR = 0xFFFF;                    /* 16 位满量程 */
    TIM8->CNT = 0;                         /* 清零 */
    TIM8->CR1 = TIM_CR1_CEN;               /* 启动计数 */

    g_last_left  = 0;        /* 增量基准同步归零，否则第一次 GetDelta 会算出巨大的假增量 */
    g_last_right = 0;
}

/* ===== 左轮 ===== */
/**
 * @brief  读左轮硬件计数值（TIM4->CNT）
 * @return 计数值按有符号 16 位解释，−32768 ~ +32767；正负代表转向
 * @note   无副作用、不改变增量基准，可随时调用。计数跨 0xFFFF↔0 会回绕，做绝对位置
 *         时必须用差值累积，不能直接比较两次读数大小。
 */
sint16 Encoder_GetCount(void)
{
    return (sint16)TIM4->CNT;          /* 直接读硬件计数值，截成有符号 16 位 */
}

/**
 * @brief  取左轮自上次调用以来的脉冲增量（同时把基准推进到本次读数）
 * @return 增量，−32768 ~ +32767；顺时针为正、逆时针为负（16 位自然回绕仍正确）
 * @note   ⚠ **有状态接口**：一个采样窗口只能调用一次 —— 第二次调用必然返回 0。
 *         需要同时得到增量与速度时，只调 Encoder_GetSpeedRPM()/GetSpeedCmPerS()
 *         其中之一，它们内部已经消费过增量。
 */
sint16 Encoder_GetDelta(void)
{
    sint16 current = Encoder_GetCount();          /* 本次采样值 */
    sint16 delta = (sint16)(current - g_last_left);   /* 有符号差值：逆时针为负、顺时针为正。
                                                     * 16 位自然回绕在"两次采样差远小于 32767"时依然正确 */
    g_last_left = current;                        /* 更新基准 → 本接口是"有状态"的：一个窗口只能调用一次 */
    return delta;
}

/**
 * @brief  左轮转速换算：RPM = 20ms 脉冲增量 × 3000 / 836
 * @return 转速，单位 rpm（float32，负数表示反转）
 * @note   ① 3000 = 60s/0.02s；836 = 11PPR × 19 减速比 × 4 倍频（输出轴每转计数）；
 *         ② ⚠ **有状态**：内部调用 Encoder_GetDelta() 消费本窗口增量 →
 *           同一窗口内不能再调 GetDelta()/GetSpeedCmPerS()，否则后两者读到 0；
 *         ③ 换算前提是采样窗口恰为 20ms（Task_20ms 周期），窗口抖动会按比例污染结果。
 */
float32 Encoder_GetSpeedRPM(void)
{
    sint16 delta = Encoder_GetDelta();          /* 20ms 窗口内的脉冲增量（消费一次） */
    return ((float32)delta * 3000.0f) / (float32)ENCODER_PPR;
    /* 3000 = 60s / 0.02s（把"每 20ms 的增量"换算成"每分钟"）；
     * ENCODER_PPR=836 = 输出轴每转计数（11PPR × 19 减速比 × 4 倍频） */
}

/**
 * @brief  左轮线速度换算：rpm ÷ 60 × 轮周长（π × 65mm）
 * @return 线速度，单位 cm/s（float32，负数表示倒车）；周长 ≈ 20.42cm
 * @note   ⚠ **有状态**：内部消费本窗口增量（与 GetDelta/GetSpeedRPM 三选一）。
 *         轮径常量来自 WHEEL_DIAMETER_MM，换轮子必须改那里，否则所有速度按比例偏。
 */
float32 Encoder_GetSpeedCmPerS(void)
{
    float32 rpm = Encoder_GetSpeedRPM();        /* 先把增量换成转速 */
    float32 circumference = 3.14159f * (float32)WHEEL_DIAMETER_MM / 10.0f;
    /* 轮周长：π × 直径；直径 65mm ÷ 10 = 6.5cm → 周长 ≈ 20.42cm（÷10 是 mm→cm 单位换算） */
    return (rpm / 60.0f) * circumference;       /* rpm ÷ 60 = 每秒转数，再乘周长 = 线速度 cm/s */
}

/* ===== 右轮 ===== */
/**
 * @brief  读右轮硬件计数值（TIM8->CNT）
 * @return 计数值按有符号 16 位解释，−32768 ~ +32767
 * @note   无副作用，可随时调用；回绕特性与左轮相同。
 */
sint16 Encoder_GetCountRight(void)
{
    return (sint16)TIM8->CNT;          /* 右轮硬件计数器 */
}

/**
 * @brief  取右轮自上次调用以来的脉冲增量
 * @return 增量，−32768 ~ +32767（正负代表转向）
 * @note   ⚠ **有状态接口**：一个采样窗口只能调用一次，第二次返回 0；
 *         与右轮 GetSpeed*Right() 互斥（它们内部会消费增量）。
 */
sint16 Encoder_GetDeltaRight(void)
{
    sint16 current = Encoder_GetCountRight();          /* 本次采样值 */
    sint16 delta = (sint16)(current - g_last_right);   /* 与上一拍的差值 */
    g_last_right = current;                            /* 更新右轮基准（同样是有状态接口） */
    return delta;
}

/**
 * @brief  右轮转速换算：与左轮同一套公式（增量 × 3000 / 836）
 * @return 转速，单位 rpm（float32，负数表示反转）
 * @note   ⚠ **有状态**：内部消费右轮 20ms 增量，同窗口内不能再调 GetDeltaRight()/
 *         GetSpeedCmPerSRight()。左右轮各自独立计数，互不影响。
 */
float32 Encoder_GetSpeedRPMRight(void)
{
    sint16 delta = Encoder_GetDeltaRight();     /* 右轮 20ms 增量 */
    return ((float32)delta * 3000.0f) / (float32)ENCODER_PPR;   /* 与左轮完全同一套换算 */
}

/* 右轮线速度 cm/s：与左轮同一套换算（轮径 WHEEL_DIAMETER_MM）
 * ⚠ 与左轮一样是有状态接口（内部消费 20ms 增量），每个采样窗口只能调用一次 */
/**
 * @brief  右轮线速度换算：rpm ÷ 60 × 轮周长（π × 65mm）
 * @return 线速度，单位 cm/s（float32，负数表示倒车）
 * @note   ⚠ **有状态**：内部消费右轮增量（与 GetDeltaRight/GetSpeedRPMRight 三选一）；
 *         轮径与左轮共用 WHEEL_DIAMETER_MM。
 */
float32 Encoder_GetSpeedCmPerSRight(void)
{
    float32 rpm = Encoder_GetSpeedRPMRight();   /* 右轮转速 */
    float32 circumference = 3.14159f * (float32)WHEEL_DIAMETER_MM / 10.0f;   /* 轮周长 cm ≈ 20.42 */
    return (rpm / 60.0f) * circumference;       /* 每秒转数 × 周长 = cm/s */
}

/* ===== 清零 ===== */
/**
 * @brief  双路编码器清零：硬件 CNT 与软件增量基准一起归零
 * @return 无
 * @note   ① 只清基准不清硬件（或反之），下一次 GetDelta 会算出假增量 —— 两者必须成对清；
 *         ② 不动计数模式（ARR=0xFFFF 自由计数）也不会关 CEN；
 *         ③ 清完后第一次 GetDelta() 从 0 起算，可用于“从此刻开始计里程”。
 */
void Encoder_Reset(void)
{
    TIM4->CNT = 0;           /* 硬件计数器清零 */
    TIM8->CNT = 0;
    g_last_left  = 0;        /* 软件基准同步清零，避免下次算出假增量 */
    g_last_right = 0;
}

/* ===== 自检 ===== */
/**
 * @brief  编码器自检：检查两路定时器是否都在计数（只读，不转电机）
 * @return TRUE = TIM4 与 TIM8 的 CEN 都为 1（计数链路已就绪）；
 *         FALSE = 任一路未启动（该路将永远测不到速度）
 * @note   只验证“硬件已使能”，**不验证轮子是否真的在转**，也不校验读数合理性；
 *         读数合理性（越界/NaN）由应用层 wheel_reading_valid() 负责。
 *         本函数无副作用，可在 20ms 任务里每拍调用做前置判断。
 */
boolean Encoder_SelfTest(void)
{
    if (!(TIM4->CR1 & TIM_CR1_CEN) || !(TIM8->CR1 & TIM_CR1_CEN)) {
        /* 任一路定时器 CEN=0（没启动）→ 编码器不会计数，测速链路必然失效 */
        return FALSE;
    }
    return TRUE;             /* 两路计数都在跑 = 自检通过（不断言轮子是否转，只查硬件已使能） */
}
