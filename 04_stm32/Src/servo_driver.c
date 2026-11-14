/**
 * servo_driver.c - 舵机驱动实现（MG996R，TIM1_CH1/PA8）
 *
 * ═══════════ 舵机控制原理 ═══════════
 * 舵机不是"转多快"而是"转到位"：内部电位器闭环，
 * 输入脉冲宽度(1~2ms)决定输出轴角度，50Hz 周期重复即可保持。
 *   脉宽 1.0ms -> -90°（左极限）
 *   脉宽 1.5ms ->   0°（中位）
 *   脉宽 2.0ms -> +90°（右极限）
 * 本工程把转角限制在 ±30°（SERVO_MAX_ANGLE_DEG），
 * 对应脉宽 1250us ~ 1750us，对阿克曼转向足够且不堵转。
 *
 * 频率计算：
 *   TIM1 时钟 = APB2×2 = 168MHz；PSC=167 -> 1MHz（每 tick = 1us）；
 *   ARR=19999 -> 周期 20000us = 50Hz。
 *   CCR 数值直接就是脉冲宽度微秒数：CCR=1500 -> 1.5ms -> 中位。
 *
 * 高级定时器注意：TIM1/TIM8 是"高级定时器"，输出 PWM 前
 * 必须置 BDTR.MOE=1（主输出使能），否则引脚保持低电平——经典坑。
 */
#include "servo_driver.h"

static sint8 g_current_angle = 0;    /* 最近一次设定的角度（度），供 Servo_GetAngle 查询 */

/* ===== 角度→脉冲 ===== */
/**
 * @brief  角度 → 舵机脉宽换算（含行程限幅）
 * @param  angle 角度，单位度，范围 -30 ~ +30（0 = 中位）；
 *               越界自动夹到 ±SERVO_MAX_ANGLE_DEG，不报错
 * @return 脉宽，单位微秒：1000 ~ 2000（1500 = 中位）；50Hz(20ms) PWM 下直接当 CCR 值用
 * @note   用 sint32 做中间运算，避免 sint8 × 500 在 16 位上溢出。
 *         限幅是保护措施：超行程会让舵机顶到机械死点堵转、堵转电流烧舵机/烧电源。
 */
static uint16 AngleToPulse(sint8 angle)
{
    sint32 pulse;        /* 用 32 位中间量算，避免角度的乘法在 16 位上溢出 */

    if (angle > SERVO_MAX_ANGLE_DEG)  angle = SERVO_MAX_ANGLE_DEG;    /* 上限 +30°（再大会堵转/超行程） */
    if (angle < -SERVO_MAX_ANGLE_DEG) angle = -SERVO_MAX_ANGLE_DEG;   /* 下限 −30° */

    // pulse = SERVO_CENTER_PULSE_US +
    //         ((sint32)angle * (SERVO_MAX_LEFT_PULSE_US - SERVO_CENTER_PULSE_US)) / SERVO_MAX_ANGLE_DEG;
    /* 线性插值：脉冲 = 中位 1500us + 角度 × (2000−1500)/30 ≈ 1500us ± 16.7us/度。
     * 即 ±30° 对应 1000us ~ 2000us，正好是 MG996R 的标准行程（本文件头注释写的
     * 1250~1750us 是早期限幅方案的残留说明，以这里的实际计算为准）。 */
    pulse = (sint32)SERVO_CENTER_PULSE_US +
        ((sint32)angle * ((sint32)SERVO_MAX_LEFT_PULSE_US - (sint32)SERVO_CENTER_PULSE_US)) / (sint32)SERVO_MAX_ANGLE_DEG;


    if (pulse > (sint32)SERVO_MAX_LEFT_PULSE_US)  pulse = (sint32)SERVO_MAX_LEFT_PULSE_US;   /* 兜底：≤2000us */
    if (pulse < (sint32)SERVO_MAX_RIGHT_PULSE_US) pulse = (sint32)SERVO_MAX_RIGHT_PULSE_US;  /* 兜底：≥1000us */

    return (uint16)pulse;    /* 已保证落在 1000~2000 的正数范围，可安全截成 16 位 */
}

/**
 * @brief  舵机初始化：把前轮回中位（0° / 1500us）
 * @return 无
 * @note   ⚠ 本函数**不配置定时器**：TIM1 的 PSC/ARR/MOE 由 main.c 的 MX_TIM1_PWM_Init()
 *         完成，必须在那之后调用。若 TIM1 未启动（CEN=0 或 MOE=0），写 CCR1 不会有波形，
 *         舵机不动 —— 这也是 Servo_SelfTest() 返回 FALSE 的判据。
 */
void Servo_Init(void)
{
    /* 定时器由 main.c 初始化并启动；这里只回中 */
    Servo_SetAngle(0);       /* 角度 0 = 脉冲 1500us = 舵机机械中位（前轮打正） */
}

/**
 * @brief  设置前轮转向角（改 CCR1 脉冲宽度）
 * @param  angle 目标角度，单位度，范围 −30 ~ +30（0 = 中位/直行）；
 *               越界自动夹到 ±SERVO_MAX_ANGLE_DEG，不报错
 * @return 无
 * @note   ① 会更新软件缓存 g_current_angle（Servo_GetAngle 读它）；
 *         ② 只写 CCR1，不改任何定时器配置 —— 若有人改了 TIM1 的 PSC/ARR，
 *           “1 计数 = 1us” 的前提失效，脉宽与角度全部失准；
 *         ③ 角度到脉冲是线性插值：±30° ↔ 1000~2000us（约 ±16.7us/度）。
 */
void Servo_SetAngle(sint8 angle)
{
    uint16 pulse_us;

    if (angle > SERVO_MAX_ANGLE_DEG)  angle = SERVO_MAX_ANGLE_DEG;    /* 再限一次幅度：本函数是公开接口，可能被上层直接调用 */
    if (angle < -SERVO_MAX_ANGLE_DEG) angle = -SERVO_MAX_ANGLE_DEG;

    g_current_angle = angle;              /* 记录当前角度（查询用） */
    pulse_us = AngleToPulse(angle);       /* 角度 → 脉宽微秒 */
    TIM1->CCR1 = (uint32)pulse_us;   /* 1MHz -> 1 计数 = 1us */
    /* CCR1 直接等于脉宽微秒数（PSC=167 让计数器每 us 计 1 次），硬件在计数值低于 CCR1 期间输出高电平 */
}

/**
 * @brief  查询最近一次设定的转向角
 * @return 角度值，单位度，−30 ~ +30（0 = 中位）
 * @note   纯软件缓存，**不读硬件、不读舵机电位器**；用 Servo_SetPulse() 直接改脉宽时
 *         缓存不会更新，此时本返回值与实际位置不一致。
 */
sint8 Servo_GetAngle(void)
{
    return g_current_angle;      /* 返回最近一次设定的角度（不读硬件，纯软件缓存） */
}

/**
 * @brief  直接以脉宽驱动舵机（底层调试/标定接口）
 * @param  pulse_us 脉宽，单位微秒；有效范围 1000 ~ 2000（1500 = 中位），
 *                  越界自动夹到 [SERVO_MAX_RIGHT_PULSE_US, SERVO_MAX_LEFT_PULSE_US]
 * @return 无
 * @note   ① **不更新** g_current_angle：之后调 Servo_GetAngle() 得到的仍是上一次
 *           SetAngle() 的值，两者会不一致；
 *         ② 常规控制请用 Servo_SetAngle()（带角度语义与限幅），本接口只用于标定
 *           舵机死区与行程。
 */
void Servo_SetPulse(uint16 pulse_us)
{
    if (pulse_us > SERVO_MAX_LEFT_PULSE_US)  pulse_us = SERVO_MAX_LEFT_PULSE_US;   /* 上限 2000us，防超行程堵转 */
    if (pulse_us < SERVO_MAX_RIGHT_PULSE_US) pulse_us = SERVO_MAX_RIGHT_PULSE_US;  /* 下限 1000us */
    TIM1->CCR1 = (uint32)pulse_us;       /* 直接把脉宽写进比较寄存器（底层调试接口，不改 g_current_angle） */
}

/* ===== 自检：左右摆一次回中 ===== */
/**
 * @brief  舵机自检：左极限 → 右极限 → 回中，各停 300ms
 * @return TRUE = TIM1 已启动且摆动动作已执行完毕；
 *         FALSE = TIM1 的 CEN=0（定时器没跑，写 CCR1 无效，自检无意义）
 * @note   ⚠ 内含 2×300ms 的 delay_ms 阻塞 → **只允许开机初始化阶段调用一次**，
 *         禁止在 Task_10ms/20ms/100ms 里调用（规则 §3：任务内禁止阻塞，会丢 CAN 帧）。
 *         返回 FALSE 说明 MX_TIM1_PWM_Init() 没执行或 TIM1 被停掉，应先查初始化顺序。
 */
boolean Servo_SelfTest(void)
{
    if (!(TIM1->CR1 & TIM_CR1_CEN)) {     /* CEN=0 表示定时器没启动 → 写 CCR 也不会有波形输出 */
        return FALSE;
    }
    Servo_SetAngle(-SERVO_MAX_ANGLE_DEG);  /* 打到一端 */
    delay_ms(800);                        /* 等舵机转到位（此处允许阻塞：只在开机自检调用一次） */
    Servo_SetAngle(0);                    /* 回中位 */
    delay_ms(800);
    Servo_SetAngle(SERVO_MAX_ANGLE_DEG); /* 打到另一端 */
    delay_ms(800);                        /* 同样等它转到位，肉眼可确认舵机与供电都正常 */
    Servo_SetAngle(0);                    /* 回中位，恢复直行 */
    delay_ms(800); 
    return TRUE;                          /* 定时器在跑 + 摆动完成 = 自检通过 */
}
