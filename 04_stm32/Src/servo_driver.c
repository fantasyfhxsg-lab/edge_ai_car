/**
 * servo_driver.c - 舵机驱动实现
 *
 * 硬件连接：
 *   - PWM: TIM2_CH2 (PA1), 50Hz
 *   - 舵机: MG996R, 脉冲范围 1000~2000us
 *
 * 定时器配置：
 *   TIM2 时钟 84MHz (APB1*2)
 *   PSC = 167 → 计数时钟 500kHz
 *   ARR = 9999 → PWM频率 50Hz (500kHz/10000)
 *   CCR = 500~1000 → 脉冲 1000~2000us
 *
 * 后期迁移 AUTOSAR 时：
 *   - __HAL_TIM_SET_COMPARE() → Pwm_SetDutyCycle()
 *   - 头文件接口不变
 */
#include "servo_driver.h"

/* ===== 外部硬件句柄 ===== */
extern TIM_HandleTypeDef htim2;

/* ===== 内部状态 ===== */
static sint8  g_current_angle = 0;    /* 当前转角（度） */

/* ===== 初始化 ===== */
void Servo_Init(void)
{
    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    /* 舵机回中 */
    Servo_SetAngle(0);
}

/* ===== 角度→脉冲换算 ===== */
/**
 * @brief 角度换算成 PWM 脉冲宽度
 * @param angle 转角 -30~+30（度）
 * @return 脉冲宽度（微秒），范围 1000~2000
 *
 * 换算公式：
 *   pulse = center + (angle / max_angle) * (max_left - center)
 *
 * 例：angle=+30 → pulse = 1500 + (30/30)*(2000-1500) = 2000us (右转最大)
 *     angle=-30 → pulse = 1500 + (-30/30)*(2000-1500) = 1000us (左转最大)
 */
static uint16 AngleToPulse(sint8 angle)
{
    sint32 pulse;

    /* 限幅 */
    if (angle > SERVO_MAX_ANGLE_DEG)  angle = SERVO_MAX_ANGLE_DEG;
    if (angle < -SERVO_MAX_ANGLE_DEG) angle = -SERVO_MAX_ANGLE_DEG;

    /* 线性换算 */
    pulse = SERVO_CENTER_PULSE_US +
            ((sint32)angle * (SERVO_MAX_LEFT_PULSE_US - SERVO_CENTER_PULSE_US)) / SERVO_MAX_ANGLE_DEG;

    /* 再次限幅，防止计算溢出 */
    if (pulse > SERVO_MAX_LEFT_PULSE_US)  pulse = SERVO_MAX_LEFT_PULSE_US;
    if (pulse < SERVO_MAX_RIGHT_PULSE_US) pulse = SERVO_MAX_RIGHT_PULSE_US;

    return (uint16)pulse;
}

/* ===== 角度控制 ===== */
void Servo_SetAngle(sint8 angle)
{
    uint16 pulse_us;

    /* 限幅 */
    if (angle > SERVO_MAX_ANGLE_DEG)  angle = SERVO_MAX_ANGLE_DEG;
    if (angle < -SERVO_MAX_ANGLE_DEG) angle = -SERVO_MAX_ANGLE_DEG;

    g_current_angle = angle;

    /* 角度→脉冲→CCR值 */
    pulse_us = AngleToPulse(angle);
    /*
     * CCR换算：
     *   PWM频率 50Hz，ARR=9999，计数时钟 500kHz
     *   1us = 0.5 个计数 (500kHz = 0.5us/计数)
     *   CCR = pulse_us * 0.5 = pulse_us / 2
     *
     * 例：1500us → CCR=750, 2000us → CCR=1000, 1000us → CCR=500
     */
    uint32_t ccr = (uint32_t)pulse_us / 2u;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
}

sint8 Servo_GetAngle(void)
{
    return g_current_angle;
}

/* ===== 直接脉冲控制 ===== */
void Servo_SetPulse(uint16 pulse_us)
{
    /* 限幅 */
    if (pulse_us > SERVO_MAX_LEFT_PULSE_US)  pulse_us = SERVO_MAX_LEFT_PULSE_US;
    if (pulse_us < SERVO_MAX_RIGHT_PULSE_US) pulse_us = SERVO_MAX_RIGHT_PULSE_US;

    uint32_t ccr = (uint32_t)pulse_us / 2u;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
}

/* ===== 自检 ===== */
boolean Servo_SelfTest(void)
{
    /*
     * 自检流程：
     * 1. 左转最大 → 等待500ms
     * 2. 右转最大 → 等待500ms
     * 3. 回中
     *
     * 实际项目中可以加角度反馈（电位器）验证
     */

    /* 检查定时器状态 */
    if (htim2.State != HAL_TIM_STATE_READY)
    {
        return FALSE;
    }

    /* 测试 PWM 写入 */
    Servo_SetAngle(SERVO_MAX_ANGLE_DEG);
    HAL_Delay(200);
    Servo_SetAngle(-SERVO_MAX_ANGLE_DEG);
    HAL_Delay(200);
    Servo_SetAngle(0);

    return TRUE;
}
