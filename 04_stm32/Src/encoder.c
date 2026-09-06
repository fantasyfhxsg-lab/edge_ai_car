/**
 * encoder.c - 编码器实现
 *
 * 硬件连接：
 *   - 编码器A相: PA6 (TIM3_CH1)
 *   - 编码器B相: PA7 (TIM3_CH2)
 *   - TIM3 配置为编码器模式（正交解码，4倍频）
 *
 * 工作原理：
 *   正交编码器输出 A、B 两相方波，相位差90°
 *   STM32 定时器编码器模式自动计数，正转+1，反转-1
 *   4倍频模式下，一个方波周期计4个数
 *
 * 后期迁移 AUTOSAR 时：
 *   - __HAL_TIM_GET_COUNTER() → Gpt_GetTimerValue() 或 ICU 接口
 *   - 头文件接口不变
 */
#include "encoder.h"

/* ===== 外部硬件句柄 ===== */
extern TIM_HandleTypeDef htim3;

/* ===== 内部状态 ===== */
static sint16 g_last_count = 0;   /* 上次读取的计数值 */

/* ===== 初始化 ===== */
void Encoder_Init(void)
{
    /* 启动定时器编码器模式 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* 清零计数器 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    g_last_count = 0;
}

/* ===== 计数读取 ===== */
sint16 Encoder_GetCount(void)
{
    return (sint16)__HAL_TIM_GET_COUNTER(&htim3);
}

sint16 Encoder_GetDelta(void)
{
    sint16 current_count;
    sint16 delta;

    current_count = Encoder_GetCount();

    /*
     * 计算增量，处理 int16 溢出回绕
     * 例：上次=32000，这次=-32000（溢出回绕）
     *     delta = -32000 - 32000 = -64000（错误）
     *     正确处理：delta 应该是正的小数值
     *
     * int16 溢出回绕处理：
     *   delta = (int16)(current - last)
     *   利用 int16 截断自动处理回绕
     */
    delta = (sint16)(current_count - g_last_count);
    g_last_count = current_count;

    return delta;
}

/* ===== 速度计算 ===== */
float32 Encoder_GetSpeedRPM(void)
{
    /*
     * RPM 换算：
     *   delta = 20ms 内的脉冲数
     *   每秒脉冲数 = delta * (1000 / 20) = delta * 50
     *   每分钟脉冲数 = delta * 50 * 60 = delta * 3000
     *   RPM = 每分钟脉冲数 / 每转脉冲数 = delta * 3000 / ENCODER_PPR
     */
    sint16 delta = Encoder_GetDelta();
    float32 rpm = ((float32)delta * 3000.0f) / (float32)ENCODER_PPR;
    return rpm;
}

float32 Encoder_GetSpeedCmPerS(void)
{
    /*
     * 线速度换算：
     *   车轮周长 = π * 直径 = π * 6.5cm ≈ 20.42cm
     *   每秒转数 = RPM / 60
     *   线速度(cm/s) = 每秒转数 * 周长
     *                 = (RPM / 60) * (π * 6.5)
     */
    float32 rpm = Encoder_GetSpeedRPM();
    float32 circumference = 3.14159f * (float32)WHEEL_DIAMETER_MM / 10.0f;  /* cm */
    float32 cm_per_s = (rpm / 60.0f) * circumference;
    return cm_per_s;
}

/* ===== 清零 ===== */
void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    g_last_count = 0;
}

/* ===== 自检 ===== */
boolean Encoder_SelfTest(void)
{
    /*
     * 自检：
     * 1. 检查定时器状态
     * 2. 手动写入计数器，读取验证
     * 3. 恢复清零
     *
     * 实际项目中可以让电机转一下，看计数是否变化
     */
    if (htim3.State != HAL_TIM_STATE_READY)
    {
        return FALSE;
    }

    /* 测试计数器读写 */
    __HAL_TIM_SET_COUNTER(&htim3, 1000);
    if (Encoder_GetCount() != 1000)
    {
        return FALSE;
    }

    Encoder_Reset();
    return TRUE;
}
