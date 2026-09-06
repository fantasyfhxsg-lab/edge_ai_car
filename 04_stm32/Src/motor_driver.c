/**
 * motor_driver.c - 电机驱动实现
 *
 * 硬件连接：
 *   - PWM:  TIM2_CH1 (PA0), 20kHz
 *   - 方向: PB0 (高=反转, 低=正转)
 *   - 电机驱动板: TB6612 / BTS7960
 *
 * 后期迁移 AUTOSAR 时：
 *   - __HAL_TIM_SET_COMPARE() → Pwm_SetDutyCycle()
 *   - HAL_GPIO_WritePin()      → Dio_WriteChannel()
 *   - 头文件接口不变，业务代码不用改
 */
#include "motor_driver.h"

/* ===== 外部硬件句柄（由 main.c 中 CubeMX 生成） ===== */
extern TIM_HandleTypeDef htim2;   /* PWM定时器 */

/* ===== 内部状态 ===== */
static sint16       g_current_speed = 0;     /* 当前设置速度 -1000~1000 */
static MotorState_t g_state = MOTOR_STATE_STOP;  /* 当前电机状态 */

/* ===== 初始化 ===== */
void Motor_Init(void)
{
    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /* 初始状态：停止 */
    Motor_SetSpeed(0);
    g_state = MOTOR_STATE_STOP;
}

/* ===== 速度控制 ===== */
void Motor_SetSpeed(sint16 speed)
{
    /* 限幅 */
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    g_current_speed = speed;

    if (speed == 0)
    {
        /* 停止：PWM=0，方向引脚保持 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        g_state = MOTOR_STATE_STOP;
    }
    else if (speed > 0)
    {
        /* 正转：方向引脚低电平 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)speed);
        g_state = MOTOR_STATE_FORWARD;
    }
    else /* speed < 0 */
    {
        /* 反转：方向引脚高电平 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(-speed));
        g_state = MOTOR_STATE_REVERSE;
    }
}

sint16 Motor_GetSpeed(void)
{
    return g_current_speed;
}

/* ===== 急刹控制 ===== */
void Motor_EmergencyBrake(void)
{
    /*
     * 紧急制动原理：
     * 把电机两个输入端都接低电平（或都接高电平），
     * 电机线圈短路，产生反向电动势制动，比自由停快很多。
     *
     * 这里用 PWM=0 + 方向引脚保持的方式实现短路制动。
     * 对应 AUTOSAR：安全相关输出，需要功能安全监控。
     */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    g_state = MOTOR_STATE_BRAKE;
    g_current_speed = 0;
}

void Motor_ReleaseBrake(void)
{
    /* 解除制动，恢复到停止状态 */
    if (g_state == MOTOR_STATE_BRAKE)
    {
        g_state = MOTOR_STATE_STOP;
    }
}

/* ===== 状态查询 ===== */
MotorState_t Motor_GetState(void)
{
    return g_state;
}

/* ===== 自检 ===== */
boolean Motor_SelfTest(void)
{
    /*
     * 自检流程：
     * 1. 检查 PWM 定时器是否正常
     * 2. 输出 50% PWM，检查占空比寄存器是否正确写入
     * 3. 恢复停止
     *
     * 实际项目中可以加电流检测、堵转检测等
     * 对应 AUTOSAR DEM（故障检测管理）
     */

    /* 检查定时器句柄状态 */
    if (htim2.State != HAL_TIM_STATE_READY)
    {
        return FALSE;
    }

    /* 测试 PWM 写入 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
    if (__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1) != 500)
    {
        return FALSE;
    }

    /* 恢复停止 */
    Motor_SetSpeed(0);

    return TRUE;
}
