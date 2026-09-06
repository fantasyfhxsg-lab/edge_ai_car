/**
 * servo_driver.h - 舵机驱动抽象层（Servo Driver Abstraction）
 *
 * 作用：封装舵机所有硬件操作（PWM脉冲输出），
 *       业务逻辑只调用 SetAngle()，不直接碰定时器寄存器。
 *
 * 对应 AUTOSAR 层级：
 *   - 接口定义  → ECU抽象层（IoHwAb）
 *   - 内部实现  → MCAL层（Pwm）
 *
 * 硬件：MG996R 舵机，50Hz PWM，脉冲范围 1000~2000us
 *       对应 STM32 TIM2_CH2 (PA1)，ARR=10000-1, PSC=168-1 → 50Hz
 */
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "platform.h"

/* ===== 舵机参数配置（根据实际舵机修改） ===== */
#define SERVO_PWM_FREQ_HZ       50u       /* 舵机PWM频率 50Hz（标准） */
#define SERVO_CENTER_PULSE_US   1500u     /* 中位脉冲宽度 1500us */
#define SERVO_MAX_LEFT_PULSE_US 2000u     /* 最大左转脉冲 2000us */
#define SERVO_MAX_RIGHT_PULSE_US 1000u    /* 最大右转脉冲 1000us */
#define SERVO_MAX_ANGLE_DEG     30        /* 最大转角（度），阿克曼底盘通常30° */

/* ===== 初始化 ===== */
/**
 * @brief 初始化舵机驱动（PWM定时器，舵机回中）
 */
void Servo_Init(void);

/* ===== 角度控制 ===== */
/**
 * @brief 设置舵机转角
 * @param angle 转角，范围 -30~+30（度）
 *              负数=左转，正数=右转，0=回中
 * @note  内部自动换算成 PWM 脉冲宽度
 *        对应 AUTOSAR：业务层给角度，IoHwAb 层换算成脉冲，MCAL 输出PWM
 */
void Servo_SetAngle(sint8 angle);

/**
 * @brief 获取当前设置的转角
 * @return 当前转角 -30~+30（度）
 */
sint8 Servo_GetAngle(void);

/* ===== 直接脉冲控制（底层调试用） ===== */
/**
 * @brief 直接设置脉冲宽度（调试用，正常用 SetAngle）
 * @param pulse_us 脉冲宽度，范围 1000~2000（微秒）
 */
void Servo_SetPulse(uint16 pulse_us);

/* ===== 自检 ===== */
/**
 * @brief 舵机自检（左右摆动一次后回中）
 * @return TRUE=正常, FALSE=故障
 */
boolean Servo_SelfTest(void);

#endif /* SERVO_DRIVER_H */
