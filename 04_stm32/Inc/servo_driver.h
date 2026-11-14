/**
 * servo_driver.h - 舵机驱动抽象层（Servo Driver Abstraction）
 *
 * 硬件：MG996R 舵机，50Hz PWM
 *   PA8 = TIM1_CH1（高级定时器，需开 MOE）
 *   TIM1 时钟 168MHz：PSC=167 -> 1MHz，ARR=19999 -> 50Hz
 *   CCR 数值 = 脉冲宽度(us)，中位 1500us，范围 1000~2000us
 *
 * 供电：舵机必须独立 5-6V 供电（MG996R 峰值电流可达 2A+），
 *       严禁从 STM32 3.3V 取电！GND 与系统共地。
 */
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "platform.h"

/* ===== 舵机参数配置 ===== */
#define SERVO_PWM_FREQ_HZ        50u       /* 重复频率 50Hz（20ms 周期），MG996R 标准驱动频率 */
#define SERVO_CENTER_PULSE_US    1500u     /* 中位脉宽 1500us = 0°（前轮打正） */
#define SERVO_MAX_LEFT_PULSE_US  2000u     /* +角度方向 = 右转（按需对调） */
#define SERVO_MAX_RIGHT_PULSE_US 1000u
#define SERVO_MAX_ANGLE_DEG      30

/* ===== 初始化 ===== */

/**
 * @brief  初始化舵机通道（TIM1 PWM 由 main.c 的 MX_TIM1_PWM_Init 启动，这里只回中位）
 * @param  无
 * @return 无
 * @note   等价于 Servo_SetAngle(0)：脉冲 1500us = 机械中位（前轮打正）。
 *         ⚠ 舵机必须独立 5~6V 供电，严禁从 STM32 3.3V 取电（峰值 2A+）。
 */
void Servo_Init(void);

/* ===== 角度控制 ===== */

/**
 * @brief  设置转向角度（角度域接口，推荐业务层用这个）
 * @param  angle 角度，单位度，范围 -30 ~ +30（0 = 中位/直行）；
 *               越界自动夹到 ±30，不会报错
 * @return 无
 * @note   内部线性换算：脉冲 = 1500us + angle × (2000-1500)/30，
 *         即 ±30° → 1000us ~ 2000us（MG996R 标准行程），再写 TIM1->CCR1。
 *         立即生效、不阻塞（舵机机械到位需要时间，由调用方按需留间隔）。
 */
void Servo_SetAngle(sint8 angle);

/**
 * @brief  读最近一次设定的角度（软件缓存，不读硬件）
 * @param  无
 * @return -30 ~ +30 度；若之前用 Servo_SetPulse() 直接改了脉宽，这里**仍是旧值**
 * @note   只反映"上次通过 Servo_SetAngle() 设了什么"，不代表舵机真实位置。
 */
sint8 Servo_GetAngle(void);

/* ===== 直接脉冲控制（底层调试用） ===== */

/**
 * @brief  直接写脉宽（跳过角度换算，底层调试口）
 * @param  pulse_us 脉宽，单位微秒，范围 1000 ~ 2000（1500 = 中位）；
 *                  越界自动夹到 1000/2000，防超行程堵转烧舵机
 * @return 无
 * @note   ⚠ 不更新角度缓存（Servo_GetAngle() 会与实际不符）；
 *         正常业务请用 Servo_SetAngle()，本接口只在示波器/行程标定时用。
 */
void Servo_SetPulse(uint16 pulse_us);

/* ===== 自检 ===== */

/**
 * @brief  舵机自检：左右各打一次再回中，肉眼确认机械行程与供电
 * @param  无
 * @return TRUE = TIM1 已启动且摆动流程跑完；FALSE = 定时器未启动（写 CCR 无波形）
 * @note   ⚠ 内部含 2×300ms 的 delay_ms 阻塞，**只允许开机自检调用一次**，
 *         严禁放进 Task_*（规则 §3）。
 */
boolean Servo_SelfTest(void);

#endif /* SERVO_DRIVER_H */
