/**
 * motor_driver.h - 电机驱动抽象层（Motor Driver Abstraction）
 *
 * 作用：封装电机所有硬件操作（PWM输出、方向控制、急刹），
 *       业务逻辑只调用这里的接口，不直接碰 HAL 库。
 *
 * 对应 AUTOSAR 层级：
 *   - 接口定义  → ECU抽象层（IoHwAb）
 *   - 内部实现  → MCAL层（Pwm、Dio）
 *
 * 后期迁移 AUTOSAR 时：
 *   - 头文件接口不变（业务代码不用改）
 *   - 源文件内部把 HAL 调用换成 AUTOSAR Pwm_SetDutyCycle / Dio_WriteChannel
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "platform.h"

/* ===== 电机状态定义 ===== */
typedef enum {
    MOTOR_STATE_STOP = 0,    /* 停止     */
    MOTOR_STATE_FORWARD,     /* 正转     */
    MOTOR_STATE_REVERSE,     /* 反转     */
    MOTOR_STATE_BRAKE        /* 急刹(短路制动) */
} MotorState_t;

/* ===== 初始化 ===== */
/**
 * @brief 初始化电机驱动（PWM定时器、方向GPIO）
 * @note  在 main() 中调用一次，必须在 HAL_Init() 之后
 */
void Motor_Init(void);

/* ===== 速度控制 ===== */
/**
 * @brief 设置电机速度（带方向）
 * @param speed 速度值，范围 -1000~1000
 *              正数=正转，负数=反转，0=停止
 *              绝对值对应 PWM 占空比（0~1000对应0%~100%）
 * @note  这是开环速度设置，闭环PID在业务层计算后调用此函数
 */
void Motor_SetSpeed(sint16 speed);

/**
 * @brief 获取当前设置的速度值
 * @return 当前速度值 -1000~1000
 */
sint16 Motor_GetSpeed(void);

/* ===== 急刹控制 ===== */
/**
 * @brief 紧急制动（电机短路刹车，比自由停更快）
 * @note  AEB触发时调用，对应 AUTOSAR 的安全相关输出
 */
void Motor_EmergencyBrake(void);

/**
 * @brief 解除制动，恢复正常速度控制
 */
void Motor_ReleaseBrake(void);

/* ===== 状态查询 ===== */
/**
 * @brief 获取当前电机状态
 * @return MotorState_t 枚举值
 */
MotorState_t Motor_GetState(void);

/**
 * @brief 电机驱动自检（检查PWM输出是否正常）
 * @return TRUE=正常, FALSE=故障
 * @note  上电时调用，对应 AUTOSAR 的 DEM 故障检测
 */
boolean Motor_SelfTest(void);

#endif /* MOTOR_DRIVER_H */
