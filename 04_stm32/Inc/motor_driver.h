/**
 * motor_driver.h - 电机驱动抽象层（Motor Driver Abstraction）
 *
 * 硬件：TB6612FNG 双通道 H 桥
 *   A通道(左轮): PWMA=PA6(TIM3_CH1), AIN1=PB0, AIN2=PB1
 *   B通道(右轮): PWMB=PA7(TIM3_CH2), BIN1=PB4, BIN2=PB5
 *   STBY=PB3（必须拉高）
 *
 * 接口风格与脚手架一致：业务层只调用 Motor_xxx()，不直接碰寄存器。
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
 * @brief  初始化 TB6612 方向脚与软件状态（PWM 定时器由 main.c 的 MX_TIM3_PWM_Init 启动）
 * @param  无
 * @return 无
 * @note   开 GPIOB 时钟 → PB3(STBY)=1 整片使能 → PB0/PB1/PB4/PB5 配推挽输出
 *         → CCR1/CCR2 置 0（上电别让轮子窜）→ 速度/状态/目标全部归零。
 *         ⚠ STBY 必须为高，否则芯片待机、所有输出高阻（表现为"电机完全不动"）。
 */
void Motor_Init(void);

/* ===== 速度控制（阿克曼：两轮同速） ===== */
/**
 * @brief  设置电机速度（两轮同速，带方向，带斜坡）
 * @param  speed 目标速度，量程 ±1000（±1000 ≈ 满占空比）：
 *               正 = 前进，负 = 后退，0 = 停止（IN1=IN2=L，滑行，不是短路制动）
 * @return 无
 * @note   ① 入参先被夹到 ±1000；② 内部按 RAMP_STEP=80/次 做斜坡（每次调用 ≈ 一拍），
 *         所以**输出不会跳变**，需周期性调用（本项目 20ms 一拍）才能爬到目标；
 *         ③ 每拍调用一次即可，重复调用同一目标不会超调。
 */
void Motor_SetSpeed(sint16 speed);

/**
 * @brief  左右轮独立设置速度（差速/转向调试用，**无斜坡、立即生效**）
 * @param  left  左轮速度，±1000；正=前进，负=后退，0=滑行停止
 * @param  right 右轮速度，±1000；语义同 left
 * @return 无
 * @note   本函数不做差速算法，也不做斜坡限幅 —— 直接换算成占空比输出，
 *         缩放/斜坡由调用方负责（正常行驶请用 Motor_SetSpeed）。
 *         状态归类仅供查询：两轮皆 0 → STOP；任一为正 → FORWARD；否则 REVERSE。
 */
void Motor_SetSpeedLR(sint16 left, sint16 right);

/**
 * @brief  读当前**实际输出**速度（斜坡处理之后的值）
 * @param  无
 * @return -1000 ~ +1000；与最近一次设定的目标值可能不同（还在爬坡时）
 * @note   想判断"是否已减到 0"用它（例如部分制动服务里判断减速是否到位），
 *         不要用目标值判断。
 */
sint16 Motor_GetSpeed(void);

/* ===== 急刹控制 ===== */

/**
 * @brief  短路制动（最强制动）：IN1=IN2=H 把电机两端短接，立即生效
 * @param  无
 * @return 无
 * @note   不走斜坡、不等下一拍 —— 收到紧急制动等级时立刻调用。
 *         同时把占空比清零、软件速度与目标归零（刹停后再起步必须重新爬斜坡）。
 *         ⚠ 短路制动 ≠ 停止：停止是 IN1=IN2=L（滑行），两者行为完全不同。
 */
void Motor_EmergencyBrake(void);

/**
 * @brief  解除短路制动，回到"停止/滑行"态（IN1=IN2=L）
 * @param  无
 * @return 无
 * @note   幂等：当前不是制动态时直接返回，不做任何硬件写。
 *         历史缺陷已修：旧实现只改软件状态、不清 IN1/IN2，硬件仍处短路（"假解除"）。
 *         解除后速度为 0，需要重新 Motor_SetSpeed() 才会动。
 */
void Motor_ReleaseBrake(void);

/* ===== 状态查询 ===== */

/**
 * @brief  读电机当前状态（停止 / 正转 / 反转 / 急刹）
 * @param  无
 * @return MOTOR_STATE_STOP / FORWARD / REVERSE / BRAKE（枚举见本文件上方）
 * @note   供应用层判断"是否已处于短路制动"（避免重复干预），以及日志打印。
 */
MotorState_t Motor_GetState(void);

/**
 * @brief  自检：PWM 通道是否已就绪（只读检查，**不转电机**）
 * @param  无
 * @return TRUE = TIM3 已启动（CEN=1），CCR 写下去会有波形；FALSE = 定时器未启动
 * @note   只查定时器使能，不断言电机/驱动芯片是否真的好；上电自检流程里调用。
 */
boolean Motor_SelfTest(void);

//demo
void demo_forward(sint16 speed);
void demo_reverse(sint16 speed);
#endif /* MOTOR_DRIVER_H */
