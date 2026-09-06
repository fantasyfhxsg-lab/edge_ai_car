/**
 * encoder.h - 编码器抽象层（Encoder Abstraction）
 *
 * 作用：封装编码器所有硬件操作（定时器编码器模式读取），
 *       业务逻辑只调用 GetCount() / GetSpeed()，不直接碰定时器。
 *
 * 对应 AUTOSAR 层级：
 *   - 接口定义  → ECU抽象层（IoHwAb）
 *   - 内部实现  → MCAL层（Gpt / ICU）
 *
 * 硬件：直流减速电机自带霍尔编码器，AB相正交输出
 *       STM32 TIM3 编码器模式 (PA6/PA7)
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "platform.h"

/* ===== 编码器参数（根据实际电机修改） ===== */
#define ENCODER_PPR           390u    /* 每转脉冲数（减速后，含4倍频） */
#define ENCODER_CONTROL_PERIOD_MS 20u /* 速度计算周期（毫秒），与控制周期一致 */
#define WHEEL_DIAMETER_MM     65u     /* 车轮直径（毫米），用于换算线速度 */

/* ===== 初始化 ===== */
/**
 * @brief 初始化编码器（定时器编码器模式）
 */
void Encoder_Init(void);

/* ===== 计数读取 ===== */
/**
 * @brief 获取编码器累计计数（带符号，正转+，反转-）
 * @return 累计计数值（int16，溢出会回绕）
 * @note  对应 AUTOSAR Gpt_GetTimerValue()
 */
sint16 Encoder_GetCount(void);

/**
 * @brief 获取自上次调用后的计数增量，并清零内部基准
 * @return 计数增量（带符号）
 * @note  每 20ms 调用一次，用于速度计算
 *        这是速度闭环的关键输入
 */
sint16 Encoder_GetDelta(void);

/* ===== 速度计算 ===== */
/**
 * @brief 获取当前转速（RPM，转/分钟）
 * @return 电机转速 RPM（带符号）
 * @note  基于最近一次 GetDelta() 的结果计算
 */
float32 Encoder_GetSpeedRPM(void);

/**
 * @brief 获取当前线速度（cm/s）
 * @return 车轮线速度 cm/s（带符号）
 * @note  基于车轮直径换算，用于 AEB 距离估算
 */
float32 Encoder_GetSpeedCmPerS(void);

/* ===== 清零 ===== */
/**
 * @brief 清零编码器计数
 */
void Encoder_Reset(void);

/* ===== 自检 ===== */
/**
 * @brief 编码器自检（检查定时器是否正常工作）
 * @return TRUE=正常, FALSE=故障
 */
boolean Encoder_SelfTest(void);

#endif /* ENCODER_H */
