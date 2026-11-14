/**
 * encoder.h - 编码器抽象层（Encoder Abstraction）
 *
 * 硬件：JGB37-520 霍尔编码器（11 PPR 电机轴 × 减速比 19 = 209 PPR 输出轴）
 *   左轮: TIM4 编码器模式，PD12(TIM4_CH1)/PD13(TIM4_CH2)
 *   右轮: TIM8 编码器模式，PC6 (TIM8_CH1)/PC7 (TIM8_CH2)
 *   正交 4 倍频：输出轴每转计数值 = 209 × 4 = 836
 *
 * 接口：默认 API 操作左轮，新增 _Right 后缀操作右轮。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "platform.h"

/* ===== 编码器参数（JGB37-520） ===== */
#define ENCODER_PPR           836u   /* = 11 × 19 × 4（输出轴每转计数值） */
#define ENCODER_CONTROL_PERIOD_MS 20u   /* 测速采样窗口 20ms：必须与 Task_20ms 周期一致，否则 RPM 换算会整体偏 */
#define WHEEL_DIAMETER_MM     65u       /* 轮径 65mm：线速度 cm/s 换算用（π×D/10 = 20.42cm 周长） */

/* ===== 初始化 ===== */

/**
 * @brief  初始化两路编码器接口（TIM4 左轮 / TIM8 右轮，正交 4 倍频自由计数）
 * @param  无
 * @return 无
 * @note   PSC=0、ARR=0xFFFF（不设时基，计数不随 APB 频率漂移）；
 *         同时把"上一拍计数"基准清零 —— 否则第一次 GetDelta 会算出巨大的假增量。
 */
void Encoder_Init(void);

/* ===== 左轮（默认 API） ===== */

/**
 * @brief  读左轮编码器累计计数值
 * @param  无
 * @return -32768 ~ +32767（TIM4->CNT 截成有符号 16 位，超出自然回绕）
 * @note   反映"上电/Reset 以来的相对位置"，不是绝对角度；正负号表示转向。
 */
sint16 Encoder_GetCount(void);

/**
 * @brief  取左轮自上次调用以来的计数增量
 * @param  无
 * @return 脉冲增量：正 = 前进方向，负 = 后退方向（两次采样差远小于 32767 时，
 *         16 位回绕不影响结果）
 * @note   ⚠ **有状态接口**：内部会更新基准，所以一个 20ms 采样窗口**只能调用一次**。
 *         同一窗口内若先调了 Encoder_GetSpeedRPM()，再调本函数会拿到 0（增量已被消费）。
 */
sint16 Encoder_GetDelta(void);

/**
 * @brief  取左轮转速（基于本窗口增量换算）
 * @param  无
 * @return 转速，单位 rpm（每分钟转数）= delta × 3000 / 836；
 *         3000 = 60s/20ms 的换算系数，836 = 输出轴每转计数（11PPR×19×4）
 * @note   ⚠ 同样**有状态**（内部消费一次增量）；与 GetDelta/GetSpeedCmPerS 互斥，
 *         一个窗口里只能选一个"消费者"。
 */
float32 Encoder_GetSpeedRPM(void);

/**
 * @brief  取左轮线速度（基于本窗口增量换算）
 * @param  无
 * @return 线速度，单位 cm/s = (rpm/60) × 轮周长(π×65mm/10 ≈ 20.42cm)
 * @note   ⚠ **有状态**，与 GetDelta/GetSpeedRPM 竞争同一个增量；
 *         0x125 车速上报就取两轮 cm/s 的平均值再 ×0.036 换成 km/h。
 */
float32 Encoder_GetSpeedCmPerS(void);

/**
 * @brief  清零编码器：硬件计数器 + 软件增量基准一起归零（左右轮同时）
 * @param  无
 * @return 无
 * @note   必须在同一个采样窗口的边界处调用，并与测速调用配对，避免出现假增量。
 */
void Encoder_Reset(void);

/**
 * @brief  自检：两路编码器定时器是否都已启动
 * @param  无
 * @return TRUE = TIM4/TIM8 的 CEN 都为 1（计数在跑）；FALSE = 有一路没启动
 * @note   只检查硬件使能状态，不断言轮子是否真的转（转动测试要人工）。
 */
boolean Encoder_SelfTest(void);

/* ===== 右轮 ===== */

/**
 * @brief  读右轮编码器累计计数值
 * @param  无
 * @return -32768 ~ +32767（TIM8->CNT 截成有符号 16 位）
 * @note   与左轮独立计数、独立基准，可用于比对两轮是否同步（差异告警用）。
 */
sint16  Encoder_GetCountRight(void);

/**
 * @brief  取右轮自上次调用以来的计数增量
 * @param  无
 * @return 脉冲增量（正 = 前进方向）
 * @note   ⚠ **有状态**：一个 20ms 窗口只能调用一次（同左轮规则）。
 */
sint16  Encoder_GetDeltaRight(void);

/**
 * @brief  取右轮转速
 * @param  无
 * @return 转速，单位 rpm（换算系数与左轮完全一致）
 * @note   ⚠ **有状态**，与右轮其它测速接口竞争同一增量。
 */
float32 Encoder_GetSpeedRPMRight(void);

/**
 * @brief  取右轮线速度
 * @param  无
 * @return 线速度，单位 cm/s
 * @note   ⚠ **有状态**；右轮 cm/s（0x125 两轮平均用的第二路）。
 */
float32 Encoder_GetSpeedCmPerSRight(void);

#endif /* ENCODER_H */
