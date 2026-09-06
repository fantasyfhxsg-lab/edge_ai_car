/**
 * platform.h - 平台抽象层（Platform Abstraction Layer）
 *
 * 作用：隔离底层硬件库，所有业务模块都 include 这个文件，
 *       而不是直接 include "stm32f4xx_hal.h"。
 *
 * 后期迁移到 AUTOSAR 时，只需要修改这一个文件：
 *   当前：#include "stm32f4xx_hal.h"  (HAL库)
 *   后期：#include "Std_Types.h"       (AUTOSAR标准类型)
 *         #include "Dio.h"             (AUTOSAR MCAL)
 *         #include "Pwm.h"
 *         #include "CanIf.h"
 *
 * 对应 AUTOSAR 层级：ECU抽象层（ECU Abstraction Layer）
 * 核心思想：上层不关心底层用 HAL 还是 MCAL，只关心抽象接口。
 */
#ifndef PLATFORM_H
#define PLATFORM_H

/* ===== 当前使用 STM32 HAL 库 ===== */
#include "main.h"
#include "stm32f4xx_hal.h"

/*
 * ===== 后期迁移 AUTOSAR 时，替换为以下头文件 =====
 * #include "Std_Types.h"    // AUTOSAR标准类型: uint8, uint16, boolean等
 * #include "Dio.h"          // MCAL: GPIO读写
 * #include "Pwm.h"          // MCAL: PWM输出
 * #include "CanIf.h"        // MCAL: CAN接口
 * #include "Gpt.h"          // MCAL: 定时器
 */

/* ===== 通用类型别名（与 AUTOSAR Std_Types.h 对齐） =====
 * 业务代码用这些类型，而不是直接用 uint8_t，
 * 后期迁 AUTOSAR 时这些类型定义直接来自 Std_Types.h，无需修改。
 */
typedef uint8_t  uint8;     /* 无符号8位  */
typedef uint16_t uint16;    /* 无符号16位 */
typedef uint32_t uint32;    /* 无符号32位 */
typedef uint64_t uint64;    /* 无符号64位 */
typedef int8_t   sint8;     /* 有符号8位  */
typedef int16_t  sint16;    /* 有符号16位 */
typedef int32_t  sint32;    /* 有符号32位 */
typedef float    float32;   /* 32位浮点  */
typedef double   float64;   /* 64位浮点  */

/* 布尔类型（AUTOSAR 标准） */
#ifndef TRUE
#define TRUE  1u
#endif
#ifndef FALSE
#define FALSE 0u
#endif
typedef uint8 boolean;

/* ===== 常用宏 ===== */
#define UNUSED(x)  ((void)(x))   /* 标记未使用变量，消除警告 */

/* ===== 系统时钟频率（根据实际硬件修改） ===== */
#define SYSTEM_CLOCK_HZ  168000000u   /* STM32F407 168MHz */

#endif /* PLATFORM_H */
