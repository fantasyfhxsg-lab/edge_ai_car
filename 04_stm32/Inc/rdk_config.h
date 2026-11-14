/**
 * rdk_config.h - RDK 行车链路参数集中配置（**唯一配置源**）
 *
 * ═══════════ 这个头文件是"行车链路调参面板" ═══════════
 * 上层（应用层 RDK 状态机）不写死任何数字，全部读 g_rdk_cfg 的字段：
 *   时序（判活超时/0x101 恒发周期/60s 锁定/握手应答帧数）
 *   速度与方向（巡航 PWM、预警 PWM、前进方向）
 *   制动方式（短路制动 / 纯滑行）与 LED 代替蜂鸣
 * 改参数只动 Src/rdk_config.c 一处 —— 不需要碰 main.c（.clinerules §4：
 * 业务层不碰寄存器，配置层不碰执行逻辑）。
 *
 * ⚠ 单位约定：PWM 用 Motor_SetSpeed() 的量程（±1000）；时间 ms。
 * ⚠ hb_timeout_ms 是**安全参数**（断 CAN 多久必须停），单一真源是
 *   Inc/can_protocol.h 的 RDK_HB_TIMEOUT_MS_DEF（那里带 [200,500] 范围护栏）。
 */
#ifndef RDK_CONFIG_H
#define RDK_CONFIG_H

#include "platform.h"        /* AUTOSAR 风格类型（uint8/sint16/boolean） */
#include "can_protocol.h"   /* CAN_PROTOCOL_HEARTBEAT_V1：本配置表只在 heartbeat_v1 分支有意义 */

/* ⚠ 仅在 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译（brake_v2 分支不含 RDK 行车链路）。 */
#if CAN_PROTOCOL_HEARTBEAT_V1

typedef struct {
    /* ---------- 时序 / 次数 ---------- */
    uint16  hb_timeout_ms;      /* 300  ：判活阈值 —— **只看 0x100**；静默超过此时间即
                                 *        本地刹停进 HOLD（0x102 不参与，验收第 3 条） */
    uint16  hs_period_ms;       /* 200  ：0x101 恒发周期（= 握手重发周期，无次数上限） */
    uint32  lock_timeout_ms;    /* 60000：STATIC 未握手 / HOLD 未恢复 超过此时间 → LOCK */
    uint8   hs_ack_need;        /* 2    ：累计 2 帧有效 0x102 应答 → CRUISE */

    /* ---------- 速度 / 方向 ---------- */
    sint16  cruise_pwm;         /* 300 ：巡航 PWM（低速直行），量程 ±1000 */
    sint16  warn_pwm;           /* 150 ：收到 0x100 cmd=02（预警）后减速到的 PWM */
    sint16  cruise_dir;         /* +1  ：前进方向（+1 前进 / -1 后退；装反了改这里） */

    /* ---------- 制动方式 ---------- */
    boolean stop_short_brake;   /* TRUE：0x03/0x100 超时/上电 → PWM=0 + 短路制动（最强制动）
                                 * FALSE：仅 PWM=0（滑行停车，用于对比滑行距离） */
    boolean buzz_use_led;       /* TRUE：0x02 的"蜂鸣"用板载 LED0 慢闪代替（板卡无蜂鸣器） */
} Rdk_Config_t;

/**
 * @brief  RDK 行车链路的**唯一配置实例**（只读常量表，定义在 Src/rdk_config.c）
 * @note   应用层（main.c 的 RDK 状态机）只读它的字段，不写死任何数字；
 *         调参只改 Src/rdk_config.c 一处，不动执行逻辑。
 *         字段与单位：
 *           hb_timeout_ms   判活阈值（ms，只看 0x100；单一真源是
 *                           can_protocol.h 的 RDK_HB_TIMEOUT_MS_DEF，带范围护栏）
 *           hs_period_ms    0x101 恒发/重握手周期（ms，默认 200）
 *           lock_timeout_ms STATIC 未握手 / HOLD 未恢复 → LOCK 的窗口（ms，默认 60000）
 *           hs_ack_need     握手所需有效 0x102 应答帧数（默认 2）
 *           cruise_pwm      巡航 PWM，量程 ±1000（Motor_SetSpeed 的域）
 *           warn_pwm        0x02 预警后减速到的 PWM，量程 ±1000
 *           cruise_dir      前进方向系数（+1 前进 / -1 后退；装反改这里不改接线）
 *           stop_short_brake TRUE = 刹停用短路制动，FALSE = 仅 PWM=0 滑行
 *           buzz_use_led    TRUE = 0x02 的蜂鸣用 LED0 慢闪代替
 *         ⚠ 是 const 对象：运行期不可写，只能在 Src/rdk_config.c 的初始化式里改。
 *         ⚠ 仅在 CAN_PROTOCOL_HEARTBEAT_V1=1 时存在（brake_v2 分支没有它）。
 */
extern const Rdk_Config_t g_rdk_cfg;   /* 唯一实例，定义在 Src/rdk_config.c */

#endif /* CAN_PROTOCOL_HEARTBEAT_V1 */

#endif /* RDK_CONFIG_H */
