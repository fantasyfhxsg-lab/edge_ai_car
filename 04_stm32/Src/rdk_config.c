/**
 * rdk_config.c - RDK 行车链路参数定义（**改参数只动这个文件**）
 *
 * 分层位置：配置层（AUTOSAR CP 里属应用层的标定数据）。本文件只有常量表，
 * 不做任何寄存器/外设操作，也不含执行逻辑 —— 所以"业务不碰寄存器"的约束天然成立。
 *
 * 取值依据（每个数都给出出处，避免"魔数"）：
 *   hb_timeout_ms   = 300  ：用户规格（判活：**只看 0x100**；静默 300ms → 本地刹停），
 *                            同时是验收"断 CAN 300ms 内刹停"的判据。
 *                            单一真源 = Inc/can_protocol.h 的 RDK_HB_TIMEOUT_MS_DEF。
 *   hs_period_ms    = 200  ：用户规格（0x101 恒发周期，无次数上限）。
 *   lock_timeout_ms = 60000：用户规格（60s 未握手 / 未恢复 → LOCK，需复位）。
 *   hs_ack_need     = 2    ：用户规格（累计收 2 帧有效 0x102 才进 CRUISE）。
 *   cruise_pwm      = 300  ：低速直行标定（约 30% 占空）、warn_pwm = 150（预警减速一半）。
 *   cruise_dir      = +1   ：前进；若整车装反（上电即后退）改 -1，不动接线。
 *   stop_short_brake= TRUE ：0x03 / 0x100 超时 / 上电都用短路制动（"PWM=0 刹停"）。
 *   buzz_use_led    = TRUE ：板卡无蜂鸣器 → 0x02 用 LED0 慢闪代替。
 */
#include "rdk_config.h"
#include "can_protocol.h" /* RDK_*_DEF：协议级默认值（契约在本层） */

/* ⚠ 仅在 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译（brake_v2 分支不含 RDK 行车链路）。 */
#if CAN_PROTOCOL_HEARTBEAT_V1

/* ===== 编译期护栏：安全参数不允许被放宽（协议级默认值见 can_protocol.h）=====
 * 谁把恒发周期/锁定窗口/握手帧数改离协议默认值，这里直接编译失败。
 * 注：判活阈值（RDK_HB_TIMEOUT_MS_DEF）的护栏已随真源移到 can_protocol.h
 *     （范围 [200,500]）—— 判活阈值在那里是"单一配置点 + 同文件护栏"。 */
/* 0x101 必须 200ms 一帧：改快/改慢都会让板端握手时序与抓包判据对不上 */
_Static_assert(RDK_HS_PERIOD_MS_DEF == 200u,     "can_protocol: 0x101 恒发周期被改动，需先确认");
/* 60s 是"失联多久后彻底锁死等复位"的安全窗口，改动等于改规格 */
_Static_assert(RDK_LOCK_TIMEOUT_MS_DEF == 60000u,"can_protocol: 60s 锁定窗口被改动，需先确认");
/* 需要累计 2 帧有效 0x102 才算握手成功：单帧可能是噪声/重放，2 帧才能确认对端真的在应答 */
_Static_assert(RDK_HS_ACK_NEED_DEF == 2u,        "can_protocol: 握手应答帧数被改动，需先确认");

/* ===== 唯一实例 ===== */
const Rdk_Config_t g_rdk_cfg = {
    /* 时序 / 次数 */
    .hb_timeout_ms   = RDK_HB_TIMEOUT_MS_DEF,    /* 300   ：只看 0x100 */
    .hs_period_ms    = RDK_HS_PERIOD_MS_DEF,     /* 200   ：0x101 恒发周期 */
    .lock_timeout_ms = RDK_LOCK_TIMEOUT_MS_DEF,  /* 60000 ：60s → LOCK */
    .hs_ack_need     = RDK_HS_ACK_NEED_DEF,      /* 2     ：累计 2 帧有效 0x102 → CRUISE */

    /* 速度 / 方向 */
    .cruise_pwm      = 300,      /* 巡航 PWM：300/1000 = 30% 占空（低速直行标定值，实测不会冲出去） */
    .warn_pwm        = 150,      /* 预警减速 PWM：150/1000 = 15% 占空，约为巡航速度的一半 */
    .cruise_dir      = 1,        /* +1 = 前进；若整车装反（上电即后退）只改这里为 −1，不动接线 */

    /* 制动方式 */
    .stop_short_brake = TRUE,    /* 0x03 / 0x100 超时 / 上电都用短路制动（"PWM=0 刹停"） */
    .buzz_use_led     = TRUE,    /* 板卡无蜂鸣器：0x02 用 LED0 慢闪代替 */
};

#endif /* CAN_PROTOCOL_HEARTBEAT_V1 */
