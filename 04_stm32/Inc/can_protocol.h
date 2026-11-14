/**
 * CAN 协议定义（AI板 ↔ STM32）
 *
 * ⚠ 本文件是 AI 板与 STM32 的"线上接口契约"，与板端 clinerules 一一对应，
 *   禁止单边改动（见 .clinerules §5）。改 ID/字节布局/周期前必须先出方案给用户确认。
 *
 * 通用：标准帧（11 位 ID）、500kbps、数据段固定 8 字节、未用字节填 0、
 *       多字节字段小端（uint16 低字节在前）、只用数据帧（RTR=0）。
 *
 * 协议之一（AEB 域，§5）：
 *   0x123 板端→STM32  制动指令：byte0 = 0正常/1预警/2部分制动/3紧急制动；byte1-7 填 0
 *   0x124 板端→STM32  障碍物距离：byte0-1 = uint16 小端 cm；0xFFFF = 无有效目标
 *   0x125 STM32→板端  当前车速：byte0-1 = uint16 小端 km/h；0xFFFF = 车速无效
 *
 * 协议之二（RDK 行车链路域，见本文件「heartbeat_v1」段；默认协议）：
 *   0x100 RDK→STM32  心跳/命令 @50ms：byte0=命令 0x00 停车/0x01 巡航/0x02 预警/0x03 制动
 *                     byte1=距离cm byte2=置信度 byte3=计数(0~255 递增)；byte4-7 填 0
 *   0x101 STM32→RDK  握手请求 + STM 心跳 @200ms：[0xAA,seq,...]（恒发，无次数上限）
 *   0x102 RDK→STM32  应答 [0x55,0x01,seq,...] / 心跳 [0x55,0x02,seq,...]，DLC=8
 *
 * 两个域编译期二选一（见下方协议版本宏），ID 不重叠（0x123/0x124/0x125 仅 brake_v2）。
 * ⚠ 旧脚手架协议（0x100~0x103 的旧语义 + CTRL_* / STATUS_* / FAULT_* +
 *   CanControlFrame / CanStatusFrame）**已作废（deprecated）**，全部躺在文件末尾的
 *   `#if 0` 里，每个符号都标了 @deprecated 与"现由谁取代"；其中 0x101/0x102 现由
 *   「heartbeat_v1」重新定义（语义完全不同），旧的 Can_SendStatus()/Can_SendHeartbeat()
 *   同样已作废（见 Inc/can_driver.h、Src/can_driver.c 的同名 #if 0 段）。
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include <stdint.h>

/* ================================================================
 *  协议版本编译期宏二选一（.clinerules §5，默认 heartbeat_v1）
 *  ------------------------------------------------------------------
 *  构建期切换（见 Makefile，不改源码）：
 *      make                     -> heartbeat_v1（默认）
 *      make PROTOCOL=brake_v2   -> brake_v2
 *  等价于链接期注入：
 *      -DCAN_PROTOCOL_HEARTBEAT_V1=1 -DCAN_PROTOCOL_BRAKE_V2=0
 *      -DCAN_PROTOCOL_HEARTBEAT_V1=0 -DCAN_PROTOCOL_BRAKE_V2=1
 *  两个宏之和必须恰为 1（防"两个都开 / 都不开"），否则编译期 #error。
 *  ⚠ 选中的版本必须与板端 can.protocol 一致（.clinerules §5/§6），不一致先不联调。
 * ================================================================ */
#ifndef CAN_PROTOCOL_HEARTBEAT_V1
/** @brief 协议版本选择宏：heartbeat_v1（默认 1）—— 0x100 命令/0x101 握手请求/0x102 应答
 *  @note  ① 由 Makefile 的 `PROTOCOL=` 选项经 -D 注入，源码里不硬编码；
 *         ② 与 CAN_PROTOCOL_BRAKE_V2 之和必须恰为 1，否则下方 #error 直接编译失败
 *            （防"两个都开 / 都不开"）；③ 必须与板端 can.protocol 一致。 */
#define CAN_PROTOCOL_HEARTBEAT_V1   1
#endif
#ifndef CAN_PROTOCOL_BRAKE_V2
/** @brief 协议版本选择宏：brake_v2（legacy，默认 0）—— 0x123 制动/0x124 距离/0x125 车速
 *  @note  默认关闭；`make PROTOCOL=brake_v2` 置 1。改协议必须同步板端（§5/§6），
 *         单边改动只会表现为"收不到帧"，不会有编译期报错。 */
#define CAN_PROTOCOL_BRAKE_V2       0
#endif
#if ((CAN_PROTOCOL_HEARTBEAT_V1 + CAN_PROTOCOL_BRAKE_V2) != 1)
#error "CAN 协议必须二选一：CAN_PROTOCOL_HEARTBEAT_V1 与 CAN_PROTOCOL_BRAKE_V2 恰有一个为 1（默认 heartbeat_v1）"
#endif

/** @brief 数据段固定长度 8 字节（DLC=8，§5 通用约定，两协议共用）
 *  @note  ① 驱动层解析按此**整帧**拒绝短帧/长帧（不做"部分字节有效"的容忍）；
 *         ② 真机首跑教训：一帧 DLC=1 的畸形 0x123 曾被解析层放行并改写制动等级，
 *            与 §5 和应用层日志 "(DLC!=8 ... ignored)" 矛盾 —— 故在解析层收紧为严格相等；
 *         ③ 发送侧同样固定 8（Can_Send 的 len 实参），未用字节填 0。 */
#define CAN_DATA_LEN           8u

/* ================================================================
 *  AEB 协议（brake_v2，0x123 / 0x124 / 0x125；.clinerules §5）
 *  仅当 CAN_PROTOCOL_BRAKE_V2=1 时编译
 * ================================================================ */
#if CAN_PROTOCOL_BRAKE_V2

/* ===== CAN ID（标准帧 11 位） ===== */
/** @brief 0x123 制动指令（板端→STM32）
 *  @note  byte0 = 制动等级 BRAKE_LEVEL_*；byte1~7 必须填 0；DLC 必须 =8（否则整帧拒绝）。
 *         解析在 can_driver.c 的 Can_ParseBrakeCmd()，执行在 main.c 应用层 Aeb_ApplyBrakeLevel()。 */
#define CAN_ID_BRAKE_CMD      0x123u
/** @brief 0x124 障碍物距离（板端→STM32）
 *  @note  byte0-1 = uint16 小端 cm；0xFFFF(AEB_DIST_INVALID) = 无有效目标。
 *         本端**只记录+打印**（联调观察用），不直接驱动动作 —— 制动只由 0x123 的等级决定。 */
#define CAN_ID_OBSTACLE_DIST  0x124u
/** @brief 0x125 当前车速上报（STM32→板端）
 *  @note  byte0-1 = uint16 小端 km/h（编码器 cm/s × 0.036）；0xFFFF(AEB_SPEED_INVALID) = 车速无效。
 *         由 Task_20ms 用 20ms 测速窗口的发布值周期发送，必须 < 板端 recv_timeout(100ms)。 */
#define CAN_ID_VEHICLE_SPEED  0x125u

/* ===== 0x123 byte0：制动等级（4 档） ===== */
/** @brief 等级 0 正常：解除制动 —— **唯一能解除锁存**的等级
 *  @note  只有收到 0x00 才解除；0x123 超时不会自动解除（刹车优先，见 AEB_BRAKE_TIMEOUT_MS）。 */
#define BRAKE_LEVEL_NONE       0u
/** @brief 等级 1 预警：LED 闪烁 + 日志，**不制动**、也不改锁存状态
 *  @note  解析层用 `data[0] > BRAKE_LEVEL_EMERGENCY → 整帧拒绝` 做合法区间上界，
 *         新增等级必须同时改板端（§5 禁止单边改动）。 */
#define BRAKE_LEVEL_WARN       1u
/** @brief 等级 2 部分制动：Task_20ms 逐拍把 PWM 往下限幅减速，**不锁存**短路制动 */
#define BRAKE_LEVEL_PARTIAL    2u
/** @brief 等级 3 紧急制动：立即 Motor_EmergencyBrake() 并锁存，只有等级 0 才能解除
 *  @note  同时是等级合法性的上边界常量（解析层判 `> BRAKE_LEVEL_EMERGENCY` 即非法）。 */
#define BRAKE_LEVEL_EMERGENCY  3u

/* ===== 无效值哨兵（线上值，必须与板端一致） ===== */
/** @brief 0x124 距离无效哨兵 0xFFFF（= 无有效目标）
 *  @note  与板端约定位值一致，禁止改成 0/65534 这类"更省事"的值；应用层据此打印 "dist=none"。 */
#define AEB_DIST_INVALID       0xFFFFu
/** @brief 0x125 车速无效哨兵 0xFFFF（无有效轮速时上报此值，板端按"无数据"处理） */
#define AEB_SPEED_INVALID      0xFFFFu

/* ===== AEB 时序与阈值（应用层使用；改动视为协议改动，需先确认） ===== */
/** @brief 0x125 契约周期 20ms（必须 < 板端 recv_timeout 100ms）
 *  @note  ⚠ 固件内**不引用**本宏：20ms 周期由 Task_20ms 这个时间片本身实现。
 *         本宏只作线上契约常量，供回归脚本对照（tools/regress/02 断言平均周期 15~25ms）。 */
#define AEB_SPEED_PERIOD_MS      20u
/** @brief 0x123 丢失判定阈值 500ms：超时只告警 + **保持当前制动**，不自动解除（刹车优先、可恢复）
 *  @note  恢复的唯一途径是收到等级 0；与 heartbeat_v1 的 300ms（只看 0x100）是两套独立规则。 */
#define AEB_BRAKE_TIMEOUT_MS     500u
/** @brief 左右轮一致性判据的相对阈值 30（单位 %）：|vL-vR| 超过平均速度的 30% 判为打滑/堵转
 *  @note  超阈只置观测告警（Task_100ms 打印），**不影响** 0x125 上报值；低速时用绝对下限兜底。 */
#define AEB_WHEEL_DIFF_PCT       30u
/** @brief 一致性判据绝对下限 13.9 cm/s（= 0.5 km/h ÷ 0.036），防低速时 30% 阈值过小导致误报 */
#define AEB_WHEEL_DIFF_CMPS_MIN  13.9f

/* ===== 回环 AEB 自测时序（⑤；仅自测镜像使用，**不是线上协议**，可自由改动）=====
 * 自测帧走回环（CAN_BTR.LBKM|SILM）自发自收，不出总线、不影响 AEB 联调。 */
/** @brief 自测步进周期 200ms（每 2 个 Task_100ms 拍推进一步）
 *  @note  ⚠ 必须等于 Task_100ms × 2：main.c 有
 *         `#if (TASK_100MS_MS * 2u) != AEB_LB_STEP_MS → #error` 守卫，
 *         只改这里而不同步任务周期会直接编译失败（故意如此，防静默错拍）。 */
#define AEB_LB_STEP_MS         200u
/** @brief 上电后多久开始第一轮自测（500ms，给 CAN/串口初始化留时间） */
#define AEB_LB_FIRST_DELAY_MS  500u
/** @brief 两轮自测之间的间隔 5000ms（一轮 10 步 = 14 拍 ≈ 2.8s，间隔后自动重跑） */
#define AEB_LB_ROUND_MS       5000u
/** @brief 失效保护步的静默时长 600ms（发完 level3 后停止发帧，等本地超时刹停）
 *  @note  ⚠ 必须 > AEB_BRAKE_TIMEOUT_MS(500)：静默不够长则超时判定不触发 → 自测假 PASS。
 *         实际拍数由 main.c 的 AEB_LB_LOST_TICKS = ceil(本值 / AEB_LB_STEP_MS) 算出。 */
#define AEB_LB_LOST_WAIT_MS    600u

#endif /* CAN_PROTOCOL_BRAKE_V2 */

/* ================================================================
 *  heartbeat_v1（0x100 / 0x101 / 0x102；.clinerules §5，默认协议）
 *  仅当 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译；与 brake_v2 编译期二选一。
 *
 *  通用约定与 §5 一致：标准帧（11 位 ID）、500kbps、数据段固定 8 字节
 *  （CAN_DATA_LEN）、未用字节填 0、只用数据帧（RTR=0）。
 *
 *  0x100  板端→STM32  心跳+命令 @50ms（DLC=8）
 *         byte0 = 命令：0x00 停车 / 0x01 巡航 / 0x02 预警 / 0x03 制动
 *         byte1 = 距离 cm；byte2 = 置信度；byte3 = 计数（0~255 循环递增）
 *         byte4~7 = 0
 *  0x101  STM32→板端  握手请求 + STM 心跳 @200ms（DLC=8）
 *         byte0 = 0xAA；byte1 = seq（每帧 +1，仅联调观测）；byte2~7 = 0
 *         恒发：STATIC/CRUISE/HOLD 都发、无次数上限；仅 LOCK 停发。
 *  0x102  板端→STM32  应答 / 心跳（DLC=8）
 *         byte0 = 0x55；byte1 = 0x01 应答 / 0x02 心跳；byte2 = seq（每帧 +1）
 *
 *  ── 本端状态机与判活（**只定义本端怎么用收到的帧**，不改线上 ID/字节/周期）──
 *  判活铁律：**只看 0x100**。0x100 静默 ≥ hb_timeout_ms(300) 即"命令失效" → 立即
 *    本地刹停进 HOLD；0x102 在不在都一样（0x102 仅在 STATIC 阶段参与"累计 2 帧握手"，
 *    CRUISE/HOLD 中只作日志，不参与任何判活/恢复/超时）。
 *  帧有效性：DLC=8 && byte0∈{0x00,0x01,0x02,0x03} && byte3 与上一帧**不同**（去重）。
 *    无效帧一律丢弃且**不刷新判活时间戳**（防"一帧卡住"把车一直开着）。
 *  状态（Src/main.c 应用层）：
 *    STATIC  上电/复位，PWM=0；发 0x101@200ms；**忽略 0x100**；
 *            有效应答（DLC=8 && byte0=0x55 && byte1=0x01 && byte2 递增）累计 2 帧
 *            → CRUISE；60s 未完成握手 → LOCK。
 *    CRUISE  唯一可驱动状态；发 0x101@200ms；按有效 0x100 的 byte0 执行
 *            （00→PWM=0 / 01→巡航软启动 / 02→减速+LED 慢闪 1Hz / 03→刹停+LED 快闪 5Hz）；
 *            0x100 超时 hb_timeout_ms(300) → 本地刹停进 HOLD。
 *    HOLD    PWM=0 保持；发 0x101@200ms 重握手；收到有效 0x100（**唯一恢复条件**）
 *            → CRUISE（按该帧 byte0 执行）；60s 未恢复 → LOCK。
 *    LOCK    PWM=0、LED 常亮、串口打印 LOCK、**停发 0x101**；需复位恢复。
 *  ⚠ 0x101 无次数上限；60s 窗口由"进入 STATIC/HOLD 的时刻 + lock_timeout_ms"实现。
 *  ⚠ 0x00 停车是本协议的"退出/解除"指令（不是 0x01）；收到后 PWM 保持 0。
 * ================================================================ */
#if CAN_PROTOCOL_HEARTBEAT_V1

/* ===== CAN ID（与 0x123/0x124/0x125 不重叠） ===== */
/** @brief 0x100 命令帧（RDK→STM32，周期 50ms，DLC=8）—— 既是命令也是"判活"的唯一依据
 *  @note  byte0=命令 RDK_CMD_*、byte1=距离 cm、byte2=置信度、byte3=计数(0~255 递增)、byte4~7=0；
 *         帧有效性还要求 byte3 与上一帧不同（去重），无效帧丢弃且**不刷新判活时间戳**。 */
#define CAN_ID_RDK_HEARTBEAT  0x100u
/** @brief 0x101 握手请求 + STM 心跳（STM32→RDK，周期 200ms，DLC=8，恒发）
 *  @note  byte0=0xAA(RDK_HS_REQ_TAG)、byte1=seq（每帧 +1，仅联调观测）、byte2~7=0；
 *         STATIC/CRUISE/HOLD 都发、无次数上限，**仅 LOCK 停发**。 */
#define CAN_ID_RDK_HS_REQ     0x101u
/** @brief 0x102 应答 / 心跳（RDK→STM32，DLC=8）
 *  @note  byte0=0x55、byte1=0x01 应答 / 0x02 心跳、byte2=seq（每帧 +1）。
 *         ⚠ 只在 STATIC 阶段用于"累计 2 帧完成握手"；CRUISE/HOLD 中只作日志，
 *         不参与任何判活/恢复/超时（判活铁律：只看 0x100）。 */
#define CAN_ID_RDK_HS_ACK     0x102u

/* ===== 0x100 byte0：命令码（4 档，0x00 与 0x01~0x03 同等合法） ===== */
/** @brief 命令 0x00 停车：PWM=0 保持静止 —— 本协议的"退出/解除"指令（不是 0x01）
 *  @note  收到后保持 PWM=0；也是 HOLD 之外的正常静止态语义。 */
#define RDK_CMD_PARK          0x00u
/** @brief 命令 0x01 巡航：软启动爬升到巡航 PWM（限幅爬升，禁止瞬间全速）
 *  @note  握手成功时也会被置为该语义（见 Rdk_EnterCruise），并非只有收到此命令才巡航。 */
#define RDK_CMD_CRUISE        0x01u
/** @brief 命令 0x02 预警：减速到预警 PWM + LED 慢闪 1Hz（不刹停） */
#define RDK_CMD_ALERT         0x02u
/** @brief 命令 0x03 制动：PWM=0 刹停 + LED 快闪 5Hz
 *  @note  ⚠ 同时是命令码合法性的**上边界**：can_driver.c 用 `data[0] > RDK_CMD_STOP`
 *         判非法命令并整帧拒绝；新增命令码必须同步板端（§5 禁止单边改动）。 */
#define RDK_CMD_STOP          0x03u

/* ===== 0x101 / 0x102 固定字节 ===== */
/** @brief 0x101 byte0 签名 0xAA（握手请求帧标识） */
#define RDK_HS_REQ_TAG        0xAAu
/** @brief 0x102 byte0 签名 0x55（应答/心跳帧标识，不符则整帧判非法） */
#define RDK_HS_ACK_TAG0       0x55u
/** @brief 0x102 byte1：0x01 = 握手应答（参与 STATIC 阶段累计计数） */
#define RDK_HS_ACK_KIND_ACK   0x01u
/** @brief 0x102 byte1：0x02 = 心跳（合法但不计数，只记时刻作日志） */
#define RDK_HS_ACK_KIND_HB    0x02u

/* ===== 时序 / 次数默认值（应用层经 Inc/rdk_config.h 覆盖）=====
 * 这几个是**协议级默认值**：hb_timeout_ms 直接决定"断 CAN 多久必须停"。
 *
 * ★★ 判活阈值 = 单一配置点（single source of truth）★★
 *   全仓只有 RDK_HB_TIMEOUT_MS_DEF 一个真源，改它一处即全链生效：
 *     - Src/rdk_config.c 的 g_rdk_cfg.hb_timeout_ms（引用本宏，自动跟随）
 *     - 开机串口回显 [RDK] link: live only if 0x100 within <值>ms（main.c）
 *     - 回归/验收脚本（tools/regress/04/05 与 tools/hw_verify_rdk_recovery.sh）
 *       都从本文件 grep 本宏取值，不再硬编码。
 *   ⚠ 本值受下面"安全护栏"约束（范围 [200,500]），越界直接编译失败。 */
/** @brief 判活阈值 300ms（安全参数）：0x100 静默 ≥ 此值即"命令失效" → 立即本地刹停进 HOLD
 *  @note  ⚠ 只看 0x100，0x102 不参与；受下方护栏约束 [200,500]，越界编译失败。
 *         改此一处即全链生效（rdk_config.c 引用、开机回显、回归脚本 grep）。 */
#define RDK_HB_TIMEOUT_MS_DEF    300u
/** @brief 0x101 恒发周期 200ms（= 握手重发周期，也是 STM 心跳周期）
 *  @note  rdk_config.c 有 `_Static_assert(== 200u)`：改这里必须同步改断言并确认。
 *         该周期与 0x100 的 50ms 互不相干，不影响判活灵敏度。 */
#define RDK_HS_PERIOD_MS_DEF     200u
/** @brief 锁定窗口 60000ms（60s）：进入 STATIC/HOLD 后未完成握手/未恢复 → LOCK（PWM=0、LED 常亮、停发 0x101）
 *  @note  窗口起点是"进入 STATIC/HOLD 的时刻"，不是上电时刻；LOCK 只能靠复位恢复；
 *         rdk_config.c 有 `_Static_assert(== 60000u)` 守卫。 */
#define RDK_LOCK_TIMEOUT_MS_DEF  60000u
/** @brief 握手应答帧数门槛 2：STATIC 阶段累计 2 帧有效 0x102 应答（0x55/0x01/seq 递增）→ CRUISE
 *  @note  rdk_config.c 有 `_Static_assert(== 2u)` 守卫；改成 1 会让单帧噪声直接放行，
 *         改成 >2 会拉长握手时间（60s 窗口内仍可完成）。 */
#define RDK_HS_ACK_NEED_DEF        2u

/* ===== 判活阈值安全护栏（范围 [200,500]，与真源同处一个文件）=====
 * 判活阈值直接决定"断 CAN 多久必须停"，是**安全参数**：低于 200ms 会因 0x100 抖动误刹，
 * 高于 500ms 等于放松失效保护（车掉线后多跑）。越界即编译失败 ← 必须人工确认后再改。
 * 护栏放在数据源旁边，就是为了"改阈值只在本文件一处（相邻两行）"。
 * ⚠ 不要把它放开成更宽范围：那会失去"越界即拦"的保护（见 README 本轮修订）。 */
_Static_assert(RDK_HB_TIMEOUT_MS_DEF >= 200u,
               "RDK_HB_TIMEOUT_MS_DEF 低于下限 200ms：0x100 抖动误刹风险，需先确认");
_Static_assert(RDK_HB_TIMEOUT_MS_DEF <= 500u,
               "RDK_HB_TIMEOUT_MS_DEF 超过上限 500ms：放松失效保护，需先确认");

#endif /* CAN_PROTOCOL_HEARTBEAT_V1 */

/* ===== 配置参数 =====
 * CAN_BITRATE 被 tools/timing_audit.py 与 can_driver.h 的 CAN_BITRATE_BPS 交叉核对，
 * 两者必须同为 500000，改动前先确认。（两协议共用） */
/** @brief CAN 位率 500000 bps（500kbps，标准帧、DLC≤8，两协议共用）
 *  @note  ① 必须与 Inc/can_driver.h 的 CAN_BITRATE_BPS 一致：tools/timing_audit.py 会交叉核对，
 *            不一致直接 FAIL；
 *         ② ⚠ 固件**不引用**本宏算 BTR：can_driver.c 里 BTR 是硬编码常量
 *            （BRP=3 / TS1=14 / TS2=4 → 21 tq @ 42MHz÷4 = 10.5MHz → 500kbps，采样点 76%）；
 *            改波特率必须同时改那段 BTR 常量，否则实际波特率与本宏不符且编译不报错；
 *         ③ 改波特率必须同步板端与总线另一端，不一致的表现是 Bus-Off / 收不到帧（不是报错）。 */
#define CAN_BITRATE           500000

/* ===== 旧脚手架协议（0x100~0x103）：⚠ 整块已作废（deprecated），仅历史留档 =====
 * ⚠ 本块 `#if 0` 内的所有 ID / 命令字 / 状态字 / 故障码 / 结构体**全部已作废**，
 *   不参与编译、也不属 §5 协议表；每个符号的文档块都标了 @deprecated 与"现由谁取代"。
 *   最需要注意的两处语义冲突（同名 ID、完全不同的含义）：
 *     0x101  本块 = 旧"状态帧"（STM32→AI板）→ 现为 heartbeat_v1 的**握手请求**（STM32→RDK，0xAA+seq）
 *     0x102  本块 = 旧"心跳"（AI板→STM32）  → 现为 heartbeat_v1 的**应答/心跳**（RDK→STM32，0x55+kind+seq）
 *   对应的 Can_SendStatus()/Can_SendHeartbeat() 同样是 #if 0（见 Src/can_driver.c）。
 *   启用本块任一符号前必须先改 §5 协议文档并与板端同步，否则会直接破坏联调协议表。 */
#if 0

/* ===== CAN ID 定义 ===== */
/** @brief [已作废] 0x100 旧控制指令 ID（AI板→STM32，周期 20ms）
 *  @deprecated 现 0x100 在 heartbeat_v1 里是 CAN_ID_RDK_HEARTBEAT（命令+判活帧），
 *              语义/字节布局完全不同；控件指令改由 RDK_CMD_* 表达。 */
#define CAN_ID_CONTROL        0x100
/** @brief [已作废] 0x101 旧状态反馈 ID（STM32→AI板，周期 20ms）
 *  @deprecated ⚠ 同号冲突：现 0x101 是 CAN_ID_RDK_HS_REQ（STM32→RDK 握手请求，0xAA+seq），
 *              方向与语义都变了，据此收发帧会破坏 heartbeat_v1 握手。 */
#define CAN_ID_STATUS         0x101
/** @brief [已作废] 0x102 旧"AI板心跳" ID（AI板→STM32，周期 1s）
 *  @deprecated ⚠ 同号冲突：现 0x102 是 CAN_ID_RDK_HS_ACK（应答 0x55/0x01 或心跳 0x55/0x02，
 *              DLC=8、seq +1），不再表示"AI板 1s 心跳"。 */
#define CAN_ID_HEARTBEAT_AI   0x102
/** @brief [已作废] 0x103 旧"STM32 心跳" ID（STM32→AI板，周期 1s）
 *  @deprecated 现行 heartbeat_v1 **没有 0x103**：STM 心跳合并进 0x101（200ms 恒发）。 */
#define CAN_ID_HEARTBEAT_STM  0x103

/* ===== 控制字（0x100 byte0） ===== */
/** @brief [已作废] 旧控制字"使能"（0x100 byte0 = 0x01）
 *  @deprecated 现 0x100 byte0 是命令码 RDK_CMD_*（0x01 = RDK_CMD_CRUISE 巡航），
 *              不是"使能/失能"二态。 */
#define CTRL_ENABLE           0x01
/** @brief [已作废] 旧控制字"失能"（0x100 byte0 = 0x00）
 *  @deprecated 现 0x00 是 RDK_CMD_PARK（停车，PWM=0），含义为"停车/退出"而非"失能"。 */
#define CTRL_DISABLE          0x00

/* ===== 制动指令（0x100 byte3） ===== */
/** @brief [已作废] 旧制动指令"无制动"（0x100 byte3 = 0x00）
 *  @deprecated heartbeat_v1 的 byte3 是**帧计数**（去重用），不是制动位；
 *              brake_v2 的制动等级在 0x123 byte0（BRAKE_LEVEL_*）。 */
#define BRAKE_NONE            0x00
/** @brief [已作废] 旧制动指令"紧急制动"（0x100 byte3 = 0x01）
 *  @deprecated 现急刹路径：heartbeat_v1 收 RDK_CMD_STOP(0x03)，或 brake_v2 收
 *              BRAKE_LEVEL_EMERGENCY(3) → Motor_EmergencyBrake()。 */
#define BRAKE_EMERGENCY       0x01

/* ===== 状态字（0x101 byte0） ===== */
/** @brief [已作废] 旧状态字"正常"（0x101 byte0 = 0x00）
 *  @deprecated 现行协议表**不定义状态帧**：STM32 的运行状态由 0x101 的 seq 心跳
 *              （byte0=0xAA 恒发）与串口诊断行体现，不再上报状态字。 */
#define STATUS_NORMAL         0x00
/** @brief [已作废] 旧状态字"预警"（0x101 byte0 = 0x01）
 *  @deprecated 预警态现由 heartbeat_v1 的 RDK_CMD_ALERT(0x02) 驱动（减速 + LED 慢闪），
 *              本端不回状态字。 */
#define STATUS_WARN           0x01
/** @brief [已作废] 旧状态字"制动中"（0x101 byte0 = 0x02）
 *  @deprecated 制动结果不上报：执行动作由 RDK_CMD_STOP / BRAKE_LEVEL_* 决定，
 *              观测改看 Task_1000ms 的诊断行——heartbeat_v1 是 `[RDK] st=/cmd=/pwm=`，
 *              brake_v2 是 `[AEB] lvl=/latched=`。 */
#define STATUS_BRAKE          0x02
/** @brief [已作废] 旧状态字"故障"（0x101 byte0 = 0x03）
 *  @deprecated 故障判定现由 Can_IsBusOff()/Can_GetTxErrorCount()/Can_GetRxErrorCount()
 *              在 1s 任务里打印，并靠 Can_RecoverIfStuck() 自愈；线上无故障状态字。 */
#define STATUS_FAULT          0x03

/* ===== 故障码（0x101 byte3） ===== */
/** @brief [已作废] 旧故障码"无故障"（位掩码 0）
 *  @deprecated 整组 FAULT_* 都已作废：现行协议没有故障位字段，见 STATUS_FAULT 的说明。 */
#define FAULT_NONE            0x00
/** @brief [已作废] 旧故障码"AI板心跳丢失"（位 0x01）
 *  @deprecated 失效保护改为**本端判活**：heartbeat_v1 只看 0x100 超时（300ms）→ HOLD；
 *              brake_v2 看 0x123 超时（500ms）→ 保持制动。不再依赖对端上报。 */
#define FAULT_AI_HEARTBEAT    0x01
/** @brief [已作废] 旧故障码"左电机故障"（位 0x02）
 *  @deprecated 电机异常现由编码器读数合理性（wheel_reading_valid）+ 左右一致性判据
 *              （AEB_WHEEL_DIFF_PCT / AEB_WHEEL_DIFF_CMPS_MIN）在本地判，命中打
 *              `[AEB] WHEEL-DIFF warn: vL=… vR=…`；不走线上故障位。 */
#define FAULT_MOTOR_LEFT      0x02
/** @brief [已作废] 旧故障码"右电机故障"（位 0x04）
 *  @deprecated 同上：本地一致性判据观测，不走线上故障位。 */
#define FAULT_MOTOR_RIGHT     0x04
/** @brief [已作废] 旧故障码"低电压"（位 0x08）
 *  @deprecated 本工程未实现电压采样（无 ADC 通道接入电池分压），该位从来不会置位。 */
#define FAULT_LOW_BATTERY     0x08
/** @brief [已作废] 旧故障码"编码器故障"（位 0x10）
 *  @deprecated 编码器健康现由 Encoder_SelfTest() 每 20ms 判定（失败则上报 0xFFFF 无效车速），
 *              不走线上故障位。 */
#define FAULT_ENCODER         0x10

/** @brief [已作废] 旧 0x100 控制帧结构（8 字节）
 *  @note  历史字节布局：byte0 控制字（CTRL_ENABLE/CTRL_DISABLE）、byte1 目标速度(-100~100)、
 *         byte2 转向(-100~100，正=右转)、byte3 制动指令(BRAKE_NONE/BRAKE_EMERGENCY)、byte4~7 保留。
 *  @deprecated 现行 heartbeat_v1 的 0x100 是 8 字节命令帧（byte0 命令 / byte1 距离 /
 *              byte2 置信度 / byte3 计数），用 Can_ParseRdkHeartbeat() 解析成
 *              (cmd, dist_cm, confidence)，**不再有结构体映射**。 */
typedef struct {
    uint8_t  control_word;
    int8_t   target_speed;
    int8_t   steering;
    uint8_t  brake_cmd;
    uint8_t  reserved[4];
} CanControlFrame;

/** @brief [已作废] 旧 0x101 状态帧结构（8 字节）
 *  @note  历史字节布局：byte0 状态字、byte1 当前速度(有符号)、byte2 电压×10、
 *         byte3 故障码(位掩码)、byte4~5 左编码器、byte6~7 右编码器。
 *  @deprecated 0x101 现在是 **STM32→RDK 的握手请求**（byte0=0xAA、byte1=seq），
 *              由 Can_SendRdkHsReq() 组帧；本端不再有"状态帧"这一概念。 */
typedef struct {
    uint8_t  status_word;
    int8_t   current_speed;
    uint8_t  battery_voltage_x10;
    uint8_t  fault_code;
    uint16_t encoder_left;
    uint16_t encoder_right;
} CanStatusFrame;

/* ===== 心跳帧结构（0x102 / 0x103）—— ⚠ 已作废，仅留布局记录 =====
 * 历史布局：byte0 = 心跳计数器(0~255 循环)，byte1~7 保留。
 * 现无对应结构体：0x102 由 Can_ParseRdkHsAck() 解析成 (kind, seq)，0x103 根本不存在。 */

/** @brief [已作废] 旧心跳超时 500ms（对端 1s 心跳丢失判定）
 *  @deprecated 现行判活阈值见 RDK_HB_TIMEOUT_MS_DEF（300ms，**只看 0x100**，带 [200,500] 护栏）；
 *              brake_v2 侧是 AEB_BRAKE_TIMEOUT_MS（500ms，保持制动不自动解除）。 */
#define HEARTBEAT_TIMEOUT_MS  500
/** @brief [已作废] 旧控制周期 20ms（旧 0x100 控制帧的发送周期）
 *  @deprecated 现行周期由任务时间片决定：0x101 = 200ms（RDK_HS_PERIOD_MS_DEF）、
 *              0x100 由板端 50ms 发、0x125 = 20ms（Task_20ms 本身）。 */
#define CONTROL_PERIOD_MS     20

#endif /* 旧脚手架协议（0x100~0x103）：已作废·仅历史留档 */

#endif /* __CAN_PROTOCOL_H */
