/**
 * can_driver.h - CAN驱动抽象层（CAN Driver Abstraction）
 *
 * 硬件：CAN1 @ PA11(RX)/PA12(TX)，500kbps
 *   最小系统板没有板载 CAN 收发器，必须外接 TJA1050/SN65HVD230 模块：
 *     STM32 PA12(TX) -> 收发器 TXD
 *     STM32 PA11(RX) <- 收发器 RXD
 *     收发器 CANH/CANL -> 总线；收发器 VCC 5V/3.3V 与系统共地
 *
 * 接口风格与脚手架一致：Can_Send() / Can_RegisterRxCallback() /
 * Can_SendStatus() / Can_SendHeartbeat()。
 * 接收采用轮询（Can_ProcessRx 在 10ms 任务里调用），回调签名不变。
 */
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "platform.h"
#include "can_protocol.h"

#define CAN_BITRATE_BPS    500000u   /* 总线波特率 500kbps（与板端 / tools/timing_audit.py 交叉核对） */
#define CAN_MAX_DATA_LEN   8u        /* 经典 CAN 数据段上限 8 字节（本项目固定用满 8 字节） */

/* ===== 接收回调函数类型 =====
 * 参数：can_id = 标准帧 ID（11 位）；data = 数据段（固定 8 字节装，实际有效 len 字节）；
 *       len = DLC（0~8，已夹紧）。
 * ⚠ 回调在 Can_ProcessRx() 里**同步**执行（不在中断里），必须保持短小（规则 §3）；
 *   data 指向的是 Can_ProcessRx 的栈上缓冲，回调返回后即失效 —— 需要留存请自行拷贝。 */
typedef void (*Can_RxCallback_t)(uint16 can_id, const uint8* data, uint8 len);

/* ===== 初始化 ===== */

/**
 * @brief  初始化 CAN1 为**正常**模式（真实驱动 CANH/CANL，接 TJA1050 与板端联调）
 * @param  无
 * @return TRUE = 已退出初始化模式、成功加入总线，可以收发；
 *         FALSE = 有界等待内 INAK 未清 0（总线不合格：无对端/缺 120Ω/未共地）
 * @note   内容：开 GPIOA+CAN1 时钟 → PA11/PA12 配 AF9 → 位时序 500kbps
 *         （BRP=4、TS1=15tq、TS2=5tq、SJW=1tq）→ ABOM+TXFP → 过滤器 0 全收进 FIFO0。
 *         返回 FALSE 时不影响后续重试（Can_RecoverIfStuck 每秒兜底一次）。
 */
boolean Can_Init(void);

/**
 * @brief  初始化 CAN1 为**静默环回**模式（上电自测用，规则 §6.1 步骤①）
 * @param  无
 * @return TRUE = 初始化成功；FALSE = 未退出初始化模式（回环模式下基本不会发生）
 * @note   BTR 额外置 LBKM|SILM：发送内部接回接收、CANTX 保持隐性不打扰总线，
 *         因此**不需要**收发器/对端/120Ω 终端，可单独验证驱动与过滤器。
 *         位时序数值与正常模式完全一致，切回来不会引入新的波特率差异。
 */
boolean Can_InitLoopback(void);

/* ===== 发送 ===== */

/**
 * @brief  把一帧标准数据帧装入空闲发送邮箱并请求发送（不等待对端 ACK）
 * @param  can_id 11 位标准帧 ID，取值 0x000~0x7FF（IDE=0、RTR=0 由本函数固定）
 * @param  data   数据段首地址，不可为 NULL，至少可读 len 字节
 * @param  len    DLC，0~8；传 >8 会被夹到 8；len 之后的字节自动填 0（规则 §5）
 * @return TRUE  = 已成功装入邮箱（只代表"已发出请求"，对端是否 ACK 要看 ESR/BOFF）；
 *         FALSE = 三个邮箱在 100000 次有界等待内始终满载（调用方下个周期重发即可）
 * @note   全程有界等待，不会卡死 10ms 任务（规则 §3）。写入顺序为
 *         TIR(不含 TXRQ) → TDTR → TDLR → TDHR → 最后才置 TXRQ，
 *         避免奇数 DLC 时"带着邮箱旧数据"提前发帧。
 */
boolean Can_Send(uint16 can_id, const uint8* data, uint8 len);

/* ===== 接收回调注册 ===== */

/**
 * @brief  注册/注销接收回调（真正分发发生在本层轮询接收里，不在中断里）
 * @param  callback 处理函数指针；传 NULL = 注销（之后收到的帧直接丢弃）
 * @return 无
 * @note   只保存指针，不做其它事；必须在 Can_ProcessRx() 开始被调用之前注册。
 */
void Can_RegisterRxCallback(Can_RxCallback_t callback);

/* ===== 轮询接收（在 10ms 任务中调用） ===== */

/**
 * @brief  排空 FIFO0 并把每帧交给已注册的回调（非阻塞、有界）
 * @param  无
 * @return 无
 * @note   必须由 Task_10ms 周期调用：CAN 没有用中断，不调就永远收不到帧。
 *         一次最多处理 8 帧（FIFO 深度仅 3，上限是防异常死循环）；
 *         每帧读完立即写 RFOM0 释放，否则会反复读到同一帧；
 *         顺带观测并清除 FOVR0 溢出标志（累计到 Can_GetRxOverrunCount()）。
 */
void Can_ProcessRx(void);

/* ===== 状态查询 ===== */

/**
 * @brief  查询节点是否已进入 Bus-Off
 * @param  无
 * @return TRUE = BOFF=1（发送错误计数 TEC>255，节点已自动脱离总线）；FALSE = 正常
 * @note   Bus-Off 必须有恢复逻辑（规则 §6.6）：硬件 ABOM 先自恢复，本层的
 *         Can_RecoverIfStuck() 兜底"卡在初始化态"的情况。
 */
boolean Can_IsBusOff(void);

/**
 * @brief  读发送错误计数 TEC
 * @param  无
 * @return 0~255（ESR[23:16]）；>127 说明总线质量差，达 255 即 Bus-Off
 * @note   联调观测用：数值持续上升基本等于"终端电阻/共地/波特率"有问题。
 */
uint8 Can_GetTxErrorCount(void);

/**
 * @brief  读接收错误计数 REC
 * @param  无
 * @return 0~255（ESR[31:24]）；>127 表示接收侧错误累积（对端时序/干扰）
 * @note   与 TEC 分开看：只有 TEC 会触发 Bus-Off。
 */
uint8 Can_GetRxErrorCount(void);

/**
 * @brief  读 FIFO0 溢出累计次数
 * @param  无
 * @return 上电以来的溢出次数（0 = 没丢过帧）
 * @note   >0 说明 10ms 轮询没跟上（FIFO 深度只有 3）：帧是真丢了，
 *         不是"没收到"。排查方向：任务里是否有阻塞（串口打印过长等）。
 */
uint16 Can_GetRxOverrunCount(void);

/**
 * @brief  读自愈成功累计次数
 * @param  无
 * @return 上电以来自愈成功次数（0 = 从未卡过；>0 = 总线曾不合格、后被拉回）
 * @note   与 Can_RecoverIfStuck() 配对使用，便于用串口/SWD 判断"假死"是否发生过。
 */
uint16 Can_GetRecoverCount(void);

/* 自愈（§6.4 Bus-Off 必须有恢复逻辑）：检测到 CAN 被硬件停在初始化态
 * （MSR.INAK=1 而 MCR.INRQ=0，典型成因 = 无对端/总线不空闲导致 bus-off 后 ABOM 出不来）
 * 时，在驱动层内部做一次有界 INRQ 往返把它拉回总线。调度层每秒调用一次即可。 */

/**
 * @brief  节点卡在初始化态时做一次自愈（有界 INRQ 往返 + 重装过滤器）
 * @param  无
 * @return TRUE  = 本次执行了自愈且已回到总线（自愈计数 +1）；
 *         FALSE = 没卡住（INAK=0）/ 是软件自己正在初始化（INRQ=1）/ 本次自愈失败
 * @note   判据必须是 INAK=1 **且** INRQ=0（后者排除"软件主动初始化"误判）。
 *         建议在 Task_1000ms 里调用；失败不要紧，下个周期会再试（总线修好后 1s 内自动回归）。
 *         真机根因记录见 Src/can_driver.c 本函数上方的实测数据。
 */
boolean Can_RecoverIfStuck(void);

/* ===== AEB 协议帧（brake_v2，.clinerules §5）：解析 / 组装（纯字节操作，不碰寄存器） =====
 *   0x123 制动指令 → Can_ParseBrakeCmd()：byte0 = 等级 0~3，>3 视为非法帧
 *   0x124 障碍物距离 → Can_ParseObstacleDist()：byte0-1 = uint16 小端 cm（0xFFFF=无效）
 *   0x125 车速上报 ← Can_SendVehicleSpeed()：byte0-1 = uint16 小端 km/h（0xFFFF=无效）
 * ⚠ DLC 契约（§5 通用：数据段固定 8 字节）：DLC≠8 时**整帧拒绝**（len 必须 == CAN_DATA_LEN），
 *   不做"部分字节有效"的容忍 —— 真机首跑曾有一帧 DLC=1 的畸形 0x123 被放行并改写制动等级。
 *   业务侧只传"等级 / 距离 / km/h"语义值，字节序与 DLC 由本层负责。
 * ⚠ 仅当 CAN_PROTOCOL_BRAKE_V2=1 时编译。 */
#if CAN_PROTOCOL_BRAKE_V2

/**
 * @brief  解析 0x123 制动指令帧 → 制动等级（纯字节操作，不碰寄存器）
 * @param  data   收到的数据段指针，不可为 NULL（本函数按 8 字节读取）
 * @param  len    帧 DLC，**必须 == 8**，否则整帧拒绝（§5 通用契约）
 * @param  level  输出：制动等级指针，不可为 NULL；
 *                成功时写入 0 正常 / 1 预警 / 2 部分制动 / 3 紧急制动
 * @return TRUE = 帧合法且已写出 level；FALSE = 空指针 / DLC≠8 / byte0>3（此时**不写** level）
 * @note   byte1-7 保留（本函数忽略）。真机首跑曾有一帧 DLC=1 的畸形 0x123 绕过解析
 *         改写了制动等级，所以这里不做任何"部分字节有效"的容忍。
 */
boolean Can_ParseBrakeCmd(const uint8* data, uint8 len, uint8* level);

/**
 * @brief  解析 0x124 障碍物距离帧 → 距离（纯字节操作，不碰寄存器）
 * @param  data    收到的数据段指针，不可为 NULL
 * @param  len     帧 DLC，**必须 == 8**，否则整帧拒绝
 * @param  dist_cm 输出：距离（cm）指针，不可为 NULL；
 *                 成功时写入 0~65535（byte0-1 小端拼成 uint16）
 * @return TRUE = 已写出 dist_cm；FALSE = 空指针 / DLC≠8（此时**不写**出参）
 * @note   `0xFFFF` 的"无有效目标"语义由**应用层**判断，本层只负责字节→数值。
 *         本项目仅用于联调观测打印，不直接驱动动作。
 */
boolean Can_ParseObstacleDist(const uint8* data, uint8 len, uint16* dist_cm);

/**
 * @brief  组装并发送 0x125 当前车速帧（byte0-1 = uint16 小端 km/h，byte2-7 = 0）
 * @param  kmh 车速，单位 km/h，0~65535；传 0xFFFF 表示"车速无效"（§5 约定哨兵值）
 * @return TRUE/FALSE 同 Can_Send()（TRUE = 已装入邮箱，不代表对端已收到）
 * @note   发送周期必须 < 板端 recv_timeout(100ms)，本项目在 Task_20ms 里发（20ms）。
 *         换算来源：编码器 cm/s × 0.036 = km/h。
 */
boolean Can_SendVehicleSpeed(uint16 kmh);
#endif /* CAN_PROTOCOL_BRAKE_V2 */

/* ===== heartbeat_v1 帧（0x100/0x101/0x102）：解析 / 组装（纯字节操作，不碰寄存器）=====
 *   0x100 心跳/命令 → Can_ParseRdkHeartbeat()：出参 cmd=byte0（0x00~0x03，越界拒绝）；
 *                     byte1~3 一并带出（dist_cm/conf/cnt，供联调观测与 byte3 去重）
 *   0x102 应答/心跳 → Can_ParseRdkHsAck()：byte0=0x55 且 byte1∈{0x01 应答, 0x02 心跳}
 *                     才有效；出参 kind=byte1、seq=byte2（应用层按 kind/seq 记账）
 *   0x101 握手请求 ← Can_SendRdkHsReq()：byte0=0xAA、byte1=seq（seq 仅对端观测）
 * ⚠ DLC 契约（§5 通用：数据段固定 8 字节）：三帧一律 **DLC==8**，≠8 整帧拒绝。
 * ⚠ 仅当 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译。
 *   ⚠ byte3 去重（"与上一帧不同"）在**应用层**做（本层只做字节合法性）；重复帧本层照收。 */
#if CAN_PROTOCOL_HEARTBEAT_V1
/* 0x100 解析结果容器（字段直接对应 §5 的 byte0~byte3） */
typedef struct {
    uint8 cmd;       /* 0x00 停车 / 0x01 巡航 / 0x02 预警 / 0x03 制动 */
    uint8 dist_cm;   /* byte1：距离 cm（联调观测；本端不据此判定） */
    uint8 conf;      /* byte2：置信度（联调观测） */
    uint8 cnt;       /* byte3：计数（应用层据此去重） */
} Rdk_HbFrame_t;

/**
 * @brief  解析 0x100 心跳/命令帧 → 结构体（纯字节操作，不碰寄存器）
 * @param  data 收到的数据段指针，不可为 NULL（按 8 字节读）
 * @param  len  帧 DLC，**必须 == 8**，否则整帧拒绝（§5 三帧统一契约）
 * @param  out  输出结构体指针，不可为 NULL；成功时四个字段一并填好
 * @return TRUE = 帧合法且已填充 out；FALSE = 空指针 / DLC≠8 / byte0>3（此时**不写** out）
 * @note   本层只做字节合法性；**byte3 重复帧去重由应用层做**（本层照收重复帧）。
 *         byte4-7 保留位被忽略。
 */
boolean Can_ParseRdkHeartbeat(const uint8* data, uint8 len, Rdk_HbFrame_t* out);

/**
 * @brief  解析 0x102 应答/心跳帧（仅 STATIC 阶段用于握手判定）
 * @param  data 收到的数据段指针，不可为 NULL
 * @param  len  帧 DLC，**必须 == 8**
 * @param  kind 输出：帧类型指针，不可为 NULL；成功写入 0x01=应答 / 0x02=心跳
 * @param  seq  输出：序号指针，不可为 NULL；成功写入 byte2 的值（0~255，每帧 +1）
 * @return TRUE = 签名合法（byte0=0x55 且 byte1∈{0x01,0x02}）且已写出 kind/seq；
 *         FALSE = 空指针 / DLC≠8 / 签名不符（此时**不写**出参）
 * @note   ⚠ 本帧不参与任何判活、恢复、超时判定（§5 判活铁律：只看 0x100）。
 */
boolean Can_ParseRdkHsAck(const uint8* data, uint8 len, uint8* kind, uint8* seq);

/**
 * @brief  组装并发送 0x101 握手请求/心跳（byte0=0xAA，byte1=seq，byte2-7=0）
 * @param  seq 序号 0~255，每发一帧 +1（20 次后自然回绕）；板端可据此判"是否还在发"
 * @return TRUE/FALSE 同 Can_Send()（TRUE = 已装入邮箱）
 * @note   STATIC/HOLD 阶段 200ms 恒发、无次数上限；LOCK 后停发（§5）。
 *         联调时 CANable 上看到 0x101 周期稳定 = 本端链路活着。
 */
boolean Can_SendRdkHsReq(uint8 seq);
#endif /* CAN_PROTOCOL_HEARTBEAT_V1 */

/* ===== 便捷发送（旧脚手架协议 0x101/0x103）—— ⚠ 已作废（deprecated） =====
 * 这两个接口在 #if 0 里 = **不参与编译、也没有定义**，只作为历史接口形状留档；
 * 现在的 0x101 是 RDK 握手请求（见 Can_SendRdkHsReq），0x103 已不存在。 */
#if 0
/**
 * @brief  [已作废] 旧脚手架状态帧发送 0x101：status_word / current_speed /
 *         voltage_x10 / fault_code
 * @param  status_word    状态字，位域含义随旧脚手架定义（已作废，见 can_protocol.h 的 STATUS_*）
 * @param  current_speed  当前速度（旧协议的 sint8 域，单位已不可考）
 * @param  voltage_x10    电压 ×10（例如 120 = 12.0V）
 * @param  fault_code     故障码（旧 FAULT_* 位掩码，已作废）
 * @return 无
 * @deprecated 已在 `#if 0` 内：不编译、无定义。0x101 现为 STM32→RDK 的**握手请求**
 *             （0xAA + seq，200ms 恒发），语义完全不同；启用会直接破坏 §5 协议表。
 *             现行发送接口是 Can_SendRdkHsReq()。 */
void Can_SendStatus(uint8 status_word, sint8 current_speed, uint8 voltage_x10, uint8 fault_code);

/**
 * @brief  [已作废] 旧脚手架心跳帧发送 0x103
 * @param  counter 心跳计数 0~255，每帧 +1
 * @return 无
 * @deprecated 同上：仅留档、不编译。现行 heartbeat_v1 **没有 0x103**——
 *             本端心跳合并进 0x101（200ms 恒发），对端心跳/应答走 0x102。
 *             现行发送接口是 Can_SendRdkHsReq()。 */
void Can_SendHeartbeat(uint8 counter);
#endif

/* ===== 回环自测帧（0x111，仅限自测，不属于 AEB 协议表 0x123/0x124/0x125） =====
 * 数据固定 8 字节：byte0 = counter, byte1..3 = 'C','A','N', byte4..7 = 0 */
#define CAN_ID_SELFTEST   0x111u

/**
 * @brief  发送回环自测帧 0x111（{counter,'C','A','N',0,0,0,0}，DLC=8）
 * @param  counter 帧计数 0~255，写入 byte0；自测流程用它判断"有没有漏帧/乱序"
 * @return 无（内部调用 Can_Send()，失败只在自测结果里体现）
 * @note   仅回环自测镜像使用（规则 §6.1 步骤①），不属于 AEB 协议表，
 *         联调阶段不再发送；byte1-3 固定 'C','A','N' 用于防噪声帧误判 PASS。
 */
void Can_SendSelfTest(uint8 counter);

/* ===== 回环 AEB 自测组帧（⑤；仅自测镜像编译）=====
 * 0x123/0x124 在 §5 里是"板端→STM32"**单向**帧，STM32 正常不发送；
 * 这两个接口只用于回环自发自收，验证解析与制动映射。
 * 字节布局仍集中在本层（应用层只传语义值）；Normal 镜像里没有这些符号，
 * 因此不可能误发 0x123/0x124 到真实总线、污染联调。 */
#if CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2

/**
 * @brief  回环自测：组帧并发送 0x123 制动指令（byte0=level，byte1-7=0，DLC=8）
 * @param  level 制动等级 0~3，直接写入 byte0（越界不夹紧，调用方须保证 0~3）
 * @return 无
 * @note   只用于回环自发自收（验证"解析 + 制动映射"整链），Normal 镜像里没有该符号，
 *         因此不可能误把 0x123 发到真实总线污染联调。
 */
void Can_SendBrakeCmdTest(uint8 level);

/**
 * @brief  回环自测：组帧并发送 0x124 障碍物距离（byte0-1 = uint16 小端 cm，DLC=8）
 * @param  cm 距离，单位 cm，0~65535（0xFFFF 表示无有效目标，与协议一致）
 * @return 无
 * @note   同 Can_SendBrakeCmdTest()：仅自测镜像编译，用于验证小端拼装与解析。
 */
void Can_SendObstacleDistTest(uint16 cm);
#endif

#endif /* CAN_DRIVER_H */
