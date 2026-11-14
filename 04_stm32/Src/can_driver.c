/**
 * can_driver.c - CAN1 驱动实现（寄存器级，免 HAL）
 *
 * ═══════════ CAN 总线速成（读代码前先懂这几条）═══════════
 * 1. CAN 是差分总线：CANH/CANL 两根线，物理层靠 TJA1050 收发器转换。
 *    本板没有板载收发器，PA11/PA12 只是控制器引脚（TTL 电平），
 *    必须外接 TJA1050：PA12=TX 接 TXD、PA11=RX 接 RXD。
 * 2. 帧格式：标准帧 = 11 位 ID + RTR + IDE + DLC(数据长度) + 0~8 字节数据。
 *    寄存器里：TIR[31:21]=ID、TIR[0]=TXRQ；TDTR[3:0]=DLC；TDLR/TDHR=数据。
 * 3. 位时序（这是理解 BTR 数值的关键）：
 *    每个位 = SYNC_SEG(1tq) + TS1 + TS2，波特率 = APB1时钟 / (BRP × (1+TS1+TS2))
 *    APB1 = 42MHz，目标 500kbps：
 *      42MHz / (BRP × (1+TS1+TS2)) = 500k
 *      取 BRP=4、TS1=15、TS2=5 -> 42M / (4 × 21) = 500kbps ✓
 *    （原脚手架注释里的 Prescaler=6/BS1=11/BS2=4 实际算出 437.5kHz，
 *      是错的，本版本已修正为 500kbps，与 can_bridge.py 对齐。）
 * 4. 发送邮箱：bxCAN 有 3 个发送邮箱(T0/T1/T2)，任一空闲即可写入，
 *    写 TIR 置 TXRQ=1 请求发送。接收有 2 个 FIFO，本驱动只用 FIFO0。
 * 5. 过滤器：F0 用掩码模式（ID=0x000, Mask=0x000）=> 不过滤，全收。
 *
 * 引脚：PA11=CAN1_RX(AF9), PA12=CAN1_TX(AF9)
 * 接收：轮询方式，Can_ProcessRx() 在 10ms 任务里调用
 * 自测：Can_InitLoopback() 走 BTR 静默环回（LBKM|SILM=1），无需收发器/对端
 */
#include "can_driver.h"
#include <stddef.h>

/* ===== 编译期钉死 CAN 寄存器偏移（依据 RM0090 30.5 / ST 官方 stm32f407xx.h） =====
 * 历史缺陷：stm32f407_regs.h 里邮箱区前的保留段少写了 0x100（RESERVED0[24] 应为 [88]），
 * 导致 TX/RX 邮箱全部寻址到保留地址上——编译 0 警告、Can_Send 还"成功"，
 * 但总线/回环上永远收不到帧（TSR 在正确地址 0x08，掩盖了问题）。
 * 以下静态断言把关键偏移钉死，谁改结构体谁编译失败。 */
_Static_assert(offsetof(CAN_TypeDef, TSR)   == 0x008, "CAN TSR 偏移错");
_Static_assert(offsetof(CAN_TypeDef, RF0R)  == 0x00C, "CAN RF0R 偏移错");
_Static_assert(offsetof(CAN_TypeDef, BTR)   == 0x01C, "CAN BTR 偏移错");
_Static_assert(offsetof(CAN_TypeDef, TX0R)  == 0x180, "CAN TX0R(TIR0) 偏移错");
_Static_assert(offsetof(CAN_TypeDef, TX1R)  == 0x190, "CAN TX1R 偏移错");
_Static_assert(offsetof(CAN_TypeDef, TX2R)  == 0x1A0, "CAN TX2R 偏移错");
_Static_assert(offsetof(CAN_TypeDef, RX0R)  == 0x1B0, "CAN RX0R(RI0R) 偏移错");
_Static_assert(offsetof(CAN_TypeDef, RX0D2) == 0x1BC, "CAN RX0D2(RDHR0) 偏移错");
_Static_assert(offsetof(CAN_TypeDef, FMR)   == 0x200, "CAN FMR 偏移错");
_Static_assert(offsetof(CAN_TypeDef, FS1R)  == 0x20C, "CAN FS1R 偏移错");
_Static_assert(offsetof(CAN_TypeDef, FFA1R) == 0x214, "CAN FFA1R 偏移错");
_Static_assert(offsetof(CAN_TypeDef, FA1R)  == 0x21C, "CAN FA1R 偏移错");
_Static_assert(offsetof(CAN_TypeDef, F0R1)  == 0x240, "CAN F0R1 偏移错");
_Static_assert(offsetof(CAN_TypeDef, F0R2)  == 0x244, "CAN F0R2 偏移错");

static Can_RxCallback_t g_rx_callback = NULL;   /* 上层注册的收帧回调（未注册时收到的帧直接丢弃） */
static volatile uint16 g_rx_overrun = 0;   /* FIFO0 溢出累计（RF0R.FOVR0），联调观测用 */
static volatile uint16 g_recover_cnt = 0;  /* 自愈成功累计（总线修好后把节点拉回的次数） */
static boolean g_can_loopback = FALSE;     /* 当前模式：自愈重建 BTR 时必须保持同一模式 */

/* ===== CAN 初始化（内部公共实现）
 * loopback = TRUE ：BTR.LBKM|SILM = 1 → 静默环回（TX 内部接回 RX，不驱动 CANTX，
 *                  与总线隔离），无需 TJA1050/对端/120Ω 终端，用于上电自测 
 * loopback = FALSE：Normal 模式（与板端 AEB 联调）
 * 依据：RM0090 CAN_BTR[31:30] SILM/LBKM；只置 SILM 是"只收不发"，自发自收必须 LBKM|SILM
 */
/**
 * @brief  CAN 初始化的公共实现（Normal / 静默环回共用一套时序与过滤器配置）
 * @param  loopback TRUE  = 静默环回模式（额外置 LBKM|SILM，自发自收、不驱动 CANTX、
 *                        不需要收发器/对端/120Ω 终端，用于上电自测）；
 *                 FALSE = Normal 模式（真实上总线，与板端 AEB 联调）
 * @return TRUE = 已退出初始化模式、加入总线；FALSE = 有界等待内 INAK 未清 0
 * @note   模式会被记进 g_can_loopback：Can_RecoverIfStuck() 重建 BTR 时
 *         必须沿用同一模式，否则自愈会把回环节点悄悄切到 Normal 模式。
 *         两种模式的位时序数值（500kbps）完全一致，切换不引入波特率差异。
 */
static boolean can_init_internal(boolean loopback)
{
    g_can_loopback = loopback;   /* 记下来：Can_RecoverIfStuck() 重建 BTR 时要用同一模式 */

    /* 1. GPIO: PA11/PA12 复用 AF9 */
    gpio_enable_clock(GPIOA);                                       /* 开 GPIOA 时钟 */
    gpio_mode(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_MODE_AF);       /* 两脚设为复用功能（交给 bxCAN 驱动） */
    gpio_af(GPIOA, GPIO_PIN_11, 9);                                 /* PA11 = CAN1_RX，AF9 */
    gpio_af(GPIOA, GPIO_PIN_12, 9);                                 /* PA12 = CAN1_TX，AF9 */
    /* ⚠ 用 CAN1 时禁止插板载 USB SLAVE：PA11/PA12 与 OTG_FS 是同一组引脚（规则 §1） */

    /* 2. 使能 CAN1 时钟 */
    RCC->APB1ENR |= RCC_CAN1EN;                  /* CAN1 挂在 APB1（42MHz）——波特率计算用的就是这个时钟 */

    /* 3. 初始化请求 + 等待进入初始化模式 */
    CAN1->MCR |= CAN_MCR_INRQ;                   /* INRQ=1：请求进入初始化模式（此模式下才能改 BTR/过滤器） */
    uint32 timeout = 1000000;                    /* 有界等待，避免硬件异常时死等 */
    while (!(CAN1->MSR & CAN_MSR_INAK)) {        /* INAK=1 才算真正进入初始化模式 */
        if (--timeout == 0) return FALSE;        /* 超时 → 初始化失败，返回给上层（不会卡在任务里） */
    }

    /* 4. 退出睡眠模式 */
    CAN1->MCR &= ~CAN_MCR_SLEEP;                 /* 清 SLEEP：一进初始化模式硬件会自动置位它，必须清掉 */

    // /* 5. 配置位时序：BRP=4, TS1=15, TS2=5, SJW=1 -> 500kbps
    //  *    BTR = (BRP-1) | (TS1-1)<<16 | (TS2-1)<<20 | (SJW-1)<<24
    //  *        = 3 | 14<<16 | 4<<20 | 0<<24 = 0x4E0003
    //  */
    // CAN1->BTR = 3UL | (14UL << 16) | (4UL << 20) | (0UL << 24);

    /* 5. 配置位时序：APB1=42MHz，目标 500kbps
     *    42MHz / (BRP × (1+TS1+TS2)) = 500k
     *    取 BRP=4, TS1=15tq(寄存器写14), TS2=5tq(寄存器写4), SJW=1
     *    总 tq = 1+15+5 = 21，42MHz/(4×21)=500kbps
     *    采样点 = (1+15)/21 = 76.2%
     *    BTR = 3 | 14<<16 | 4<<20 = 0x004E0003
     *    （自测时再或上 CAN_BTR_LBKM|CAN_BTR_SILM，时序数值不变） */
    uint32 btr = 3UL | (14UL << 16) | (4UL << 20) | (0UL << 24);
    /* BRP-1=3（BRP=4）、TS1-1=14（TS1=15tq）、TS2-1=4（TS2=5tq）、SJW-1=0（SJW=1tq）；
     * 这四段拼出的就是上面推导的 0x004E0003，改动任一数值都会改变波特率 → 必须与板端 500k 一致 */
    if (loopback) {
        btr |= CAN_BTR_LBKM | CAN_BTR_SILM;   /* 静默环回：不驱动 CANTX，仅内部自发自收 */
        /* LBKM=环回（TX 内部接回 RX）、SILM=静默（CANTX 保持隐性，不打扰总线）；两个都要置 */
    }
    CAN1->BTR = btr;                             /* 只有在初始化模式（INAK=1）下写才生效 */


    /* 6. 自动总线恢复 ABOM + TX 优先级按邮箱号 */
    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_TXFP;
    /* ABOM=1：Bus-Off 后硬件自动尝试恢复（128×11 隐性位），配合 Can_RecoverIfStuck 双保险；
     * TXFP=1：三个邮箱按请求先后顺序发送（而非按 ID 优先级），保证 0x101 等周期帧不乱序 */

    /* 7. 过滤器 0：掩码全 0（接收所有标准帧），FIFO0 */
    CAN1->FMR |= CAN_FMR_FINIT;          /* 进入过滤器初始化模式 */
    CAN1->FM1R &= ~(1UL << 0);           /* 掩码模式 */
    /* 掩码模式下：F0R1=ID、F0R2=掩码，掩码位为 1 才比较，为 0 表示"该位不关心" */
    CAN1->FS1R |= (1UL << 0);            /* 32位过滤器 */
    /* 32 位过滤器：一个寄存器对（ID+掩码）过滤一整个 ID 范围，标准帧/扩展帧都能匹配 */
    CAN1->FFA1R &= ~(1UL << 0);          /* 关联 FIFO0 */
    /* 清这一位 = 过滤器 0 的命中帧进 FIFO0（Can_ProcessRx 只轮询 FIFO0） */
    CAN1->FA1R &= ~(1UL << 0);           /* 先失活 */
    /* 改过滤器内容前必须先失活，否则写进去的值可能不生效（RM0090 要求） */
    CAN1->F0R1 = 0x00000000UL;           /* ID 全 0 */
    CAN1->F0R2 = 0x00000000UL;           /* Mask 全 0 = 全部接收 */
    /* 掩码全 0 → 任何 ID 都能通过（本项目 0x100/0x101/0x102 或 0x123/0x124/0x125 都能收） */
    CAN1->FA1R |= (1UL << 0);            /* 激活 */
    CAN1->FMR &= ~CAN_FMR_FINIT;         /* 退出过滤器初始化模式 */
    /* 退出后过滤器才真正开始工作；不清 FINIT 则过滤器一直处于配置态、不收帧 */

    /* 8. 退出初始化模式 */
    CAN1->MCR &= ~CAN_MCR_INRQ;          /* 清 INRQ：请求退出初始化模式、加入总线 */
    timeout = 1000000;                   /* 重新装载有界等待 */
    while (CAN1->MSR & CAN_MSR_INAK) {   /* 等 INAK 变 0（需要总线上出现连续隐性位，硬件才会退出） */
        if (--timeout == 0) return FALSE;   /* 退不出去 = 总线异常（无对端/缺终端电阻/未共地），交上层处理 */
    }

    /* 9. 软件复位 FIFO */
    CAN1->RF0R |= CAN_RF0R_RFOM0;        /* 释放 FIFO0 输出邮箱，确保计数从空开始（上电残留不带到运行期） */

    return TRUE;                         /* 已进入 Normal 模式，可以收发 */
}

/* ===== 对外接口：Normal 模式（行为与拆分前完全一致） ===== */
/**
 * @brief  CAN1 初始化到 Normal 模式（500kbps，标准帧，DLC≤8），可真正收发
 * @return TRUE = 已退出初始化模式、总线可用；FALSE = 退出 INAK 失败（总线异常：
 *         无对端/缺 120Ω 终端/未共地/波特率不符），调用方应打印告警并靠
 *         Can_RecoverIfStuck() 每秒重试
 * @note   ① 内部复用 can_init_internal(FALSE)（INRQ 往返 + BTR + 过滤器，幂等、等待有界）；
 *         ② .clinerules §1 的硬件前提必须成立：CANH/CANL/GND 三线 + 两端各 120Ω；
 *         ③ 用 CAN1 时禁止插板载 USB SLAVE 口（PA11/PA12 与 OTG_FS 冲突）；
 *         ④ 失败不代表外设坏：总线合格后 Can_RecoverIfStuck() 能把它拉回来。
 */
boolean Can_Init(void)
{
    return can_init_internal(FALSE);     /* 联调用的正常模式：真正驱动 CANTX，与板端 AEB 通信 */
}

/* ===== 对外接口：静默环回模式（上电自测，规则 §6.1 步骤①） ===== */
/**
 * @brief  CAN1 初始化到静默环回模式（BTR.LBKM|SILM），用于上电自测
 * @return TRUE = 环回模式就绪；FALSE = 初始化失败（极少见，多为时钟/引脚配置问题）
 * @note   ① 环回下自己发的帧会被自己收回，**不需要** TJA1050/对端/120Ω 终端，
 *           仍会消耗总线时隙（SILM 不驱动 CANTX，不影响真实总线）；
 *         ② 仅 CAN_LOOPBACK_SELFTEST 自测镜像使用（规则 §6.1 步骤①）；
 *         ③ 与 Can_Init() 互斥：后调用者覆盖前者，运行期不要来回切。
 */
boolean Can_InitLoopback(void)
{
    return can_init_internal(TRUE);      /* 自测用的环回模式：不需要 TJA1050/对端/120Ω */
}

/* ===== 自愈：硬件卡在初始化态（§6.4「Bus-Off 必须有恢复逻辑」）=====
 * 真机实测（2026-09-27，ST-Link 实测记录见 README §「Bus-Off 自愈」）：
 *   现象  MCR=0x00010044(INRQ=0) MSR=0x00000009(INAK=1) TSR=0x82000000(3 邮箱永久占满+Terr) ESR=0，
 *          1.2s/1.5s 周期连续 21 次采样完全不变 → Can_Send() 每次都走满 100000 次有界等待后
 *          返回 FALSE，0x125 永远发不出去、0x123 也收不到（表现为"车在总线上假死"）。
 *   根因  bxCAN **离开初始化模式需要总线上出现连续隐性位**。当"无对端 + 无/缺 120Ω 端接
 *          + 未共地"时差动线悬空，收发器收回的 RXD 不构成连续隐性位 → 外设无法同步，
 *          bus-off 后连 ABOM 也退不出来，永久停在 init 态。
 *          （实测三条互相印证的证据：① 让板子脱离调试器自由跑 5s 后首次 attach 仍是
 *            MSR=0x09、自愈计数=0；② 只清 MCR.DBF 后 1.5s 仍是 0x09 → 与调试冻结无关；
 *            ③ 用调试器把 BTR 临时改成 LBKM|SILM（环回，不依赖外部总线）后 INAK 立刻清 0、
 *            TSR 变 0x1C030303 = 三邮箱全空 + 三个 TXOK → 外设与驱动本身完好。）
 *   定位  所以这不是固件 bug，而是 .clinerules §1 的 CAN 硬件前提未具备（CANH/CANL/GND 三线
 *          + 总线两端各 120Ω + 共地）。本函数的职责是**兜底**：总线修好（或对端上电）后，
 *          若节点还停在 init 态，就在 1s 内把它拉回总线，不需要重新上电。
 * 实现：卡住时复用 can_init_internal()（INRQ 往返 + BTR 重写 + 过滤器重装，全部幂等，
 *       且所有等待都有界，不在任务里阻塞）；成功后累加 g_recover_cnt 供 UART/SWD 观测。
 * 返回：TRUE = 本次执行了自愈且已回到总线；FALSE = 没卡住 / 软件正主动初始化 /
 *       自愈失败（典型场景 = 总线仍不合格，下个周期再试）。*/
/**
 * @brief  总线自愈：硬件卡在初始化态（INAK=1 且 INRQ=0）时把节点拉回总线
 * @return TRUE = 本次执行了自愈且已回到总线；FALSE = 没卡住 / 软件正主动初始化 /
 *         自愈失败（总线仍不合格，下个周期会再试）
 * @note   ① 根因是 bxCAN 退出初始化需要总线上出现连续隐性位：无对端 + 缺 120Ω +
 *           未共地时差动线悬空，硬件会永久停在 init 态（TSR 三邮箱占满、0x125 发不出）；
 *         ② 复用 can_init_internal()（幂等 + 全部有界等待），**不在任务里长阻塞**；
 *         ③ 只在“硬件自己卡住”时动手（INAK=1 且 INRQ=0），不干扰主动初始化；
 *         ④ 建议每 1s 调一次（Task_1000ms），成功次数见 Can_GetRecoverCount()。
 */
boolean Can_RecoverIfStuck(void)
{
    boolean ok;

    /* 只处理"硬件自己卡住"：已是初始化态，但软件并没有请求 INRQ */
    if (!(CAN1->MSR & CAN_MSR_INAK)) return FALSE;      /* INAK=0 → 节点正常在总线上，不自愈 */
    if (CAN1->MCR & CAN_MCR_INRQ)    return FALSE;      /* INRQ=1 → 是我们自己主动在初始化（非卡死），别插手 */

    ok = can_init_internal(g_can_loopback);             /* 复用整套初始化（幂等 + 全部有界等待，不阻塞任务） */
    if (ok) g_recover_cnt++;                            /* 只有真正回到总线才计数，便于抓包/串口观测自愈次数 */
    return ok;                                          /* FALSE = 自愈失败（总线仍不合格），下个周期会再试 */
}

/* ===== 发送 ===== */
/**
 * @brief  发送一帧标准数据帧（自动挑空闲邮箱）
 * @param  can_id 标准帧 ID，11 位有效（0x000 ~ 0x7FF），高 5 位被忽略
 * @param  data   数据缓冲首地址，**不可为 NULL**；至少 len 个字节
 * @param  len    数据长度 0~8；>8 会被夹到 CAN_MAX_DATA_LEN(=8)
 * @return TRUE = 帧已成功**装入邮箱**（TXRQ 已置位）；FALSE = 三个邮箱长期满载，
 *         有界等待超时放弃 —— 调用方可下个周期重发
 * @note   ① TRUE **不代表对端收到/ACK 成功**：真实总线状态看 Can_IsBusOff()/错误计数；
 *         ② 三邮箱全满时最多等 10 万次循环即返回，不阻塞任务（规则 §3）；
 *         ③ 只支持标准帧、数据帧（IDE=RTR=0），不发远程帧/扩展帧；
 *         ④ TIR.TXRQ 在最后一步置位，避免奇数 DLC 被提前当成发送请求。
 */
boolean Can_Send(uint16 can_id, const uint8* data, uint8 len)
{
    uint32 timeout = 100000;         /* 有界等待上限：三个邮箱全满时最多等这么久就放弃（不阻塞任务） */
    volatile uint32 *tir;            /* 指向选中邮箱的 TIR 寄存器（用它 +1/+2/+3 访问同组寄存器） */
    uint32 dl, dh;                   /* 待写进 TDLR/TDHR 的低 4 字节 / 高 4 字节 */
    uint8 i;                         /* 字节循环下标 */

    if (len > CAN_MAX_DATA_LEN) len = CAN_MAX_DATA_LEN;   /* 兜底：本项目 DLC 上限 8（规则 §1/§5） */

    /* 等待有空邮箱（TME0/1/2 任一为 1） */
    while (!(CAN1->TSR & CAN_TSR_TME)) {   /* TME=1 表示至少有一个邮箱空闲（上一帧已成功发出） */
        if (--timeout == 0) return FALSE;  /* 一直满载 → 返回失败，调用方可下个周期重发（不在这里死等） */
    }

    /* 选邮箱 */
    if (CAN1->TSR & CAN_TSR_TME0) {
        tir = &CAN1->TX0R;           /* 邮箱 0 空闲，优先用它 */
    } else if (CAN1->TSR & CAN_TSR_TME1) {
        tir = &CAN1->TX1R;           /* 否则用邮箱 1 */
    } else {
        tir = &CAN1->TX2R;           /* 只剩邮箱 2（前面已确认至少有一个空闲） */
    }

    /* TIR: 标准帧(ID<<21) + 数据帧(RTR=0, IDE=0)；标准帧的 DLC 在 TDTR，不写进 TIR
     * ⚠ TXRQ(TIR bit0) 必须最后置 1：TIR bit0 就是 TXRQ，若在这里就把 len 或上，
     *   奇数 DLC(len=1/3/5/7) 会立刻置位 TXRQ → 在 TDTR/TDLR/TDHR 还没写完时
     *   就带着邮箱里的旧数据发帧（偶数 DLC 只是侥幸不触发）。
     *   正确顺序：写 TIR(不含 TXRQ) → TDTR → TDLR → TDHR → 最后置 TXRQ。 */
    *tir = ((uint32)can_id << 21);
    /* 标准帧：TIR 的 [31:21] 就是 11 位 STID；IDE=0 + RTR=0（数据帧）由"只写 ID"天然成立 */

    /* TDTR: DLC */
    *(tir + 1) = (uint32)len;        /* 偏移 +0x4 = TDTR（数据长度），值就是 DLC 0~8 */

    /* TDLR/TDHR: 8 字节数据 */
    dl = 0; dh = 0;                  /* 先清零：未使用的字节保持 0（规则 §5"未用字节填 0"） */
    for (i = 0; i < 8; i++) {
        if (i < len) {                                       /* 只搬 DLC 范围内的字节 */
            if (i < 4) dl |= ((uint32)data[i]) << (8 * i);    /* byte0~3 → TDLR，byte0 落在最低字节（小端） */
            else       dh |= ((uint32)data[i]) << (8 * (i - 4));  /* byte4~7 → TDHR，byte4 落在最低字节 */
        }
    }
    *(tir + 2) = dl;                 /* 偏移 +0x8 = TDLR（数据低 4 字节） */
    *(tir + 3) = dh;                 /* 偏移 +0xC = TDHR（数据高 4 字节） */

    /* TXRQ 请求发送 */
    *tir |= (1UL << 0);              /* TIR bit0 = TXRQ，置 1 后硬件立刻把该邮箱内容发上总线 */
    return TRUE;                     /* 只表示"已成功装入邮箱"，不代表已收到 ACK（看 ESR 判 Bus-Off） */
}

/* ===== 接收回调注册 ===== */
/**
 * @brief  注册接收回调（由 Can_ProcessRx() 在任务上下文里同步调用）
 * @param  callback 回调函数指针，可为 NULL（NULL = 取消注册，收到的帧直接丢弃）
 * @return 无
 * @note   ① 只在 Can_ProcessRx() 里调用，**不在中断上下文**，因此回调里可以调
 *           uart_puts 等；但仍必须短小（FIFO 只有 3 级，回调慢会丢帧）；
 *         ② 覆盖式注册：再次调用即替换，工程里只在 main() 注册一次；
 *         ③ 建议在 Can_Init()/Can_InitLoopback() 之前注册，避免漏掉最早几帧。
 */
void Can_RegisterRxCallback(Can_RxCallback_t callback)
{
    g_rx_callback = callback;        /* 只存函数指针；真正的分发在 Can_ProcessRx 里做（不中断里回调） */
}

/* ===== 轮询接收 ===== */
/**
 * @brief  轮询接收：把 FIFO0 整批排空，逐帧回调给已注册的处理函数
 * @return 无
 * @note   ① 必须 **while 排空**（FIFO 深度仅 3）：残留帧会被下一次轮询当成新帧，
 *           造成时间戳错位，满 FIFO 还会溢出丢帧；
 *         ② 每次调用最多处理 8 帧（兜底防死循环），保证在 10ms 任务里不会长阻塞；
 *         ③ 每帧处理完写 RFOM0 释放邮箱，否则 FMP0 不减、会重复读同一帧；
 *         ④ FIFO0 溢出（FOVR0）会累加 Can_GetRxOverrunCount() 并清标志；
 *         ⑤ 由 Task_10ms 每拍调用；回调里禁止 delay_ms/长日志。
 */
void Can_ProcessRx(void)
{
    uint8 drained = 0;               /* 本次已处理的帧数（防止 FIFO 异常时死循环） */

    /* FIFO0 排空：0x123/0x124 各按 20ms 发，10ms 轮询窗口内通常只有 1~2 帧，
     * 但串口打印/调度抖动会把窗口拉长，FIFO 深度只有 3 → 必须 while 排空，
     * 否则残留帧会被下一次轮询当成"新帧"处理（时间戳错位），且满 FIFO 会溢出丢帧。 */
    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {   /* FMP0 = FIFO0 里待读帧数，非 0 就还有帧 */
        uint16 can_id;                /* 本帧 ID */
        uint8  data[8];               /* 本帧数据（固定按 8 字节装） */
        uint8  len;                   /* 本帧 DLC */
        uint32 rdlr, rdhr;            /* 数据低/高 4 字节（一次读整个 32 位寄存器，避免两次读不一致） */
        uint8  i;                     /* 字节拆分下标 */

        can_id = (uint16)((CAN1->RX0R >> 21) & 0x7FF);   /* RIR0 */
        /* STID 在 [31:21]，标准帧 11 位；&0x7FF 去掉高位（IDE/RTR 等） */
        len = (uint8)(CAN1->RX0D0 & 0x0F);               /* RDTR0: DLC */
        /* DLC 在 RDTR 的低 4 位 */
        if (len > 8) len = 8;                            /* 经典 CAN 的 DLC>8 表示 8 字节，这里夹住后按 8 处理 */

        rdlr = CAN1->RX0D1;                              /* RDLR0 */
        rdhr = CAN1->RX0D2;                              /* RDHR0 */
        for (i = 0; i < 8; i++) {
            if (i < 4) data[i] = (uint8)(rdlr >> (8 * i));           /* 低 4 字节：byte0 在最低字节 */
            else       data[i] = (uint8)(rdhr >> (8 * (i - 4)));     /* 高 4 字节：byte4 在最低字节 */
        }

        /* 释放 FIFO */
        CAN1->RF0R |= CAN_RF0R_RFOM0;    /* 写 1 释放当前输出邮箱，FMP0 才会减 1（否则死循环读同一帧） */

        if (g_rx_callback != NULL) {
            g_rx_callback(can_id, data, len);    /* 交给上层（应用层）按 ID 分派与判活 */
        }

        /* 兜底：FIFO 深度仅 3，若 RFOM0 万一未清 FMP0，限定最多 8 次，
         * 避免在 10ms 任务里死循环（规则 §3：任务内禁止长阻塞） */
        if (++drained >= 8u) break;
    }

    /* FIFO0 溢出观测（RM0090 30.9.5：FOVR0 由硬件置位、软件写 1 清除） */
    if ((CAN1->RF0R & CAN_RF0R_FOVR0) != 0) {
        g_rx_overrun++;                  /* 溢出 = 真丢帧，累加供串口/SWD 观测 */
        CAN1->RF0R |= CAN_RF0R_FOVR0;    /* 写 1 清标志（不清会一直挂着，下次判断恒为真） */
    }
}

/* ===== 状态查询 ===== */
/**
 * @brief  查询是否处于 Bus-Off
 * @return TRUE = ESR.BOFF=1（TEC>255，节点已自动脱离总线）；
 *         FALSE = 正常在线（不代表通信质量好，还要看错误计数）
 * @note   Bus-Off 后必须能恢复（规则 §6.6）：本工程靠 Can_RecoverIfStuck() 每秒兜底；
 *         Bus-Off 期间不要相信“发出去就成功”的语义，0x125 会用 0xFFFF 上报。
 */
boolean Can_IsBusOff(void)
{
    return (CAN1->ESR & CAN_ESR_BOFF) ? TRUE : FALSE;   /* BOFF=1：TEC>255，节点已脱离总线（必须能恢复，规则 §6.6） */
}

/**
 * @brief  读发送错误计数器
 * @return TEC，0~255（>127 已进入 error-passive，255 触发 Bus-Off）
 * @note   只读、无副作用；典型健康值 0。刷屏式增长说明无 ACK / 终端电阻缺失。
 */
uint8 Can_GetTxErrorCount(void)
{
    return (uint8)((CAN1->ESR & CAN_ESR_TEC) >> 16);    /* TEC 在 ESR[23:16]：发送错误计数 0~255 */
}

/**
 * @brief  读接收错误计数器
 * @return REC，0~255（>127 进入 error-passive）
 * @note   只读、无副作用；接收错误通常来自波特率不匹配、共地不良或噪声干扰。
 */
uint8 Can_GetRxErrorCount(void)
{
    return (uint8)((CAN1->ESR & CAN_ESR_REC) >> 24);    /* REC 在 ESR[31:24]：接收错误计数 0~255 */
}

/**
 * @brief  读 FIFO0 溢出丢帧累计次数
 * @return 自上电以来 FOVR0 触发次数（0 = 10ms 轮询一直跟得上）
 * @note   只读、无副作用；>0 说明轮询不及时（串口打印过长/任务阻塞/回调太慢），
 *         这些帧是真的丢了，不是统计误差。
 */
uint16 Can_GetRxOverrunCount(void)
{
    return g_rx_overrun;        /* 累计丢帧次数：>0 说明 10ms 轮询没跟上（FIFO 深度只有 3） */
}

/**
 * @brief  读总线自愈成功累计次数
 * @return 自上电以来 Can_RecoverIfStuck() 成功把节点拉回总线的次数
 * @note   只读、无副作用；>0 说明总线曾不合格（掉电/线松/缺 120Ω）后被自愈拉回，
 *         是排查“车在总线上假死”的关键观测量。
 */
uint16 Can_GetRecoverCount(void)
{
    return g_recover_cnt;       /* 累计自愈次数：>0 说明总线曾不合格、后来被拉回来了 */
}

/* ===== 旧脚手架便捷发送（0x101/0x103）：不属 §5，留档停用 ===== */
#if 0
/**
 * @brief  [已作废] 旧脚手架状态帧发送（ID 0x101 / byte0=状态字）—— 留档·不参与编译
 * @param  status_word    状态字，写入 byte0
 * @param  current_speed  当前速度（截成 int8），写入 byte1
 * @param  voltage_x10    电压 ×10，写入 byte2
 * @param  fault_code     故障码，写入 byte3
 * @return 无
 * @deprecated 本函数整体位于 `#if 0` 内：0x101 在 heartbeat_v1 里是 STM32 的握手请求帧
 *             （0xAA + seq），语义完全不同，启用会破坏 §5 协议表。仅作为历史记录保留，
 *             取消注释前必须先改协议文档并同步板端。现行发送接口是 Can_SendRdkHsReq()。
 */
void Can_SendStatus(uint8 status_word, sint8 current_speed, uint8 voltage_x10, uint8 fault_code)
{
    uint8 data[8];

    data[0] = status_word;
    data[1] = (uint8)current_speed;
    data[2] = voltage_x10;
    data[3] = fault_code;
    data[4] = 0;   /* 编码器数据由 main.c 填充 */
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    Can_Send(CAN_ID_STATUS, data, 8);
}

/**
 * @brief  [已作废] 旧脚手架心跳帧发送（ID 0x103）—— 留档·不参与编译
 * @param  counter 心跳计数，写入 byte0
 * @return 无
 * @deprecated 同样位于 `#if 0` 内：现行 heartbeat_v1 没有 0x103，心跳/应答统一走 0x102。
 *             保留仅为追踪早期 CAN 脚手架，禁止直接启用（会与板端协议不一致）。
 *             现行发送接口是 Can_SendRdkHsReq()。
 */
void Can_SendHeartbeat(uint8 counter)
{
    uint8 data[8] = {0};
    data[0] = counter;
    Can_Send(CAN_ID_HEARTBEAT_STM, data, 8);
}
#endif

/* ===== AEB 协议帧（brake_v2，.clinerules §5）：解析 / 组装 =====
 * 本层只做"字节 ↔ 语义值"转换与合法性检查，不含业务判断（业务在应用层）。
 * 约定：小端、未用字节填 0、DLC 不足即视为非法帧（返回 FALSE 且不写出参）。
 * ⚠ 仅当 CAN_PROTOCOL_BRAKE_V2=1 时编译。 */
#if CAN_PROTOCOL_BRAKE_V2
/**
 * @brief  解析 0x123 制动指令帧（brake_v2）
 * @param  data  收到的数据段指针，**不可为 NULL**
 * @param  len   帧 DLC，必须 == 8（§5 数据段固定 8 字节，短帧整帧拒绝）
 * @param  level 输出：制动等级 0 正常 / 1 预警 / 2 部分制动 / 3 紧急制动
 * @return TRUE = 解析成功（*level 已写入）；FALSE = 空指针 / DLC≠8 / byte0>3，
 *         此时**不写出参**（*level 保持调用方初值）
 * @note   ① 只做“字节 → 数值”与合法性检查，不含任何业务判断（业务在应用层）；
 *         ② DLC 必须严格 ==8：真机首跑时 `123#03`（DLC=1）曾被放行并改写等级；
 *         ③ 仅当 CAN_PROTOCOL_BRAKE_V2=1 时编译（与 heartbeat_v1 二选一）。
 */
boolean Can_ParseBrakeCmd(const uint8* data, uint8 len, uint8* level)
{
    if ((data == NULL) || (level == NULL)) return FALSE;       /* 空指针保护 */
    /* §5：数据段固定 8 字节 —— DLC≠8 整帧拒绝（真机首跑：短帧曾绕过解析改写等级） */
    if ((uint32)len != (uint32)CAN_DATA_LEN) return FALSE;
    if ((uint32)data[0] > (uint32)BRAKE_LEVEL_EMERGENCY) return FALSE;   /* 仅 0~3 合法 */
    *level = data[0];                                          /* 0正常 / 1预警 / 2部分制动 / 3紧急制动 */
    return TRUE;
}

/**
 * @brief  解析 0x124 障碍物距离帧（brake_v2）
 * @param  data    收到的数据段指针，**不可为 NULL**
 * @param  len     帧 DLC，必须 == 8
 * @param  dist_cm 输出：距离，单位 cm（小端：byte0 低、byte1 高）；
 *                 0xFFFF 表示“无有效目标”，语义由应用层解释
 * @return TRUE = 解析成功（*dist_cm 已写入）；FALSE = 空指针 / DLC≠8，
 *         此时**不写出参**
 * @note   本层不判断 0xFFFF 的业务含义，也不做滤波/超时；仅当 brake_v2 时编译。
 */
boolean Can_ParseObstacleDist(const uint8* data, uint8 len, uint16* dist_cm)
{
    if ((data == NULL) || (dist_cm == NULL)) return FALSE;     /* 空指针保护 */
    /* §5：数据段固定 8 字节 —— 与 0x123 同一契约（短帧整帧拒绝） */
    if ((uint32)len != (uint32)CAN_DATA_LEN) return FALSE;
    /* 小端：byte0 低字节、byte1 高字节；0xFFFF 语义（无有效目标）由应用层判断 */
    *dist_cm = (uint16)(((uint16)data[1] << 8) | (uint16)data[0]);
    return TRUE;
}

/**
 * @brief  组装并发送 0x125 车速上报帧（brake_v2）
 * @param  kmh 车速，单位 km/h（小端写入 byte0-1）；0xFFFF = 车速无效占位
 * @return TRUE = 已装入邮箱；FALSE = 邮箱长期满载（见 Can_Send）
 * @note   ① DLC 固定 8，byte2-7 填 0（§5 未用字节填 0）；
 *         ② 发送周期必须 < 板端 recv_timeout(100ms)，本工程用 20ms；
 *         ③ 只组帧，寄存器操作全在 Can_Send() 内；仅 brake_v2 时编译。
 */
boolean Can_SendVehicleSpeed(uint16 kmh)
{
    uint8 data[8];                             /* 组帧缓冲 */

    data[0] = (uint8)(kmh & 0xFFu);            /* 小端：低字节在前 */
    data[1] = (uint8)((kmh >> 8) & 0xFFu);     /* 高字节；0xFFFF = 车速无效哨兵（应用层决定） */
    data[2] = 0;                               /* §5：byte2-7 保留填 0 */
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    return Can_Send(CAN_ID_VEHICLE_SPEED, data, 8);   /* 0x125，DLC 固定 8 */
}
#endif /* CAN_PROTOCOL_BRAKE_V2 */

/* ===== heartbeat_v1 帧（0x100/0x101/0x102）：解析 / 组装 =====
 * 与 AEB 段同一套纪律：本层只做"字节 ↔ 语义值"转换与合法性检查，不做业务判断。
 * DLC 契约见 can_protocol.h：三帧数据段固定 8 字节，DLC≠8 整帧拒绝。
 * ⚠ 仅当 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译。 */
#if CAN_PROTOCOL_HEARTBEAT_V1
/**
 * @brief  解析 0x100 心跳/命令帧（heartbeat_v1）
 * @param  data 收到的数据段指针，**不可为 NULL**
 * @param  len  帧 DLC，必须 == 8
 * @param  out  输出结构体指针，**不可为 NULL**；成功时写 cmd / dist_cm / conf / cnt
 * @return TRUE = 解析成功；FALSE = 空指针 / DLC≠8 / byte0>3，此时**不写出参**
 * @note   ① 字段布局：byte0 命令(0~3)、byte1 距离 cm、byte2 置信度、byte3 计数，
 *           byte4-7 保留忽略；
 *         ② 本层**不做去重与判活**：byte3 是否“与上帧不同”由应用层
 *           Rdk_OnHeartbeat() 判定（那才是“有效帧”的完整定义）；
 *         ③ 仅当 CAN_PROTOCOL_HEARTBEAT_V1=1 时编译。
 */
boolean Can_ParseRdkHeartbeat(const uint8* data, uint8 len, Rdk_HbFrame_t* out)
{
    if ((data == NULL) || (out == NULL)) return FALSE;          /* 空指针保护：不写出参、不读空指针 */
    if ((uint32)len != (uint32)CAN_DATA_LEN) return FALSE;      /* DLC≠8 整帧拒绝 */
    if (data[0] > RDK_CMD_STOP) return FALSE;                   /* 命令码只认 0x00~0x03 */

    out->cmd     = data[0];                                     /* byte0：命令 0x00停车/01巡航/02预警/03制动 */
    out->dist_cm = data[1];                                     /* byte1：距离 cm（1 字节无符号） */
    out->conf    = data[2];                                     /* byte2：置信度 0~255 */
    out->cnt     = data[3];                                     /* byte3：计数（应用层去重） */
    return TRUE;                                                /* byte4-7 保留，不解释 */
}

/**
 * @brief  解析 0x102 应答/心跳帧（heartbeat_v1）
 * @param  data 收到的数据段指针，**不可为 NULL**
 * @param  len  帧 DLC，必须 == 8
 * @param  kind 输出：帧类型 0x01 = 握手应答 / 0x02 = 普通心跳
 * @param  seq  输出：byte2 序号 0~255（递增性由应用层判定）
 * @return TRUE = 解析成功（kind/seq 已写入）；FALSE = 空指针 / DLC≠8 /
 *         byte0≠0x55 / byte1 不是 0x01/0x02，此时**不写出参**
 * @note   本层只校验“格式合法”，不判断该帧能否用于握手（只有 STATIC 阶段的
 *         0x01 且 seq 递增才计数，见 Rdk_OnHsAck()）。
 */
boolean Can_ParseRdkHsAck(const uint8* data, uint8 len, uint8* kind, uint8* seq)
{
    if ((data == NULL) || (kind == NULL) || (seq == NULL)) return FALSE;   /* 空指针保护 */
    if ((uint32)len != (uint32)CAN_DATA_LEN) return FALSE;      /* §5 通用契约：DLC=8 */
    if (data[0] != RDK_HS_ACK_TAG0) return FALSE;               /* 必须 0x55 */
    if ((data[1] != RDK_HS_ACK_KIND_ACK) &&
        (data[1] != RDK_HS_ACK_KIND_HB)) return FALSE;          /* byte1：0x01 应答 / 0x02 心跳 */

    *kind = data[1];                                            /* 只有 0x01/0x02 两种合法类型 */
    *seq  = data[2];                                            /* byte2：seq（每帧 +1） */
    return TRUE;
}

/**
 * @brief  组装并发送 0x101 握手请求帧（heartbeat_v1）
 * @param  seq 序号 byte1，0~255；每发一帧由应用层 +1，板端可据此判重/判活
 * @return TRUE = 已装入邮箱；FALSE = 邮箱长期满载（见 Can_Send）
 * @note   ① 帧内容 {0xAA, seq, 0,0,0,0,0,0}，DLC 固定 8；
 *         ② 本函数**不计数、不限次数**：周期控制与 seq 维护都在应用层
 *           （Rdk_SendHsReq()），驱动层只负责组帧与发送；
 *         ③ STATIC/CRUISE/HOLD 三个阶段都恒发，LOCK 阶段停发。
 */
boolean Can_SendRdkHsReq(uint8 seq)
{
    uint8 data[8];                             /* 组帧缓冲：一次性组好再交给 Can_Send，避免半帧状态 */

    data[0] = RDK_HS_REQ_TAG;                  /* 0xAA：握手请求签名 */
    data[1] = seq;                             /* seq：每重发一次 +1，板端可据此去重 */
    data[2] = 0;                               /* 未用字节填 0 */
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    return Can_Send(CAN_ID_RDK_HS_REQ, data, 8);   /* 0x101，DLC 固定 8（协议 §5 契约） */
}
#endif /* CAN_PROTOCOL_HEARTBEAT_V1 */

/* ===== 环回自测帧：0x111，{counter,'C','A','N',0,0,0,0} =====
 * 仅用于上电自检（不属于 AEB 协议表 0x123/0x124/0x125，联调阶段不再发送）
 * 8 字节固定：byte1..3 用 'C','A','N' 做帧内容校验，防止把噪声帧误判为 PASS
 */
/**
 * @brief  发送环回自测帧 0x111：{counter, 'C', 'A', 'N', 0, 0, 0, 0}
 * @param  counter 计数值 0~255，写入 byte0；回读时用来确认“收到的就是刚发的那帧”
 * @return 无（忽略 Can_Send 的返回值：自测是否通过由回读判定）
 * @note   ① 不属于 AEB 协议表（0x123/0x124/0x125），只在环回自测镜像里发送；
 *         ② byte1-3 用 'C','A','N' 做内容签名，防止把噪声帧误判为 PASS；
 *         ③ 需要环回模式（Can_InitLoopback）才能收回本帧。
 */
void Can_SendSelfTest(uint8 counter)
{
    uint8 data[8];                 /* 固定 8 字节自测帧 */

    data[0] = counter;             /* byte0：计数，用于确认收发的连续性 */
    data[1] = 'C';                 /* byte1..3 固定 'C','A','N'：内容校验 */
    data[2] = 'A';
    data[3] = 'N';
    data[4] = 0;                   /* 其余字节填 0 */
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    Can_Send(CAN_ID_SELFTEST, data, 8);    /* 环回模式下这一帧会被自己收回来，证明收发通路上电正常 */
}

/* ===== 回环 AEB 自测组帧（⑤，仅自测镜像且 brake_v2 时编译）=====
 * 与 Can_SendVehicleSpeed 同风格：只做字节布局，寄存器操作仍全在 Can_Send() 里。
 * 0x123：byte0 = 等级，byte1..7 = 0（§5：未用字节填 0），DLC 固定 8
 * 0x124：byte0-1 = uint16 小端 cm（0xFFFF = 无效哨兵），byte2..7 = 0 */
#if CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2
/**
 * @brief  环回自测：模拟板端发送 0x123 制动指令帧
 * @param  level 制动等级 0~3，写入 byte0（byte1-7 填 0，DLC=8）
 * @return 无（忽略 Can_Send 返回值，判定由回读断言完成）
 * @note   仅 `CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2` 镜像编译；
 *         环回下本帧会被自己收回，从而驱动整条 AEB 链路做自测。
 */
void Can_SendBrakeCmdTest(uint8 level)
{
    uint8 data[8];                 /* 自测帧：模拟板端发来的 0x123 */

    data[0] = level;               /* byte0：制动等级 0~3（与真实 0x123 布局一致） */
    data[1] = 0;                   /* byte1..7 按协议填 0 */
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    Can_Send(CAN_ID_BRAKE_CMD, data, 8);   /* 0x123：环回下由本机收回，驱动完整 AEB 链路自测 */
}

/**
 * @brief  环回自测：模拟板端发送 0x124 障碍物距离帧
 * @param  cm 距离，单位 cm（小端写入 byte0-1）；0xFFFF = 无有效目标
 * @return 无（忽略 Can_Send 返回值，判定由回读断言完成）
 * @note   仅 `CAN_LOOPBACK_SELFTEST && CAN_PROTOCOL_BRAKE_V2` 镜像编译；
 *         byte2-7 按协议填 0，DLC 固定 8。
 */
void Can_SendObstacleDistTest(uint16 cm)
{
    uint8 data[8];                 /* 自测帧：模拟板端发来的 0x124 */

    data[0] = (uint8)(cm & 0xFFu);            /* 小端：低字节在前 */
    data[1] = (uint8)((cm >> 8) & 0xFFu);     /* 高字节；0xFFFF = 无有效目标 */
    data[2] = 0;                   /* byte2..7 按协议填 0 */
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    Can_Send(CAN_ID_OBSTACLE_DIST, data, 8);   /* 0x124：距离帧，联调观察用 */
}
#endif
