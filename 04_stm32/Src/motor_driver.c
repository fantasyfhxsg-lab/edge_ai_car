/**
 * motor_driver.c - 电机驱动实现（TB6612FNG 双通道，寄存器级）
 *
 * ═══════════ TB6612 工作原理（读代码前先懂这个）═══════════
 * TB6612 是"两路 H 桥"：每路有 IN1/IN2 两个方向脚 + 1 个 PWM 脚。
 * 电机转速由 PWM 占空比决定，方向由 IN1/IN2 组合决定：
 *   IN1 IN2 PWM  ->  状态
 *   H   H   X       刹车(短路制动：电机两端短接，转动的轮子被强行停住)
 *   L   L   X       停止(滑行：电机悬空，惯性滚动)
 *   H   L   PWM     正转(转速∝PWM占空比)
 *   L   H   PWM     反转(转速∝PWM占空比)
 * STBY=1 整片使能，STBY=0 全部输出高阻（电机完全不工作）。
 *
 * 引脚（STM32F407ZGT6）：
 *   PWMA=PA6(TIM3_CH1)  AIN1=PB0  AIN2=PB1   -> 左轮(JGB37-520)
 *   PWMB=PA7(TIM3_CH2)  BIN1=PB4  BIN2=PB5   -> 右轮(JGB37-520)
 *   STBY=PB3（必须高电平，否则整片不工作）
 *
 * 频率计算（为什么 20kHz）：
 *   TIM3 时钟 = APB1×2 = 84MHz；PSC=83 -> 计数频率 1MHz；
 *   ARR=49 -> PWM 频率 = 1MHz/50 = 20kHz（听不见的超声频段，无啸叫）。
 *   CCR 值 0~ARR+1(=50) 是占空比 0~100%；驱动的速度域固定 ±1000，
 *   由 speed_to_ccr() 按 TIM3->ARR 现场换算（不硬编码 49，改 ARR 也不会饱和）。
 *
 * 斜坡限幅 RAMP_STEP=80/20ms：速度不突变，防齿轮冲击和电流尖峰。
 */
#include "motor_driver.h"

/* ===== 内部状态 ===== */
static sint16       g_current_speed = 0;    /* 当前实际输出速度（±1000 域），斜坡每拍向目标逼近 */
static MotorState_t g_state = MOTOR_STATE_STOP;   /* 对外可查的电机状态（停止/正转/反转/急刹） */
static sint16       g_ramp_target = 0;      /* 最近一次设定的目标速度（±1000 域） */

/* ===== 初始化 ===== */
/**
 * @brief  电机驱动初始化：STBY 拉高（整片使能）+ 四个方向脚配推挽输出 + 占空比清零
 * @return 无
 * @note   ① STBY=PB3 必须为高，否则 TB6612 整片待机、电机完全不响应（还需
 *           Platform_EarlyInit() 先关 JTAG 释放 PB3，否则该脚被 JTAG 占用）；
 *         ② PWM 通道（TIM3）由 main.c 的 MX_TIM3_PWM_Init() 配置并启动，本函数只写 CCR=0；
 *         ③ 上电即“停止/滑行”态（IN1=IN2=L），不会自己动。
 */
void Motor_Init(void)
{
    /* STBY 拉高（整片使能） */
    gpio_enable_clock(GPIOB);                        /* TB6612 的方向脚都挂在 GPIOB */
    gpio_mode(GPIOB, GPIO_PIN_3, GPIO_MODE_OUTPUT);  /* PB3=STBY 设为推挽输出 */
    gpio_write(GPIOB, GPIO_PIN_3, 1);                /* STBY=1：必须为高，否则整片待机、电机不动 */

    /* AIN1/AIN2/BIN1/BIN2 输出 */
    gpio_mode(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_MODE_OUTPUT);
    /* 一次把 4 个方向脚都配成推挽输出：PB0=AIN1, PB1=AIN2, PB4=BIN1, PB5=BIN2 */

    /* TIM3 双通道 PWM 输出（由 main.c 的 MX_TIM3_PWM_Init 启动） */
    TIM3->CCR1 = 0;                       /* 左轮占空比先置 0（上电别让轮子窜） */
    TIM3->CCR2 = 0;                       /* 右轮占空比先置 0 */
    g_current_speed = 0;                  /* 软件状态同步归零 */
    g_state = MOTOR_STATE_STOP;
    g_ramp_target = 0;
}

/* ===== 斜坡限幅（20ms 步进，防冲击） ===== */
#define RAMP_STEP 80
/**
 * @brief  斜坡限幅：把 current 朝 target 推进一小步（每调用一次只走 RAMP_STEP）
 * @param  target  目标速度，±1000（正=前进 / 负=后退 / 0=停）
 * @param  current 当前输出速度，±1000（上一拍的返回值）
 * @return 这一拍应该输出的速度（已朝 target 靠近，且不会越过 target）
 * @note   步长 RAMP_STEP=80：Motor_SetSpeed() 每 20ms 调用一次 → 变化率
 *         4000 单位/秒（±1000 满速约 0.25s 到 0），这就是"软启动/软减速"的来源。
 *         纯函数、无状态：调用方必须把返回值存回 g_current_speed，否则斜坡无效。
 */
static sint16 ramp_apply(sint16 target, sint16 current)
{
    if (target > current) {               /* 目标比当前大：往上爬 */
        current += RAMP_STEP;             /* 每调用一次（约 20ms）只走 RAMP_STEP=80 步长 */
        if (current > target) current = target;   /* 别越过目标（避免在目标附近来回跳） */
    } else if (target < current) {        /* 目标比当前小：往下退（减速/反向同理） */
        current -= RAMP_STEP;
        if (current < target) current = target;   /* 同样不许越过目标 */
    }
    return current;                       /* 返回"这一拍应该输出的速度" */
}

/* ===== 速度域(±1000) → CCR 域(0..ARR+1) 换算 =====
 * 历史缺陷（tools/timing_audit.py 契约项 6）：±1000 域速度曾被直写 TIM3->CCR，
 * 而 ARR=49 → 任何 |speed| ≥ 50 都饱和成 100% 占空比（编译 0 警告，
 * 电机行为与 RAMP_STEP 斜坡限幅全部失效）。此处按 ARR 现场换算：
 *   CCR = |speed| × (ARR+1) / 1000，其中 ARR+1 对应 100% 占空比。 */
/**
 * @brief  速度域(±1000) → CCR 域(0 ~ ARR+1) 线性换算
 * @param  speed 速度值，±1000；符号被忽略（方向由 IN1/IN2 决定），
 *               超过 ±1000 先夹到 ±1000
 * @return 写进 TIM3->CCR 的计数值：0 ~ ARR+1（ARR+1 对应 100% 占空比）
 * @note   按 ARR **现场**换算，所以改 PWM 频率（ARR）不会让占空比失准。
 *         历史缺陷：此前的 ±1000 域速度被直写 CCR，而 ARR=49 → |speed|≥50
 *         就饱和成 100%，斜坡与调速全部失效（tools/timing_audit.py 契约项 6）。
 */
static uint32 speed_to_ccr(sint16 speed)
{
    uint32 arr = (uint32)TIM3->ARR + 1u;   /* ARR+1 = 一个完整 PWM 周期的计数个数（ARR=49 → 50，对应 100% 占空比） */
    uint32 mag = (speed < 0) ? (uint32)(-speed) : (uint32)speed;   /* 取绝对值：方向由 IN1/IN2 决定，CCR 只管大小 */

    if (mag > 1000u) mag = 1000u;          /* 限幅到速度域上限 ±1000 */
    return (mag * arr) / 1000u;            /* 线性映射：|speed|=1000 → CCR=arr（满占空）；|speed|=500 → 半占空 */
}

/* ===== 速度控制（两轮同速） ===== */
/**
 * @brief  设置两轮同速（带斜坡限幅的软启动/软减速）
 * @param  speed 目标速度，±1000（正=前进 / 负=后退 / 0=停止即滑行）；越界自动夹到 ±1000
 * @return 无
 * @note   ① 斜坡：每调用一次只朝目标走 RAMP_STEP=80 → **每 20ms 调一次**才有
 *           ±1000 约 0.25s 到位的软启动效果；单次调用不会立刻到目标；
 *         ② speed=0 走“滑行停止”（IN1=IN2=L），不是短路制动；要强行停住请用
 *           Motor_EmergencyBrake()；
 *         ③ 会刷新 g_ramp_target/g_current_speed 与 g_state，供 Motor_GetSpeed/
 *           Motor_GetState 查询。
 */
void Motor_SetSpeed(sint16 speed)
{
    if (speed > 1000)  speed = 1000;       /* 输入限幅：上层若传超范围值，这里夹住（保护齿轮与电流） */
    if (speed < -1000) speed = -1000;

    g_ramp_target = speed;                                  /* 记录目标值（供下一拍继续斜坡逼近） */
    g_current_speed = ramp_apply(g_ramp_target, g_current_speed);   /* 斜坡：这一拍真正执行的速度 */

    if (g_current_speed == 0) {
        /* 目标速度为 0：IN1=IN2=L 是"停止/滑行"（电机两端悬空），与短路制动区分开 */
        gpio_write(GPIOB, GPIO_PIN_0, 0);   /* AIN1=L ┐ */
        gpio_write(GPIOB, GPIO_PIN_1, 0);   /* AIN2=L ┘ 左轮：停止（滑行） */
        gpio_write(GPIOB, GPIO_PIN_4, 0);   /* BIN1=L ┐ */
        gpio_write(GPIOB, GPIO_PIN_5, 0);   /* BIN2=L ┘ 右轮：停止（滑行） */
        TIM3->CCR1 = 0;                     /* 左轮占空比 0% */
        TIM3->CCR2 = 0;                     /* 右轮占空比 0% */
        g_state = MOTOR_STATE_STOP;         /* 状态机记录：停止 */
    } else if (g_current_speed > 0) {
        /* 前进：AIN1=BIN1=H, AIN2=BIN2=L */
        gpio_write(GPIOB, GPIO_PIN_0, 1);   /* AIN1=H ┐ 左轮正转 */
        gpio_write(GPIOB, GPIO_PIN_1, 0);   /* AIN2=L ┘ */
        gpio_write(GPIOB, GPIO_PIN_4, 1);   /* BIN1=H ┐ 右轮正转 */
        gpio_write(GPIOB, GPIO_PIN_5, 0);   /* BIN2=L ┘ */
        TIM3->CCR1 = speed_to_ccr(g_current_speed);   /* 左轮占空比 ← 速度换算 */
        TIM3->CCR2 = speed_to_ccr(g_current_speed);   /* 右轮同速（阿克曼：两轮等速直行） */
        g_state = MOTOR_STATE_FORWARD;
    } else {
        /* 后退 */
        gpio_write(GPIOB, GPIO_PIN_0, 0);   /* AIN1=L ┐ 左轮反转 */
        gpio_write(GPIOB, GPIO_PIN_1, 1);   /* AIN2=H ┘ */
        gpio_write(GPIOB, GPIO_PIN_4, 0);   /* BIN1=L ┐ 右轮反转 */
        gpio_write(GPIOB, GPIO_PIN_5, 1);   /* BIN2=H ┘ */
        TIM3->CCR1 = speed_to_ccr(g_current_speed);   /* 反向时 CCR 仍写绝对值（speed_to_ccr 内部取模） */
        TIM3->CCR2 = speed_to_ccr(g_current_speed);
        g_state = MOTOR_STATE_REVERSE;
    }
}

/* ===== 左右轮独立设置 ===== */
/**
 * @brief  左右轮分别设速（差速转向/原地转圈用，立即生效）
 * @param  left  左轮速度，±1000（正=前进 / 负=后退 / 0=滑行停止）
 * @param  right 右轮速度，±1000（同上）
 * @return 无
 * @note   ① **没有斜坡**：CCR 与方向脚立刻改变（急转要求响应快），所以不要用它做
 *           常规加速，避免齿轮冲击与电流尖峰；
 *         ② 不更新 g_ramp_target/g_current_speed（那是 Motor_SetSpeed 的斜坡状态），
 *           只更新 g_state 供状态查询；
 *         ③ 状态归类：两轮都 0 → STOP；任一为正 → FORWARD；否则 REVERSE。
 */
void Motor_SetSpeedLR(sint16 left, sint16 right)
{
    /* 左轮 A 通道 */
    if (left == 0) {
        gpio_write(GPIOB, GPIO_PIN_0, 0);          /* AIN1=L ┐ */
        gpio_write(GPIOB, GPIO_PIN_1, 0);          /* AIN2=L ┘ 停止/滑行 */
        TIM3->CCR1 = 0;                            /* 占空比 0% */
    } else if (left > 0) {
        gpio_write(GPIOB, GPIO_PIN_0, 1);          /* AIN1=H ┐ 正转 */
        gpio_write(GPIOB, GPIO_PIN_1, 0);          /* AIN2=L ┘ */
        TIM3->CCR1 = speed_to_ccr(left);           /* 按 |left| 换算占空比 */
    } else {
        gpio_write(GPIOB, GPIO_PIN_0, 0);          /* AIN1=L ┐ 反转 */
        gpio_write(GPIOB, GPIO_PIN_1, 1);          /* AIN2=H ┘ */
        TIM3->CCR1 = speed_to_ccr(left);           /* speed_to_ccr 内部取绝对值 */
    }

    /* 右轮 B 通道 */
    if (right == 0) {
        gpio_write(GPIOB, GPIO_PIN_4, 0);          /* BIN1=L ┐ */
        gpio_write(GPIOB, GPIO_PIN_5, 0);          /* BIN2=L ┘ 停止/滑行 */
        TIM3->CCR2 = 0;
    } else if (right > 0) {
        gpio_write(GPIOB, GPIO_PIN_4, 1);          /* BIN1=H ┐ 正转 */
        gpio_write(GPIOB, GPIO_PIN_5, 0);          /* BIN2=L ┘ */
        TIM3->CCR2 = speed_to_ccr(right);
    } else {
        gpio_write(GPIOB, GPIO_PIN_4, 0);          /* BIN1=L ┐ 反转 */
        gpio_write(GPIOB, GPIO_PIN_5, 1);          /* BIN2=H ┘ */
        TIM3->CCR2 = speed_to_ccr(right);
    }

    g_state = (left == 0 && right == 0) ? MOTOR_STATE_STOP :
              ((left > 0 || right > 0) ? MOTOR_STATE_FORWARD : MOTOR_STATE_REVERSE);
    /* 差速时的状态归类：两轮都为 0 → 停止；任一为正 → 记为前进；否则记为反转（仅用于状态查询/日志） */
}

/**
 * @brief  查询当前**实际输出**速度
 * @return 斜坡后的输出速度，±1000（不是最后一次设定的目标值）
 * @note   Motor_SetSpeed(1000) 后立刻读会得到 80（首拍斜坡步长）。判断“是否已减速
 *         到位（==0）”时用本接口最合适。
 */
sint16 Motor_GetSpeed(void)
{
    return g_current_speed;      /* 返回斜坡后的实际输出速度（不是 g_ramp_target 目标值） */
}

/* ===== 急刹（短路制动：IN1=IN2=H） ===== */
/**
 * @brief  急刹（短路制动）：四个方向脚全置 H，两轮绕组短路，同时 PWM 置 0
 * @return 无
 * @note   ① 最强制动方式，用于 AEB 紧急制动（0x123 等级 3）/ heartbeat_v1 cmd=03；
 *         ② 会把 g_current_speed 与 g_ramp_target 直接清 0 → 解除后再起步必须重新
 *           爬斜坡（不会“瞬间恢复原速”，这是刻意的安全设计）；
 *         ③ 解除用 Motor_ReleaseBrake()（回到滑行）或再次 Motor_SetSpeed()（直接驱动）。
 */
void Motor_EmergencyBrake(void)
{
    gpio_write(GPIOB, GPIO_PIN_0, 1);    /* AIN1=H ┐ 左轮：IN1=IN2=H → 电机两端短接 */
    gpio_write(GPIOB, GPIO_PIN_1, 1);    /* AIN2=H ┘ 反电动势被短路 → 强行刹停 */
    gpio_write(GPIOB, GPIO_PIN_4, 1);    /* BIN1=H ┐ 右轮同上 */
    gpio_write(GPIOB, GPIO_PIN_5, 1);    /* BIN2=H ┘ */
    TIM3->CCR1 = 0;                      /* 占空比 0%：短路制动不需要 PWM 输出 */
    TIM3->CCR2 = 0;
    g_state = MOTOR_STATE_BRAKE;
    g_current_speed = 0;                 /* 软件速度直接归零：刹停后再起步必须重新爬斜坡 */
    g_ramp_target = 0;
}

/**
 * @brief  解除短路制动，回到“停止/滑行”态（IN1=IN2=L，PWM=0）
 * @return 无
 * @note   ① **幂等**：当前状态不是 MOTOR_STATE_BRAKE 时直接返回，不做任何硬件动作；
 *         ② 历史缺陷记录：早期实现只改 g_state、不清 IN1/IN2，硬件仍短路 → “假解除”；
 *           本实现显式拉低四个方向脚，状态与硬件保持一致；
 *         ③ 解除后速度与目标均为 0，g_state = MOTOR_STATE_STOP。
 */
void Motor_ReleaseBrake(void)
{
    /* 短路制动 → 滑行。历史实现只改 g_state、不清 IN1/IN2，硬件上仍处于
     * 短路制动（"假解除"）。这里真正回到停止态：IN1=IN2=L → 电机悬空滑行。 */
    if (g_state != MOTOR_STATE_BRAKE) return;   /* 本来就没在制动 → 什么都不做（幂等） */

    gpio_write(GPIOB, GPIO_PIN_0, 0);    /* AIN1=L ┐ 左轮：解除短路，改为滑行 */
    gpio_write(GPIOB, GPIO_PIN_1, 0);    /* AIN2=L ┘ */
    gpio_write(GPIOB, GPIO_PIN_4, 0);    /* BIN1=L ┐ 右轮同上 */
    gpio_write(GPIOB, GPIO_PIN_5, 0);    /* BIN2=L ┘ */
    TIM3->CCR1 = 0;                      /* 占空比仍为 0（保持不驱动） */
    TIM3->CCR2 = 0;
    g_current_speed = 0;
    g_ramp_target = 0;
    g_state = MOTOR_STATE_STOP;          /* 状态回到停止（不是前进/反转） */
}

/**
 * @brief  查询电机状态
 * @return MOTOR_STATE_STOP（停止/滑行）/ FORWARD / REVERSE / BRAKE（短路制动）
 * @note   只反映“最后一次下发命令的结果”，不读硬件、不感知外部推车；
 *         差速时按“任一为正即 FORWARD”归类，仅用于日志与状态机判断。
 */
MotorState_t Motor_GetState(void)
{
    return g_state;                      /* 供上层判断当前是停止/正转/反转/急刹 */
}

/* ===== 自检（只读检查，不转电机） ===== */
/**
 * @brief  电机功率级自检：检查 TIM3 是否已在计数（只读，不转电机）
 * @return TRUE = TIM3 的 CEN=1（PWM 通道已就绪）；FALSE = 定时器没启动
 * @note   只验证“能出波形”，不验证 TB6612/STBY/接线；返回 FALSE 通常意味着
 *         MX_TIM3_PWM_Init() 未执行。本函数无副作用，可周期调用。
 */
boolean Motor_SelfTest(void)
{
    /* TIM3 必须已在运行 */
    if (!(TIM3->CR1 & TIM_CR1_CEN)) {    /* CEN=0：定时器没启动 → CCR 写了也不会有 PWM 波形 */
        return FALSE;
    }
    return TRUE;                         /* 定时器在计数 = PWM 通道已就绪 */
}

//demo
void demo_forward(sint16 speed) {
    /* 前进：AIN1=BIN1=H, AIN2=BIN2=L */
        gpio_write(GPIOB, GPIO_PIN_0, 1);   /* AIN1=H ┐ 左轮正转 */
        gpio_write(GPIOB, GPIO_PIN_1, 0);   /* AIN2=L ┘ */
        gpio_write(GPIOB, GPIO_PIN_4, 1);   /* BIN1=H ┐ 右轮正转 */
        gpio_write(GPIOB, GPIO_PIN_5, 0);   /* BIN2=L ┘ */
        TIM3->CCR1 = speed_to_ccr(speed);   /* 左轮占空比 ← 速度换算 */
        TIM3->CCR2 = speed_to_ccr(speed);   /* 右轮同速（阿克曼：两轮等速直行） */
        g_state = MOTOR_STATE_FORWARD;
}

void demo_reverse(sint16 speed) {
    /* 后退 */
        gpio_write(GPIOB, GPIO_PIN_0, 0);   /* AIN1=L ┐ 左轮反转 */
        gpio_write(GPIOB, GPIO_PIN_1, 1);   /* AIN2=H ┘ */
        gpio_write(GPIOB, GPIO_PIN_4, 0);   /* BIN1=L ┐ 右轮反转 */
        gpio_write(GPIOB, GPIO_PIN_5, 1);   /* BIN2=H ┘ */
        TIM3->CCR1 = speed_to_ccr(speed);   /* 反向时 CCR 仍写绝对值（speed_to_ccr 内部取模） */
        TIM3->CCR2 = speed_to_ccr(speed);
        g_state = MOTOR_STATE_REVERSE;
}
