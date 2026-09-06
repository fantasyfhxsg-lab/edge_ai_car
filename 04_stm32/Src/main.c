/**
 * main.c - 主程序（AUTOSAR 友好架构）
 *
 * 架构分层：
 * ┌─────────────────────────────────────────────┐
 * │  应用层 (Application)                        │
 * │  - AEB状态机、速度PID、心跳监控              │
 * │  - 只调用驱动层接口，不直接碰 HAL            │
 * ├─────────────────────────────────────────────┤
 * │  任务调度 (Task Scheduler)                   │
 * │  - Task_10ms / Task_20ms / Task_100ms     │
 * │  - 对应 AUTOSAR OS Task                      │
 * ├─────────────────────────────────────────────┤
 * │  驱动抽象层 (Driver Abstraction)             │
 * │  - motor_driver / servo_driver              │
 * │  - can_driver / encoder                      │
 * │  - 对应 AUTOSAR IoHwAb + CanIf              │
 * ├─────────────────────────────────────────────┤
 * │  平台抽象层 (platform.h)                     │
 * │  - 隔离 HAL 库，后期可换 AUTOSAR MCAL       │
 * └─────────────────────────────────────────────┘
 *
 * 后期迁移 AUTOSAR 时：
 *   - Task_xxx() 函数体不变，注册成 AUTOSAR OS Task
 *   - 应用层代码不变，封装成 SWC
 *   - 只需要把驱动层内部的 HAL 调用换成 MCAL 调用
 */

/* ===== 只包含抽象层头文件，不直接包含 HAL ===== */
#include "platform.h"        /* 平台抽象：类型定义、HAL库隔离 */
#include "can_protocol.h"    /* CAN 协议定义 */
#include "motor_driver.h"    /* 电机驱动抽象 */
#include "servo_driver.h"    /* 舵机驱动抽象 */
#include "can_driver.h"      /* CAN驱动抽象 */
#include "encoder.h"         /* 编码器抽象 */

/* ===== 外部硬件句柄（由 CubeMX 生成的 stm32f4xx_hal_msp.c 定义） ===== */
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim2;   /* PWM: CH1=电机, CH2=舵机 */
TIM_HandleTypeDef htim3;   /* 编码器 */

/* ===== 应用层全局变量 ===== */

/* 从 AI 板收到的控制指令（由 CAN 接收回调更新） */
static volatile CanControlFrame g_control = {0};
static volatile boolean g_control_received = FALSE;

/* 整车状态 */
static uint8  g_status_word = STATUS_NORMAL;   /* 当前状态字 */
static uint8  g_fault_code = FAULT_NONE;       /* 故障码（位掩码） */
static sint8  g_current_speed = 0;              /* 当前速度（由编码器计算） */

/* 心跳相关 */
static uint32 g_last_ai_heartbeat_tick = 0;    /* 上次收到 AI 板心跳的时间 */
static uint8  g_stm32_heartbeat_counter = 0;    /* STM32 心跳计数器 */

/* 电机 PID 参数（需根据实际电机调试） */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.1f
static float32 g_pid_integral = 0.0f;
static float32 g_pid_last_error = 0.0f;

/* ===== 函数声明 ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* CAN 接收回调（应用层，由 can_driver 调用） */
static void App_CanRxCallback(uint16 can_id, const uint8* data, uint8 len);

/* 任务函数（对应 AUTOSAR OS Task） */
static void Task_10ms(void);    /* CAN 通信任务 */
static void Task_20ms(void);    /* 电机控制任务 */
static void Task_100ms(void);   /* 心跳监控任务 */

/* 应用层功能函数 */
static void App_ProcessControl(void);           /* 处理控制指令 */
static void App_HeartbeatMonitor(void);         /* 心跳监控 */
static sint16 App_MotorPID(sint8 target_speed); /* 电机速度PID计算 */
static void App_FaultHandler(void);             /* 故障处理 */

/* ================================================================
 *  CAN 接收回调（应用层）
 *  由 can_driver 在中断中调用，只解析数据，不做耗时操作
 *  对应 AUTOSAR：CanIf_RxIndication() → SWC Runnable
 * ================================================================ */
static void App_CanRxCallback(uint16 can_id, const uint8* data, uint8 len)
{
    switch (can_id)
    {
        case CAN_ID_CONTROL:
            /* 控制指令帧：byte0=控制字, byte1=目标速度, byte2=转向, byte3=制动 */
            if (len >= 4)
            {
                g_control.control_word = data[0];
                g_control.target_speed = (sint8)data[1];
                g_control.steering = (sint8)data[2];
                g_control.brake_cmd = data[3];
                g_control_received = TRUE;
            }
            break;

        case CAN_ID_HEARTBEAT_AI:
            /* AI 板心跳帧：记录时间戳，用于超时检测 */
            g_last_ai_heartbeat_tick = HAL_GetTick();
            break;

        default:
            /* 其他 ID 忽略 */
            break;
    }
}

/* ================================================================
 *  Task_10ms - CAN 通信任务
 *  周期：10ms
 *  职责：处理收到的控制指令 + 发送状态反馈
 *  对应 AUTOSAR OS Task（10ms 周期任务）
 * ================================================================ */
static void Task_10ms(void)
{
    /* 1. 处理控制指令（应用层逻辑） */
    App_ProcessControl();

    /* 2. 发送状态反馈帧（通过驱动层接口，不直接碰 HAL） */
    Can_SendStatus(
        g_status_word,       /* 状态字 */
        g_current_speed,     /* 当前速度 */
        122,                 /* 电池电压*10（实际用ADC读取） */
        g_fault_code         /* 故障码 */
    );
}

/* ================================================================
 *  Task_20ms - 电机控制任务
 *  周期：20ms
 *  职责：编码器测速 + PID计算 + 电机输出 + 舵机转向
 *  对应 AUTOSAR OS Task（20ms 周期任务）
 * ================================================================ */
static void Task_20ms(void)
{
    sint16 motor_pwm_output;

    /* 非故障态才执行正常控制 */
    if (g_status_word != STATUS_FAULT)
    {
        /* 1. 编码器测速（通过驱动层接口） */
        Encoder_GetDelta();
        g_current_speed = (sint8)(Encoder_GetSpeedRPM() / 10.0f);

        /* 2. 电机速度 PID 计算（应用层算法，不碰硬件） */
        motor_pwm_output = App_MotorPID(g_control.target_speed);

        /* 3. 电机输出（通过驱动层接口） */
        if (g_control.brake_cmd == BRAKE_EMERGENCY)
        {
            /* 紧急制动：调用驱动层急刹接口 */
            Motor_EmergencyBrake();
            g_status_word = STATUS_BRAKE;
        }
        else if (g_control.control_word == CTRL_ENABLE)
        {
            /* 正常控制：设置速度 + 设置转向 */
            Motor_SetSpeed(motor_pwm_output);
            Servo_SetAngle(g_control.steering);
            g_status_word = STATUS_NORMAL;
        }
        else
        {
            /* 控制未使能：电机停止 */
            Motor_SetSpeed(0);
        }
    }
    else
    {
        /* 故障态：强制停车 */
        Motor_EmergencyBrake();
    }
}

/* ================================================================
 *  Task_100ms - 心跳监控任务
 *  周期：100ms
 *  职责：AI板心跳超时检测 + 发送STM32心跳 + 故障检测
 *  对应 AUTOSAR OS Task（100ms 周期任务）
 * ================================================================ */
static void Task_100ms(void)
{
    /* 1. AI 板心跳监控（应用层逻辑） */
    App_HeartbeatMonitor();

    /* 2. 发送 STM32 心跳帧（通过驱动层接口） */
    Can_SendHeartbeat(g_stm32_heartbeat_counter);
    g_stm32_heartbeat_counter++;

    /* 3. 故障检测（应用层逻辑） */
    App_FaultHandler();
}

/* ================================================================
 *  应用层功能函数
 * ================================================================ */

/**
 * @brief 处理控制指令
 * @note  纯应用层逻辑，不碰硬件
 */
static void App_ProcessControl(void)
{
    if (g_control_received)
    {
        g_control_received = FALSE;
        /* 控制指令已在 CAN 回调中更新到 g_control，这里可以做额外处理 */
    }
}

/**
 * @brief AI 板心跳监控
 * @note  超时则进入故障态（安全侧失效：通信断了就停车）
 */
static void App_HeartbeatMonitor(void)
{
    uint32 now = HAL_GetTick();

    if ((now - g_last_ai_heartbeat_tick) > HEARTBEAT_TIMEOUT_MS)
    {
        /* 心跳超时 → 故障态 → 置故障码 */
        g_status_word = STATUS_FAULT;
        g_fault_code |= FAULT_AI_HEARTBEAT;
    }
    else if ((g_status_word == STATUS_FAULT) && (g_fault_code & FAULT_AI_HEARTBEAT))
    {
        /* 心跳恢复 → 清除故障码 */
        g_fault_code &= ~FAULT_AI_HEARTBEAT;
        if (g_fault_code == FAULT_NONE)
        {
            g_status_word = STATUS_NORMAL;
            Motor_ReleaseBrake();  /* 解除制动 */
        }
    }
}

/**
 * @brief 电机速度 PID 计算
 * @param target_speed 目标速度 -100~100
 * @return PWM 输出 -1000~1000
 * @note  纯算法，不碰硬件。输出传给 Motor_SetSpeed()
 */
static sint16 App_MotorPID(sint8 target_speed)
{
    float32 error;
    float32 derivative;
    float32 output;

    /* 误差 = 目标 - 当前 */
    error = (float32)target_speed - (float32)g_current_speed;

    /* 积分项（带限幅，防止积分饱和） */
    g_pid_integral += error;
    if (g_pid_integral > 500.0f)  g_pid_integral = 500.0f;
    if (g_pid_integral < -500.0f) g_pid_integral = -500.0f;

    /* 微分项 */
    derivative = error - g_pid_last_error;
    g_pid_last_error = error;

    /* PID 输出 */
    output = PID_KP * error + PID_KI * g_pid_integral + PID_KD * derivative;

    /* 输出限幅 */
    if (output > 1000.0f)  output = 1000.0f;
    if (output < -1000.0f) output = -1000.0f;

    return (sint16)output;
}

/**
 * @brief 故障处理
 * @note  检测各类故障，置故障码
 *        对应 AUTOSAR DEM（故障检测管理）
 */
static void App_FaultHandler(void)
{
    /* 低电压检测（实际用 ADC 读取电池电压） */
    /* if (battery_voltage < 10.0f) g_fault_code |= FAULT_LOW_BATTERY; */

    /* 编码器故障检测（速度为0但电机有输出，可能堵转） */
    /* if (Motor_GetSpeed() > 500 && g_current_speed == 0) g_fault_code |= FAULT_ENCODER; */

    /* CAN 总线错误检测 */
    if (Can_IsBusOff())
    {
        /* 总线关闭故障 */
    }
}

/* ================================================================
 *  main - 主函数
 *  职责：初始化 + 任务调度
 *  不包含任何业务逻辑，业务逻辑都在 Task_xxx() 里
 * ================================================================ */
int main(void)
{
    /* ===== 硬件初始化（CubeMX 生成） ===== */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    /* ===== 驱动层初始化 ===== */
    Motor_Init();       /* 电机驱动初始化 */
    Servo_Init();       /* 舵机驱动初始化（回中） */
    Encoder_Init();     /* 编码器初始化 */
    Can_Init();         /* CAN 驱动初始化 */

    /* ===== 注册 CAN 接收回调（应用层） ===== */
    Can_RegisterRxCallback(App_CanRxCallback);

    /* ===== 上电自检 ===== */
    Motor_SelfTest();
    Servo_SelfTest();
    Encoder_SelfTest();

    /* 初始化心跳时间 */
    g_last_ai_heartbeat_tick = HAL_GetTick();

    /* ===== 主循环：只负责任务调度 ===== */
    uint32 last_10ms = 0, last_20ms = 0, last_100ms = 0;

    while (1)
    {
        uint32 now = HAL_GetTick();

        /* 10ms 任务：CAN 通信 */
        if (now - last_10ms >= 10)
        {
            last_10ms = now;
            Task_10ms();
        }

        /* 20ms 任务：电机控制 */
        if (now - last_20ms >= 20)
        {
            last_20ms = now;
            Task_20ms();
        }

        /* 100ms 任务：心跳监控 */
        if (now - last_100ms >= 100)
        {
            last_100ms = now;
            Task_100ms();
        }
    }
}

/* ================================================================
 *  硬件初始化函数（由 STM32CubeMX 生成，这里给出框架）
 *  实际项目中用 CubeMX 配置后自动生成，不需要手写
 * ================================================================ */

static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
    HAL_CAN_Init(&hcan1);

    /* 配置过滤器：接收所有标准帧 */
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

static void MX_TIM2_Init(void)
{
    /* PWM 输出初始化（由 CubeMX 生成） */
    /* TIM2: CH1=电机PWM(PA0), CH2=舵机PWM(PA1) */
}

static void MX_TIM3_Init(void)
{
    /* 编码器模式初始化（由 CubeMX 生成） */
    /* TIM3: CH1=PA6, CH2=PA7, 编码器模式 */
}

static void MX_GPIO_Init(void)
{
    /* GPIO 初始化（由 CubeMX 生成） */
    /* PB0 = 电机方向引脚 */
}

void SystemClock_Config(void)
{
    /* 系统时钟配置（由 CubeMX 生成） */
    /* STM32F407: 168MHz */
}
