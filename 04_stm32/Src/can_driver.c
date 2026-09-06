/**
 * can_driver.c - CAN驱动实现
 *
 * 硬件连接：
 *   - CAN1_TX: PA12
 *   - CAN1_RX: PA11
 *   - CAN收发器: TJA1050 / SN65HVD230
 *   - 波特率: 500kbps
 *
 * 定时器参数（STM32F407, APB1=42MHz, CAN时钟=84MHz）：
 *   Prescaler = 6
 *   BS1 = 11TQ, BS2 = 4TQ
 *   波特率 = 84MHz / (6 * (1+11+4)) = 84MHz / 96 = 875kHz (不对，需要重新算)
 *
 * 正确计算（CAN时钟=42MHz for F407 APB1）：
 *   实际 CAN 内核时钟 = APB1 时钟 = 42MHz (F407)
 *   Prescaler=6, BS1=11, BS2=4 → 42MHz/(6*16) = 437.5kHz
 *   要 500kbps: Prescaler=5, BS1=11, BS2=4 → 42MHz/(5*16) = 525kHz
 *   实际用 CubeMX 自动计算，这里只做参考
 *
 * 后期迁移 AUTOSAR 时：
 *   - HAL_CAN_AddTxMessage() → Can_Write() (MCAL) → CanIf_Transmit()
 *   - HAL_CAN_RxFifo0MsgPendingCallback → Can_RxMsgPending() → CanIf_RxIndication()
 */
#include "can_driver.h"

/* ===== 外部硬件句柄 ===== */
extern CAN_HandleTypeDef hcan1;

/* ===== 内部状态 ===== */
static Can_RxCallback_t g_rx_callback = NULL;  /* 接收回调函数 */
static uint8 g_heartbeat_counter = 0;           /* 心跳计数器 */

/* ===== 初始化 ===== */
boolean Can_Init(void)
{
    /* 启动 CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return FALSE;
    }

    /* 使能接收中断 */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return FALSE;
    }

    return TRUE;
}

/* ===== 发送 ===== */
boolean Can_Send(uint16 can_id, const uint8* data, uint8 len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (len > CAN_MAX_DATA_LEN)
    {
        len = CAN_MAX_DATA_LEN;
    }

    /* 配置发送头 */
    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;       /* 标准帧 */
    tx_header.RTR = CAN_RTR_DATA;     /* 数据帧 */
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 发送 */
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*)data, &tx_mailbox) != HAL_OK)
    {
        return FALSE;  /* 发送失败（邮箱满等） */
    }

    return TRUE;
}

/* ===== 接收回调注册 ===== */
void Can_RegisterRxCallback(Can_RxCallback_t callback)
{
    g_rx_callback = callback;
}

/*
 * HAL CAN 接收中断回调
 * 这是 HAL 库定义的弱函数，我们在这里重写。
 * 收到数据后，调用注册的业务层回调。
 *
 * 对应 AUTOSAR：Can_RxMsgPending() (MCAL) → CanIf_RxIndication()
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1)
    {
        /* 读取接收到的消息 */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            /* 调用业务层回调（如果已注册） */
            if (g_rx_callback != NULL)
            {
                g_rx_callback((uint16)rx_header.StdId, rx_data, (uint8)rx_header.DLC);
            }
        }
    }
}

/* ===== 状态查询 ===== */
boolean Can_IsBusOff(void)
{
    /*
     * CAN 错误状态：
     *   TEC < 128 且 REC < 128 → 错误主动（正常）
     *   TEC >= 128 或 REC >= 128 → 错误被动
     *   TEC >= 256 → 总线关闭（Bus-Off）
     */
    uint32_t esr = hcan1.Instance->ESR;
    uint8 tec = (uint8)((esr >> 16) & 0xFF);
    return (tec >= 256) ? TRUE : FALSE;
}

uint8 Can_GetTxErrorCount(void)
{
    uint32_t esr = hcan1.Instance->ESR;
    return (uint8)((esr >> 16) & 0xFF);
}

uint8 Can_GetRxErrorCount(void)
{
    uint32_t esr = hcan1.Instance->ESR;
    return (uint8)((esr >> 24) & 0xFF);
}

/* ===== 便捷发送函数 ===== */
void Can_SendStatus(uint8 status_word, sint8 current_speed, uint8 voltage_x10, uint8 fault_code)
{
    uint8_t data[8];

    data[0] = status_word;
    data[1] = (uint8_t)current_speed;
    data[2] = voltage_x10;
    data[3] = fault_code;
    data[4] = 0;  /* 保留：编码器高8位 */
    data[5] = 0;  /* 保留：编码器低8位 */
    data[6] = 0;  /* 保留 */
    data[7] = 0;  /* 保留 */

    Can_Send(CAN_ID_STATUS, data, 8);
}

void Can_SendHeartbeat(uint8 counter)
{
    uint8_t data[8] = {0};
    data[0] = counter;
    Can_Send(CAN_ID_HEARTBEAT_STM, data, 8);
}
