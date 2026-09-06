/**
 * can_driver.h - CAN驱动抽象层（CAN Driver Abstraction）
 *
 * 作用：封装 CAN 所有硬件操作（初始化、发送、接收回调注册），
 *       业务逻辑只调用 Send() 和注册接收回调，不直接碰 HAL_CAN_*。
 *
 * 对应 AUTOSAR 层级：
 *   - 接口定义  → CAN Interface (CanIf)
 *   - 内部实现  → CAN Driver (CanDrv, MCAL层)
 *
 * 后期迁移 AUTOSAR 时：
 *   - Can_Send()    → CanIf_Transmit()
 *   - 接收回调      → CanIf_RxIndication()
 *   - 业务层代码完全不用改
 */
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "platform.h"
#include "can_protocol.h"

/* ===== CAN 配置 ===== */
#define CAN_BITRATE_BPS    500000u   /* CAN波特率 500kbps */
#define CAN_MAX_DATA_LEN   8u         /* CAN数据场最大长度 */

/* ===== 接收回调函数类型 ===== */
/**
 * @brief CAN 接收回调函数类型
 * @param can_id  CAN ID（标准帧 11位）
 * @param data    数据指针
 * @param len     数据长度 0~8
 * @note  此函数在 CAN 接收中断中调用，执行要快！
 *        对应 AUTOSAR 的 CanIf_RxIndication()
 */
typedef void (*Can_RxCallback_t)(uint16 can_id, const uint8* data, uint8 len);

/* ===== 初始化 ===== */
/**
 * @brief 初始化 CAN 控制器（波特率、过滤器、中断）
 * @return TRUE=成功, FALSE=失败
 */
boolean Can_Init(void);

/* ===== 发送 ===== */
/**
 * @brief 发送一帧 CAN 数据
 * @param can_id  CAN ID（标准帧）
 * @param data    数据指针
 * @param len     数据长度 0~8
 * @return TRUE=发送成功, FALSE=发送失败（邮箱满等）
 * @note  对应 AUTOSAR 的 CanIf_Transmit()
 */
boolean Can_Send(uint16 can_id, const uint8* data, uint8 len);

/* ===== 接收回调注册 ===== */
/**
 * @brief 注册 CAN 接收回调函数
 * @param callback 回调函数指针，NULL=取消注册
 * @note  收到任何 CAN 帧都会调用此回调
 *        回调内部根据 can_id 分发处理
 */
void Can_RegisterRxCallback(Can_RxCallback_t callback);

/* ===== 状态查询 ===== */
/**
 * @brief 获取 CAN 总线状态
 * @return TRUE=总线正常, FALSE=总线错误（被动/离线）
 */
boolean Can_IsBusOff(void);

/**
 * @brief 获取发送错误计数
 * @return TEC (Transmit Error Counter) 0~255
 */
uint8 Can_GetTxErrorCount(void);

/**
 * @brief 获取接收错误计数
 * @return REC (Receive Error Counter) 0~255
 */
uint8 Can_GetRxErrorCount(void);

/* ===== 便捷发送函数（对应协议定义的各帧） ===== */
/**
 * @brief 发送状态反馈帧（0x101）
 * @param status_word   状态字
 * @param current_speed 当前速度
 * @param voltage_x10   电池电压*10
 * @param fault_code    故障码
 */
void Can_SendStatus(uint8 status_word, sint8 current_speed, uint8 voltage_x10, uint8 fault_code);

/**
 * @brief 发送心跳帧（0x103）
 * @param counter 心跳计数器 0~255
 */
void Can_SendHeartbeat(uint8 counter);

#endif /* CAN_DRIVER_H */
