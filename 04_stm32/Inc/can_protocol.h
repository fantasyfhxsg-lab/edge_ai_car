/**
 * CAN 协议定义（AI板 ↔ STM32）
 * 波特率：500kbps
 * 所有多字节字段：小端（与x86一致，方便AI板解析）
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include <stdint.h>

/* ===== CAN ID 定义 ===== */
#define CAN_ID_CONTROL        0x100   /* AI板 → STM32：控制指令，周期20ms */
#define CAN_ID_STATUS         0x101   /* STM32 → AI板：状态反馈，周期20ms */
#define CAN_ID_HEARTBEAT_AI   0x102   /* AI板 → STM32：心跳，周期1s */
#define CAN_ID_HEARTBEAT_STM  0x103   /* STM32 → AI板：心跳，周期1s */

/* ===== 控制字（0x100 byte0） ===== */
#define CTRL_ENABLE           0x01
#define CTRL_DISABLE          0x00

/* ===== 制动指令（0x100 byte3） ===== */
#define BRAKE_NONE            0x00
#define BRAKE_EMERGENCY       0x01

/* ===== 状态字（0x101 byte0） ===== */
#define STATUS_NORMAL         0x00
#define STATUS_WARN           0x01
#define STATUS_BRAKE          0x02
#define STATUS_FAULT          0x03

/* ===== 故障码（0x101 byte3） ===== */
#define FAULT_NONE            0x00
#define FAULT_AI_HEARTBEAT    0x01   /* AI板心跳丢失 */
#define FAULT_MOTOR_LEFT      0x02   /* 左电机故障 */
#define FAULT_MOTOR_RIGHT     0x04   /* 右电机故障 */
#define FAULT_LOW_BATTERY     0x08   /* 低电压 */
#define FAULT_ENCODER         0x10   /* 编码器故障 */

/* ===== 控制帧结构（0x100） =====
 * byte0: 控制字 (CTRL_ENABLE / CTRL_DISABLE)
 * byte1: 目标速度 (-100~100, 有符号)
 * byte2: 转向指令 (-100~100, 有符号，正=右转)
 * byte3: 制动指令 (BRAKE_NONE / BRAKE_EMERGENCY)
 * byte4~7: 保留
 */
typedef struct {
    uint8_t  control_word;
    int8_t   target_speed;
    int8_t   steering;
    uint8_t  brake_cmd;
    uint8_t  reserved[4];
} CanControlFrame;

/* ===== 状态帧结构（0x101） =====
 * byte0: 状态字
 * byte1: 当前速度 (有符号, -100~100)
 * byte2: 电池电压 * 10 (如 122 = 12.2V)
 * byte3: 故障码（位掩码，可叠加）
 * byte4: 左电机编码器计数高8位
 * byte5: 左电机编码器计数低8位
 * byte6: 右电机编码器计数高8位
 * byte7: 右电机编码器计数低8位
 */
typedef struct {
    uint8_t  status_word;
    int8_t   current_speed;
    uint8_t  battery_voltage_x10;
    uint8_t  fault_code;
    uint16_t encoder_left;
    uint16_t encoder_right;
} CanStatusFrame;

/* ===== 心跳帧结构（0x102 / 0x103） =====
 * byte0: 心跳计数器 (0~255循环)
 * byte1~7: 保留
 */

/* ===== 配置参数 ===== */
#define CAN_BITRATE           500000   /* 波特率 */
#define HEARTBEAT_TIMEOUT_MS  500      /* 心跳超时时间 */
#define CONTROL_PERIOD_MS     20       /* 控制周期 */

#endif /* __CAN_PROTOCOL_H */
