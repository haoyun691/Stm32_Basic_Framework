#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"
#include "stm32f1xx_hal_can.h"
#include <stdint.h>

//重要参数，移植时视情况修改
#define CAN_DEVICE_CNT 1            //
#define CAN_REGISTER_MAX_CNT 16
#define CAN_FLITER_CNT 14
#define CAN_DEVICE_HANDlE

#pragma pack(1)
typedef struct can_temp_s
{
    CAN_HandleTypeDef *handle;      //can handle

    CAN_TxHeaderTypeDef TxConfig;   //发送报文配置
    uint32_t tx_id;                 //发送id
    uint32_t tx_mailbox;            //发送消息填入的邮箱
    uint8_t tx_buff[8];             //发送缓存，可通过CANSetDLC()设定，最大为8

    uint32_t rx_id;                 //接收id
    uint32_t rx_len;                //接收长度 0-8
    uint8_t rx_buff[8];             //接收缓存

    void (*callback)(struct can_temp_s *); //回调函数
    void *id;           //实例标识

} CANInstance;
#pragma pack()


typedef struct
{
    CAN_HandleTypeDef *handle;

    uint32_t tx_id;
    uint32_t rx_id;

    void (*callback)(CANInstance);
    void *id;

} CAN_Init_Config_s;





#endif