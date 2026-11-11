#include "bsp_can.h"
#include "can.h"
#include <stdint.h>
#include <string.h>


static uint8_t id = 0;
static CANInstance *can_instance[CAN_REGISTER_MAX_CNT] = {NULL};

static void CanAddFilter(CANInstance *_instance){
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank = _instance->handle == &hcan ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

    HAL_CAN_ConfigFilter(_instance->handle, &can_filter_conf);
}

