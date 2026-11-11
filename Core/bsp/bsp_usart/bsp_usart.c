#include "bsp_usart.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_uart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static uint8_t id;
static USARTInstance *usart_instance[USART_DEVICE_CNT] = {NULL};

/**
 *@brief 启用DMA串口空闲中断，并关闭接收过半中断
 *
 *@param usart 模块的USART实例指针
 *
 *@note HAL库默认开启了DMA的传输过半中断从而触发HAL_UARTEx_RxEventCallback()
 *@note 一般只需要传输完成中断和串口空闲中断，因此关闭过半中断
*/
void USART_InitRxDMA(USARTInstance* usart){
    HAL_UARTEx_ReceiveToIdle_DMA(usart->usart_handle, usart->rx_buffer, usart->rx_buffer_size);
    __HAL_DMA_DISABLE_IT(usart->usart_handle->hdmarx, DMA_IT_HT);
}

/**
 *@brief 注册串口实例
 *
 *@param conf 传入初始化参数的结构体
 *
 *@return 串口实例指针
*/
USARTInstance *USARTRegister(USART_Init_Config_s *conf){
    if (id >= USART_DEVICE_CNT) 
        while (1);              //检查是否超过最大实例数

    for (uint8_t i = 0; i < id; i++) 
        if (usart_instance[i]->usart_handle == conf->usart_handle) 
            while (1);                              //防止多设备注册同一串口

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));     //初始化实例结构体内存

    instance->usart_handle = conf->usart_handle;
    instance->rx_buffer_size = conf->rx_buffer_size;
    instance->callback = conf->callback;

    usart_instance[id++] = instance;        //存储当前实例。空闲实例id自增，为下一个实例注册时使用
    return instance;
}

/**
 *@brief 检测串口发送是否空闲
 *@param usart 串口实例指针
 *
 *@return 0 忙状态 
 *@return 1 空闲
 *
 *@note 为防止短时间内调用多次DMA/IT发送，在使用这两种模式前应进行状态检测
*/
uint8_t USARTIsReady(USARTInstance *usart){
    //HAL_UART_STATE_READY    = 0x20U
    //HAL_UART_STATE_BUSY_TX  = 0x21U
    if (usart->usart_handle->gState == HAL_UART_STATE_BUSY_TX) {
        return 0;
    }else {
        return 1;
    }
}

/**
 *@brief 串口发送接口
 *
 *@param usart 串口实例指针
 *@param send_buf 将要发送的数据的指针
 *@param send_size 发送数据长度
 *@param mode 发送模式（BLOCK DAM IT）

 *@note 若时间内调用同一串口的DMA/IT发送可能会产生数据丢失/解析失败，应使用USARTIsReady()判断状态
 *@todo 添加队列发送功能
*/
void USARTSend(USARTInstance *usart, uint8_t *send_buf, uint16_t send_size, USART_MODE_e mode)
{
    switch (mode)
    {
    case USART_BLOCK_MODE:
        HAL_UART_Transmit(usart->usart_handle, send_buf, send_size, 100);
        break;
    case USART_IT_MODE:
        HAL_UART_Transmit_IT(usart->usart_handle, send_buf, send_size);
        break;
    case USART_DMA_MODE:
        HAL_UART_Transmit_DMA(usart->usart_handle, send_buf, send_size);
        break;
    default:
        while (1);//未知发送方式 TODO:添加日志功能
    }
}
 /**
 *@brief DMA/IT接收回调中断
 *@note 接收完成自动触发中断，需在初始化结构体时为.callback赋相应的处理函数名
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    for (uint8_t i = 0; i < id; i++) {
        if (huart == usart_instance[i]->usart_handle) {
            if (usart_instance[i]->callback != NULL) {
                usart_instance[i]->callback(usart_instance[i]);
                memset(usart_instance[i]->rx_buffer, 0, Size);
            }
             HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->rx_buffer, usart_instance[i]->rx_buffer_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
        }
    }
}

/**
*@brief 错误回调中断
*@note 此处设置为自动重启该串口的接收
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    for (uint8_t i = 0; i < id; i++) {
        if (huart == usart_instance[i]->usart_handle) {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->rx_buffer, usart_instance[i]->rx_buffer_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }

}
