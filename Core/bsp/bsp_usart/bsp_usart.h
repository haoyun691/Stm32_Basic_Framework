#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"
#include <stdint.h>

#define USART_DEVICE_CNT 3
#define USART_RXBUFFER_LIMIT 256

typedef enum{
    USART_BLOCK_MODE = 0,
    USART_DMA_MODE,
    USART_IT_MODE,
} USART_MODE_e;

typedef struct usart_temp_s {
    
    UART_HandleTypeDef *usart_handle;

    uint8_t rx_buffer[USART_RXBUFFER_LIMIT];
    uint8_t rx_buffer_size;
    
    void (*callback)(struct usart_temp_s *);
} USARTInstance;

typedef struct {
    
    UART_HandleTypeDef *usart_handle;

    uint8_t rx_buffer_size;
    
    void (*callback)(USARTInstance *);
} USART_Init_Config_s;

//注册串口
USARTInstance *USARTRegister(USART_Init_Config_s *conf);
//调用串口
void USARTSend(USARTInstance *usart, uint8_t *send_buf, uint16_t send_size, USART_MODE_e mode);

//重置DMA
void USART_InitRxDMA(USARTInstance* usart);
//判断串口状态
uint8_t USARTIsReady(USARTInstance *usart);

#endif