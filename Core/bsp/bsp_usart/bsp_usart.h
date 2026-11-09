#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32f1xx_hal_uart.h"
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
    
    void (*usart_callback)(struct usart_temp_s *);
} USARTInstance;

#endif