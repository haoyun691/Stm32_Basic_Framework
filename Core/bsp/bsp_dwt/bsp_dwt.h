#ifndef __BSP_DWT_H__
#define __BSP_DWT_H__

#include "stm32f1xx_hal.h"



void DWT_Init(void);
float DWT_GetDeltaT(uint32_t *cnt_last);
void DWT_Delay_us(uint32_t us);

#endif