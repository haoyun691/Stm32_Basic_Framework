/**
 * @file bsp_iic.c
 * @brief 硬件i2c的bsp层的实现
 * @version 0.1
 */

#include "bsp_iic.h"
#include "stm32f1xx_hal_i2c.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static uint8_t id = 0;
static IICInstance *iic_instance[IIC_DEVICE_CNT] = {NULL};

/**
 * @brief 注册i2c实例
 *
 * @param conf 参数配置结构体指针
 * @return IICInstance *
 */
IICInstance *IICRegieter(IIC_Init_Config_s *conf){

    if (id >= IIC_DEVICE_CNT) 
        while(1); //方便Debug定位错误。TODO:添加日志功能，输出错误日志
    IICInstance *p_instance = (IICInstance *)malloc(sizeof(IICInstance));
    memset(p_instance, 0, sizeof(IICInstance));

    p_instance->iic_address = conf->iic_address; // 使用8位地址（自行左移1位）
    p_instance->handle = conf->handle;
    p_instance->workmode = conf ->workmode;
    p_instance->callback = conf->callback;
    p_instance->id = conf->id;

    iic_instance[id++] = p_instance;

    return p_instance;
}

/**
 *@brief i2c发送数据
 *
 *@param iic iic实例
 *@param pData 要发送的数据的首地址指针
 *@param Siza 发送长度
 *@param 默认超时时间为100ms
 *
 */
void IIC_Transmit(IICInstance *iic, uint8_t *pData, uint16_t Size){

    /*选择工作模式*/
    switch (iic->workmode) {
    case IIC_BLOCK_MODE: 
        HAL_I2C_Master_Transmit(iic->handle, iic->iic_address, pData, Size, 100);
        break;
    case IIC_DMA_MODE:
        HAL_I2C_Master_Transmit_DMA(iic->handle, iic->iic_address, pData, Size);
        break;
    case IIC_IT_MODE:
        HAL_I2C_Master_Transmit_IT(iic->handle, iic->iic_address, pData, Size);
        break;
    default:
        while (1) ; //未知传输模式 TODO：添加日志功能
    }
}

/**
 *@brief i2c接收数据
 *
 *@param iic iic实例
 *@param pData 接收数据的首地址指针
 *@param Siza 接收长度
 *@param 默认超时时间为100ms
 *
 */
void IIC_Receive(IICInstance *iic, uint8_t *pData, uint16_t Size){

    /* 初始化接收缓冲区地址以及接受长度, 用于中断回调函数 */
    iic->rx_buffer = pData;
    iic->rx_len = Size;

    /* 选择工作模式 */
    switch (iic->workmode) {
    case IIC_BLOCK_MODE:
        HAL_I2C_Master_Receive(iic->handle, iic->iic_address, pData, Size, 100);
        break;
    case IIC_DMA_MODE:
        HAL_I2C_Master_Receive_DMA(iic->handle, iic->iic_address, pData, Size);
        break;
    case IIC_IT_MODE:
        HAL_I2C_Master_Receive_IT(iic->handle, iic->iic_address, pData, Size);
        break;
    default:
        while (1) ; //未知传输模式 TODO：添加日志功能
    }
}

/**
 * @brief IIC接收完成回调函数
 *
 * @param hi2c handle
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){

    for (uint8_t i = 0; i < id; i++) {
        if (iic_instance[i]->handle == hi2c && hi2c->Devaddress == iic_instance[i]->iic_address) {
            if (iic_instance[i]->callback != NULL) {
                iic_instance[i]->callback(iic_instance[i]);
            }
            return;
        }
    }
}


