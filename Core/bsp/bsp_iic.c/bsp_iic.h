#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__


#include "i2c.h"
#include "stm32f1xx_hal_i2c.h"
#include <stdint.h>

/* 移植关键参数，按实际情况修改 */
#define IIC_DEVICE_CNT 1        //板上i2c外设总数
#define MX_IIC_SLAVE_CNT 2      //最大从机数目

/* i2c的发送\接收方式 */
typedef enum{

    IIC_BLOCK_MODE = 0,     //阻塞式
    IIC_IT_MODE,            //中断式
    IIC_DMA_MODE,           //DMA

} IIC_Work_Mode_e;

/* i2c实例结构体 */
typedef struct iic_temp_s
{
    I2C_HandleTypeDef *handle;  //i2c handle
    uint8_t iic_address;        //器件通信地址（8位）

    IIC_Work_Mode_e workmode;   //发送接收模式
    uint8_t *rx_buffer;         //接收缓冲区指针
    uint8_t rx_len;             //接收长度
    void (*callback)(struct iic_temp_s *);//接收完成回调函数

    void *id; //实例标识
} IICInstance;

/* i2c初始化参数配置结构体 */
typedef struct
{
    I2C_HandleTypeDef *handle;  //i2c handle                
    uint8_t iic_address;        //器件通信地址（8位）      

    IIC_Work_Mode_e workmode;   //发送接收模式
    void (*callback)(struct iic_temp_s *);//接收完成回调函数

    void *id; //实例标识
} IIC_Init_Config_s;


IICInstance *IICRegieter(IIC_Init_Config_s *conf);
void IIC_Transmit(IICInstance *iic, uint8_t *pData, uint16_t Size);
void IIC_Receive(IICInstance *iic, uint8_t *pData, uint16_t Size);


#endif