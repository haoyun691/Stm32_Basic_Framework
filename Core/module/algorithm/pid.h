#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>


//PID优化环节使能标志位
typedef enum
{
    PID_IMPROVE_NONE                    = 0b00000000,       //无优化

    PID_Integral_Limit                  = 0b00000001,       //积分限幅

    PID_Derivative_On_Measurement       = 0b00000010,       //微分先行

    PID_Trapezoid_Integral              = 0b00000100,       //梯形积分

    PID_Proportional_On_Measurement     = 0b00001000,       //比例 P

    PID_OutputFilter                    = 0b00010000,       //输出滤波

    PID_Changing_Integration_Rate       =0b001000000,       //变速积分

    PID_DerivativeFilter                = 0b01000000,       //微分滤波

    PID_ErrorHandle                     = 0b10000000,       //报错

} PID_Improvement_e;

//PID报错类型枚举
typedef enum
{
    PID_ERROR_NONE = 0x00U,
    PID_ERROR = 0x01U,
} ErrorType_e;

//PID报错记录结构体
typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_s;

//PID实例结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MAXout;
    float DeadBand;

    PID_Improvement_e Improve;
    float Integral_limit;       //微分限幅
    float CoefA;                //变速积分参数A和B
    float CoefB;                //I_Term = Error * ((A - |error| + B) / A)   when B<|error|<A+B
    float Output_LPF_RC;        //输出滤波器 RC = 1 / Omega_c
    float Derivative_LPF_RC;    //微分滤波器系数


    float Measure;
    float Last_Measure;
    float Error;
    float Last_Error;
    float I_Term;
    float Last_I_Term;

    float P_out;                //P输出
    float I_out;                //I输出
    float D_out;                //D输出
    float Last_Dout;            //上次D输出

    float Output;               //输出
    float Last_Output;          //上次输出
    
    float Target;               //设定值/目标值

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_s ERRORHandler;

} PIDInstance;

//PID初始化配置结构体
typedef struct
{
    float Kp;      
    float Ki;      
    float Kd;      
    float MAXout;       //输出限幅
    float DeadBand;     //死区

    PID_Improvement_e Improve;
    float Integral_limit;       //微分限幅
    float CoefA;                //变速积分参数A和B
    float CoefB;                //I_Term = Error * ((A - |error| + B) / A)   when B<|error|<A+B
    float Output_LPF_RC;        //输出滤波器 RC = 1 / Omega_c
    float Derivative_LPF_RC;    //微分滤波器系数

} PID_Init_Config_s;


void PID_Init(PIDInstance *_instance, PID_Init_Config_s *conf);
float PID_Calculate(PIDInstance *pid, float measure, float target);

#endif