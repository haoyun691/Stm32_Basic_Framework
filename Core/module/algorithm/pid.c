#include "pid.h"
#include "stm32f103xb.h"
#include <complex.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_types.h>
#include <sys/types.h>

//PID优化算法
static void f_PID_ErrorHandle(PIDInstance *pid);
static void f_Output_Limit(PIDInstance *pid);
static void f_Derivative_Filter(PIDInstance *pid);
static void f_Derivative_On_Measurement(PIDInstance *pid);
static void f_Integral_Limit(PIDInstance *pid);
static void f_Changing_Integration_Rate(PIDInstance *pid);
static void f_Trapezoid_Integral(PIDInstance *pid);
static void f_Output_Filter(PIDInstance *pid);


/**
 *@brief 注册PID实例
 *@param conf 初始化参数结构体
 *@return PID实例的指针
*/
void PID_Init(PIDInstance *_instance, PID_Init_Config_s *conf){

    memset(_instance, 0, sizeof(PIDInstance));

    _instance->Kp = conf->Kp;
    _instance->Ki = conf->Kd;
    _instance->Kd = conf->Kd;
    _instance->DeadBand = conf->DeadBand;
    _instance->MAXout = conf->MAXout;
    _instance->Improve = conf->Improve;                     //优化环节
    _instance->Integral_limit = conf->Integral_limit;       //微分限幅
    _instance->CoefA = conf->CoefA;                         //变速积分参数A和B
    _instance->CoefB = conf->CoefB;                         //I_Term = Error * ((A - |error| + B) / A)   when B<|error|<A+B
    _instance->Output_LPF_RC = conf->Output_LPF_RC;         //输出滤波器 RC = 1 / Omega_c
    _instance->Derivative_LPF_RC = conf->Derivative_LPF_RC;
    _instance->DWT_CNT = 1;   //待加入dwt

}

/**
 *@brief PID计算
 *@param pid PID实例的指针
 *@param measure 测量值
 *@param target 目标值
 *@return 计算结果
*/
float PID_Calculate(PIDInstance *pid, float measure, float target){
    
    //堵转检测
    if (pid->Improve & PID_ErrorHandle){
        f_PID_ErrorHandle(pid);
    }

    pid->dt = 1;  //待加入dwt

    //计算误差
    pid->Measure = measure;
    pid->Target = target;
    pid->Error = pid->Target - pid->Measure;

    if (fabs(pid->Error) > pid->DeadBand) {

        //位置式pid
        pid->P_out = pid->Kp * pid->Error;
        pid->I_Term = pid->Ki * pid->Error *pid->dt;
        pid->D_out = pid->Kd * (pid->Error - pid->Last_Error) / pid->dt;

         // 梯形积分
        if (pid->Improve & PID_Trapezoid_Integral)
            f_Trapezoid_Integral(pid);
        // 变速积分
        if (pid->Improve & PID_Changing_Integration_Rate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->I_out += pid->I_Term;
        pid->Output = pid->P_out + pid->I_out + pid->D_out;

        //输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);
        
        //输出限幅
        f_Output_Limit(pid);
    }else {
        //进入死区，清空输出和积分
        pid->Output = 0;
        pid->I_Term = 0;
    }

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->D_out;
    pid->Last_Error = pid->Error;
    pid->Last_I_Term = pid->I_Term;

    return pid->Output;
}

/* -----------------------------PID优化环节的实现----------------------------------- */

//梯形积分(比直接对误差求积分更准确)
static void f_Trapezoid_Integral(PIDInstance *pid){

    pid->I_Term = pid->Ki * ((pid->Error + pid->Last_Error) / 2) * pid->dt;

}

//变速积分
static void f_Changing_Integration_Rate(PIDInstance *pid){

    if (pid->Error * pid->I_out > 0) {
        //当B<|error|<A+B
        if (fabsf(pid->Error) <= pid->CoefB) {
            return;
        }
        if (fabsf(pid->Error) <= (pid->CoefA + pid->CoefB)) {
            pid->I_Term *= (pid->CoefA - fabsf(pid->Error) + pid->CoefB) / pid->CoefA;
        }else {
            pid->I_Term = 0;
        }
    }
}

//积分限幅
static void f_Integral_Limit(PIDInstance *pid){
    static float temp_Output, temp_Iout;
    temp_Iout = pid->I_out + pid->I_Term;
    temp_Output = pid->P_out + pid->I_out + pid->D_out;
    if (fabsf(temp_Output) > pid->MAXout)
    {
        if (pid->Error * pid->I_out > 0) // 积分却还在累积
        {
            pid->I_Term = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->Integral_limit)
    {
        pid->I_Term = 0;
        pid->I_out = pid->Integral_limit;
    }
    if (temp_Iout < -pid->Integral_limit)
    {
        pid->I_Term = 0;
        pid->I_out = -pid->Integral_limit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid){
    pid->D_out = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid){
    pid->D_out = pid->D_out * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid){
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

//输出限幅
static void f_Output_Limit(PIDInstance *pid){
    if (pid->Output > pid->MAXout) {
        pid->Output = pid->MAXout;
    }
    if (pid->Output < -(pid->MAXout)) {
        pid->Output = -(pid->MAXout);
    }
}

//电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid){
    if (fabsf(pid->Output) < pid->MAXout * 0.001f || fabsf(pid->Target) < 0.0001f) {
        return;
    }
    if (fabsf(pid->Target - pid->Measure) / fabsf(pid->Target) > 0.95f) {
        pid->ERRORHandler.ERRORCount++;
    }else {
        pid->ERRORHandler.ERRORCount = 0;
    }
    //尝试超过五百次
    if (pid->ERRORHandler.ERRORCount > 500) {
        pid->ERRORHandler.ERRORType = PID_ERROR;
    }
}