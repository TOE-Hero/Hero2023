/**
  * @file    pid.c
  * @version 0.0
  * @data    2020/10/18
  * @author  Jack
  *
  * @usage   To declare some PID algorithm function and some struct variables
  *          STM32 F1 Series
  * @note    Now we did not use the increasing mode of PID algorithm
  */
  
/********* Includes used in this file *********/
#include "pid.h"
#include "string.h"
#include <stdio.h>

/****** Functions decleared in this file ******/
/**
  * @brief  绝对式 PID 控制 
  * @param  <-- pid_data -->
  *         传入一个存储绝对式 PID 数据的结构体
  * @retval None
  */
void PID_AbsoluteMode(s_pid_absolute_t *pid_data)
{
    // 将参数取正
    if (pid_data -> Kp < 0)        pid_data -> Kp = -(pid_data -> Kp);
    if (pid_data -> Ki < 0)        pid_data -> Ki = -(pid_data -> Ki);
    if (pid_data -> Kd < 0)        pid_data -> Kd = -(pid_data -> Kd);
    if (pid_data -> Kp < 0)        pid_data -> Kp = -(pid_data -> Kp);
    if (pid_data -> IerrorLim < 0) pid_data -> IerrorLim = -(pid_data -> IerrorLim);
    // PID 各环节偏差
    pid_data -> Perror = (pid_data -> NowError);
    pid_data -> Ierror += (pid_data -> NowError);
    pid_data -> Derror = (pid_data -> NowError) - (pid_data -> LastError);
    pid_data -> LastError = (pid_data -> NowError);
    //限制积分历史偏差
	if((pid_data -> Ierror) > (pid_data -> IerrorLim))      pid_data -> Ierror =  pid_data -> IerrorLim;
	else if((pid_data -> Ierror) < (-pid_data->IerrorLim))  pid_data->Ierror =  -pid_data->IerrorLim;
	//PID各环节输出量
	pid_data->Pout = pid_data->Kp * pid_data->Perror;
	pid_data->Iout = pid_data->Ki * pid_data->Ierror;
	pid_data->Dout = pid_data->Kd * pid_data->Derror;
	//PID总输出量
	pid_data->PIDout = pid_data->Pout + pid_data->Iout + pid_data->Dout;
	//限制PID总输出量
	if(pid_data->PIDout > pid_data->PIDoutMAX)        pid_data->PIDout = pid_data->PIDoutMAX;
	else if(pid_data->PIDout < -pid_data->PIDoutMAX)  pid_data->PIDout = -pid_data->PIDoutMAX;
}

/**
  * @brief  初始化绝对式 PID 的数据参数
  * @param  <-- pid_data -->
  *         传入一个存储绝对式 PID 数据的结构体
  * @param  <-- Kp -->
  *         上一个传入结构体内的 p 参数
  * @param  <-- Ki -->
  *         上一个传入结构体内的 i 参数
  * @param  <-- Kd -->
  *         上一个传入结构体内的 d 参数
  * @param  <-- errILim -->
  *         上一个传入结构体内的 i 误差上限
  * @param  <-- MaxOutCur -->
  *         上一个传入结构体内的最大输出量
  * @retval None
  */
void pid_abs_param_init(s_pid_absolute_t *pid_data, float Kp, float Ki, float Kd, float errILim, float MaxOutCur)
{
    // 初始化结构体内数据
    memset(pid_data, 0, sizeof(s_pid_absolute_t));
    // 将传入的初始化参数值赋给结构体
	pid_data -> Kp = Kp;  
	pid_data -> Ki = Ki;
	pid_data -> Kd = Kd;
	pid_data -> IerrorLim = errILim;
	pid_data -> PIDoutMAX = MaxOutCur;
    //printf("Motor param initialized successfully.\r\n");
}
