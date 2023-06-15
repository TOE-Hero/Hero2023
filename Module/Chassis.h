#ifndef CHASSIS_H
#define CHASSIS_H
#include <cmsis_os.h>
#include <main.h>
#include <math.h>
#include "math.h"
#include "bsp_can.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "motor.h"
#include "bsp_RC.h"
#include "pid.h"
#include "INS_task.h"
#include "Mode_Switch.h"
/*********************** 底盘信息结构体 *************************/
typedef struct
{
	//传入参数
	float gyro_speed;/*the gyro angle speed set in chassis*/
	float gyro_angle;/*the gyro angle set in chassis*/
	float angle_diff;/*the angle difference to gimbal(chassis-gimbal)*/

	/*底盘运动相关参数*/
	int16_t Vx;//目标前进速度，用作小陀螺赋值速度上了
	int16_t Vy;//目标后退速度，用作小陀螺赋值速度上了
	int16_t W;//小陀螺角速度
	
	/*功率限制闭环相关参数*/
	float chassis_power;//底盘实时功率
	float chassis_power_limit;//底盘功率限幅
	uint16_t chassis_power_buffer;//底盘缓冲能量
	uint16_t cur_sum;//输出总电流
	uint16_t cur_sum_limit;//输出总电流限幅

	/*超级电容相关参数*/
	float capVol;//超级电容输入电压
	float capInV;//超级电容电容电压
	float capInC;//超级电容输入电流
	float cap_percent;//电容剩余百分比
	float cap_expect;//超级电容设定功率

}chassisMove_t;
/***************** extern declaration *******************/
//底盘信息结构体声明
extern chassisMove_t   s_chassisMove;
/***************** function declaration *******************/
void Chassis_Init(void);
void Chassis_Move(void);

#endif

