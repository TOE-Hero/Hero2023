#ifndef __MODE_SWITCH_H
#define __MODE_SWITCH_H
#include <cmsis_os.h>
#include <main.h>
#include <math.h>
#include "bsp_can.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "motor.h"
#include "bsp_RC.h"
#include "math.h"
#include "pid.h"
#include "INS_task.h"
#include "Chassis.h"
#include "Mode_Switch.h"
/*********************** define ************************/
/*******************哪台英雄****************************/
#define	MOON	0x00
#define SUN		0x01
//设定机器人ID，用来选择哪辆车
#define	ROBOT_ID	SUN
//#define	ROBOT_ID	MOON
/****************PIT轴是哪个电机************************/
#define GM6020	1
//设定PIT轴电机
#define PIT_MOTOR	GM6020
/**************机器人各模块模式枚举类型*****************/
//云台模式枚举类型
typedef enum{
	G_NULL	      =0,//云台摩擦轮发送电流值0
	G_HALF_GYRO   =1,//陀螺仪控制YAW轴
	G_GYRO        =2,//陀螺仪控制YAW轴、PIT轴
	G_ENCODE      =3,//电机编码器控制云台
	G_FULL_VISION =4,//全自动瞄准
	G_HALF_VISION =5,//半自动瞄准
}e_gimbalMode;
//底盘模式枚举类型
typedef enum{
	C_NULL	      = 6,//底盘摩擦轮发送电流值0
	C_APART       = 7,//地盘分离
	C_FOLLOW      = 8,//地盘跟随
	C_TOP         = 9,//陀螺移动
	C_STOP        = 10,//制动
}e_chassisMode;
//摩擦轮模式枚举类型
typedef enum{
	S_NULL	      = 11,//摩擦轮发送电流值0
	S_STOP        = 12,//摩擦轮制动
	S_SWING       = 13,//摩擦轮转
}e_shootMode;
//拨弹盘模式枚举类型
typedef enum{
	T_NULL	      = 14,//拨弹盘发送电流值0
	T_STOP        = 15,//拨弹盘制动
	T_SWING       = 16,//拨弹盘运动
}e_transMode;
/**************机器人模式设置结构体*********************/
typedef struct
{
	volatile e_gimbalMode 	gimbalMode;//云台模式变量
	volatile e_chassisMode 	chassisMode;//底盘模式变量
	volatile e_shootMode  	shootMode;//摩擦轮模式变量
	volatile e_transMode 	transMode;//拨弹盘模式变量
	volatile int shootOnce;//拨弹盘转动一步标志位（也是发弹一发的标志位）
	volatile enum roboState
	{
		DEAD		   = 0,//机器人死亡
		INITIALIZING   = 1,//初始化
		ON_PROCESSING  = 2,//运行中
		WELL_PROCESSING= 3,//处理完毕
		DBUS_ERROR     = 4,//遥控器出错
		MOTOR_ERROR    = 5,//电机出错
		JUDGE_ERROR    = 6,//裁判系统出错
		CAP_ERROR      = 7,//电容控制板出错
		PC_ERROR       = 8,//PC出错
		NOMAL          = 9,//一切正常
	}roboState;//机器人状态

}s_robo_Mode_Setting;
/*********************** extern ***********************/
extern s_robo_Mode_Setting		robot_Mode;//机器人模式结构体
/******************** function declaration ********************/
/**
  * @brief          模式初始化，放到main.c里
  * @param[in]      none
  * @retval         none
  */
void ModeInit(void);//模式初始化函数，初始化为发送0电流模式

void ModeSwitch(void);//模式控制函数

void SetMode(s_robo_Mode_Setting* robot_mode, e_gimbalMode gimbalMode, e_chassisMode chassisMode, e_shootMode shootMode, e_transMode transMode);

#endif

