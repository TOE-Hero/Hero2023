#include <stdbool.h>
#include "bsp_remote.h"
#include "bsp_buzzer.h"
#include "stdbool.h"
#include "Hero_control.h"

s_FPS_monitor	Fps;//每个要检查FPS的地方++
s_FPS_monitor	startFps;//每秒计算FPS中间值
s_FPS_monitor	finalFps;//最终帧率计算值

extern s_robo_Mode_Setting robot_Mode;
/**
  * @brief          一次计算fps
  * @param[in]      none
  * @retval         none
  */
void start_Monitor(void)
{
	Fps.dbus = fps_remote_count;
	startFps.RF_motor			= Fps.RF_motor;
	startFps.LF_motor			= Fps.LF_motor;
	startFps.LB_motor			= Fps.LB_motor;
	startFps.RB_motor			= Fps.RB_motor;
	//startFps.board_imu= Fps.board_imu;
	startFps.yaw				= Fps.yaw;
	startFps.pitch				= Fps.pitch;
	startFps.vedio_transmssion 	= Fps.vedio_transmssion;
	startFps.dbus				= fps_remote_count;
	startFps.trans				= Fps.trans;
	startFps.fric_l				= Fps.fric_l;
	startFps.fric_r				= Fps.fric_r;
	startFps.cap_board			= Fps.cap_board;
	startFps.judge				= Fps.judge;
	startFps.pc					= Fps.pc;
	
}

/**
  * @brief          二次计算fps，在任务延时一秒后计算
  * @param[in]      none
  * @retval         none
  */
void final_Monitor(void)
{
	finalFps.RF_motor = Fps.RF_motor - startFps.RF_motor;
	finalFps.LF_motor = Fps.LF_motor - startFps.LF_motor;
	finalFps.LB_motor = Fps.LB_motor - startFps.LB_motor;
	finalFps.RB_motor = Fps.RB_motor - startFps.RB_motor;
	//finalFps.board_imu	=	Fps.board_imu - startFps.board_imu;
	finalFps.yaw      =	Fps.yaw - startFps.yaw;
	finalFps.pitch	  =	Fps.pitch - startFps.pitch;
	finalFps.vedio_transmssion = Fps.vedio_transmssion - startFps.vedio_transmssion;
	finalFps.trans    =	Fps.trans - startFps.trans;
	finalFps.fric_l   =	Fps.fric_l - startFps.fric_l;
	finalFps.fric_r   =	Fps.fric_r - startFps.fric_r;
	finalFps.dbus     = fps_remote_count - startFps.dbus;
	finalFps.cap_board=	Fps.cap_board - startFps.cap_board;
	finalFps.judge    = Fps.judge - startFps.judge;
    finalFps.pc       =	Fps.pc - startFps.pc;
	
 }
/**
  * @brief          检测机器人各总线上的最终FPS，不正常就按照类别赋值机器人状态，放到自检任务中
  * @param[in]      s_FPS_monitor* final_fps
  * @retval         none
  */
void Monitor_FPS_state(s_FPS_monitor* final_fps)
{
	//如果电机FPS小于600的话（正常1000左右），电机进入CAN频率不正常，机器人状态为MOTOR_ERROR
	if((((final_fps->RF_motor<600)||(final_fps->LF_motor<600)||(final_fps->LB_motor<600)||(final_fps->RB_motor<600)|| \
	(final_fps->yaw<600)||(final_fps->pitch<600)||(final_fps->trans<600)||(final_fps->fric_l<600)||(final_fps->fric_r<600))) )
	{
		robot_Mode.roboState = MOTOR_ERROR;
		return;
	}
	else if(final_fps->dbus<30)
	{
		robot_Mode.roboState = DBUS_ERROR;
		return;
	}
	//如果裁判系统（底盘功率上限发送帧率）帧率低于3的话（正常在8-12），机器人状态为JUDGE_ERROR
	else if(final_fps->judge<3)
	{
		robot_Mode.roboState = JUDGE_ERROR;
		return;
	}
	//如果pc帧率低于100帧（正常180帧以上），机器人状态为PC_ERROR
	else if(final_fps->pc<150)
	{
		robot_Mode.roboState = PC_ERROR;
		return;
	}
	//如果果电容控制板帧率低于3的话（正常8-17）,机器人状态为CAP_ERROR
	else if(final_fps->cap_board<3)
	{
		robot_Mode.roboState = CAP_ERROR;
		return;
	}
	else
	{
		robot_Mode.roboState = NOMAL;
		return;
	}

}
/**
  * @brief          检测机器人的状态，不正常就蜂鸣器报警，放到自检任务中，嫌吵的话直接在任务中注释
  * @param[in]      s_FPS_monitor* final_fps
  * @retval         none
  */
void buzzer_Monitor_FPS_state(s_FPS_monitor* final_fps)
{
	static uint16_t psc_motor=0;//电机蜂鸣器分频值
	static uint16_t psc_dbus=0;//遥控器DBUS蜂鸣器分频值
	static uint16_t psc_judge=0;//裁判系统蜂鸣器分频值
	static uint16_t psc_cap=0;//超级电容控制板蜂鸣器分频值
	static uint16_t psc_pc=0;//pc蜂鸣器分频值
	static uint16_t pwm_motor=0;//电机蜂鸣器重载值
	static uint16_t pwm_dbus=0;//遥控器DBUS蜂鸣器重载值
	static uint16_t pwm_judge=0;//裁判系统蜂鸣器重载值
	static uint16_t pwm_cap=0;//超级电容控制板蜂鸣器重载值
	static uint16_t pwm_pc=0;//pc蜂鸣器重载值
	switch (robot_Mode.roboState)
	{
		case DBUS_ERROR://每秒一次重音
			if(pwm_dbus==0) pwm_dbus=20000;
			else pwm_dbus=0;
			buzzer_on(psc_dbus, pwm_dbus);
			break;
		case MOTOR_ERROR://长重音
			psc_motor=0;
			pwm_motor=1000;
			buzzer_on(psc_motor, pwm_motor);
			break;
		case JUDGE_ERROR://短促较重音
			psc_judge=100;
			pwm_judge=210;
			buzzer_on(psc_judge, pwm_judge);
			break;
		case PC_ERROR://慢速轻音连两下
			psc_pc=2100;
			pwm_pc=2000;
			buzzer_on(psc_pc, pwm_pc);
			break;
		case CAP_ERROR://短促轻音
			psc_cap=210;
			pwm_cap=20000;
			buzzer_on(psc_cap, pwm_cap);
			break;
		case NOMAL:
			buzzer_on(0, 0);
			break;
		default:
			break;
	}
}

/**
  * @brief          检测机器人状态为正在运行
  * @param[in]      none
  * @retval         none
  */
uint8_t isProcessOn(void)
{
	if((finalFps.RF_motor > 990 || finalFps.LF_motor > 990 ||\
		finalFps.LB_motor > 990 || finalFps.RB_motor >990) && finalFps.dbus > 60)
		return 1;
	else 
		return 0;

}

/**
  * @brief          检测机器人状态是否初始化完成
  * @param[in]      none
  * @retval         none
  */
void isInit_Ok(void)
{
	if(robot_Mode.roboState == ON_PROCESSING)
		return;
	if(Fps.dbus < 60)
		robot_Mode.roboState = INITIALIZING;
	else
		robot_Mode.roboState = ON_PROCESSING;

}

/**
 * @brief 判断遥控器帧率（70左右，超过30为正常）是否正常，0为不正常，1为正常
 * 
 */
bool_t Is_DBUS_Ok(void)
{
	//如果DBUS遥控器帧率低于30（正常70），返回0
	if(finalFps.dbus < 30)
		return 0;
	else
		return 1;
}


