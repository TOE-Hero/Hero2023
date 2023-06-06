
/****** Includes in this source file ******/
#include "motor_control.h"
#include <stdio.h>
#include "bsp_can.h"

/******** Macros used in this file ********/
// 调试电机速度环pid
#define MOTOR_SPEED_LOOP_PID_DEBUG         0
// 稳定时的设定值
#define MOTOR_SPEED_LOOP_KP             0.0f
#define MOTOR_SPEED_LOOP_KI             0.0f
#define MOTOR_SPEED_LOOP_KD             0.0f
#define MOTOR_SPEED_LOOP_IERRORLIM   4000.0f
#define MOTOR_SPEED_LOOP_PIDOUTMAX  30000.0f

/********* Datas used in this file ********/
// 存储电机数据
s_motor_data_t s_chassis_motor = {CHASSIS_ID};
// 电机使用绝对式 PID 的算法数据
s_pid_absolute_t speed_loop_pid = {0};
s_pid_absolute_t position_loop_pid={0};
/******* Functions decleared in this file ********/
/**
  * @brief  电机参数初始化，给 PID 数据结构体赋初始值 0 0 0
  * @param  None
  * @retval None
  */
void motor_param_init(void)
{
    //pid_abs_param_init(&position_loop_pid, 0, 0, 0, MOTOR_SPEED_LOOP_IERRORLIM, MOTOR_SPEED_LOOP_PIDOUTMAX);
	  pid_abs_param_init(&speed_loop_pid, 14, 0, 0, MOTOR_SPEED_LOOP_IERRORLIM, MOTOR_SPEED_LOOP_PIDOUTMAX);
    printf("Motor initialized successfully.\r\n");
}

/**
  * @brief  电机参数经计算后设定，给 PID 数据结构体赋从串口来的设定值
  * @param  None
  * @retval None
  */
void motor_speed_pid_reset(void)
{
    if (MOTOR_SPEED_LOOP_PID_DEBUG)
    {
			
        //pid_abs_param_init(&position_loop_pid, P1, I1, D1, MOTOR_SPEED_LOOP_IERRORLIM, MOTOR_SPEED_LOOP_PIDOUTMAX);
			  pid_abs_param_init(&speed_loop_pid, P1, I1, D1, MOTOR_SPEED_LOOP_IERRORLIM, MOTOR_SPEED_LOOP_PIDOUTMAX);
			 // printf("%f%f%f\r\n",P1,I1,D1);
    }
}
/**
  * @brief  速度环一些参数的计算处理
  *         将上位机得到的目标速度值赋给电机数据结构体并计算此时的速度偏差
  *         计算 PID 算法中各个环节的量并确定电机输出
  * @param  None
  * @retval None
  */

void motor_speed_pid_control(void)
{
	  //计算速度偏差
	  speed_loop_pid.NowError=s_chassis_motor.target_speed - s_chassis_motor.back_speed;
    // 计算 PID 算法中各个环节的量
	  PID_AbsoluteMode(&speed_loop_pid);
    // 确定电机输出电流
	  s_chassis_motor.out_current = speed_loop_pid.PIDout;
}
/**
 * @brief 底盘功率限制函数
 * @param None
 * @return None
 * @attention None
 */
void calculate_power_param(void)
	{ 
		//取绝对值
		int out_current = s_chassis_motor.out_current > 0 ? 	s_chassis_motor.out_current : -s_chassis_motor.out_current ;
	  if(Logic.power_buffer > 40)
	  s_chassis_motor.current_maxOut = 30000;
	  else 
		s_chassis_motor.current_maxOut = Logic.power_buffer * Logic.power_buffer * 16;//算出最大输出电流，需要实践
	  if(out_current > s_chassis_motor.current_maxOut)
		{
				float scale = (float)s_chassis_motor.current_maxOut / (float)out_current;
			  s_chassis_motor.out_current *= scale;
		}
}
	








