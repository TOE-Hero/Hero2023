#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f4xx_hal.h"
#include "STMGood.h"
#include "pid.h"


/************ 电机数据结构体 ************/
typedef struct 
{
	int     id;
	uint8_t is_pos_ready;
	float   gyro_speed;
	float   gyro_angle;
	int16_t anglespd_times_ago;
	int     reduction_ratio;
	int     frequency;
	int16_t mid_pos;
	int16_t max_pos;
	int16_t min_pos;
	int16_t temperature;
	int16_t back_speed;
	int16_t back_position;
	int16_t back_pos_last;
	int64_t circle_num;
	float   target_speed;
	int     real_spd;
	int64_t tol_pos;
	double  target_pos;
	float   target_ang;
	int16_t out_current;
	int16_t back_current;
	uint32_t	current_maxOut;

}s_motor_data_t ;
extern s_motor_data_t s_chassis_motor;
extern s_pid_absolute_t position_loop_pid;
extern s_pid_absolute_t speed_loop_pid;
void motor_param_init(void);
void motor_speed_pid_control(void);
void motor_speed_pid_reset(void);
void calculate_power_param(void);


#define CHASSIS_ID  0x201


#endif  /* MOTOR_CONTROL_H */
