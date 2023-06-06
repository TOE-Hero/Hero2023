#ifndef ACTION_H_
#define ACTION_H_
#include "stdio.h"
#include "main.h"
#include <stdlib.h>
#include "motor_control.h"
#include "stdbool.h"

//底盘 chassis
//extern s_motor_data_t s_chassis_motor ;


typedef struct{
	uint8_t hurt;
	uint8_t last_hart;
}HURT;

typedef struct{
	uint8_t start;
	int16_t goal_speed;
	int16_t remote_speed;
  uint8_t run;
	uint8_t turn;
	int32_t position;
	int16_t judge_fps;
	int16_t fps;
	uint8_t judge_error;
	uint8_t warn;
	HURT hurt;
	uint16_t speed;
	uint16_t remain_HP; /* 机器人剩余血量 */
	uint16_t last_HP; /* 机器人上次剩余血量 */
  uint8_t turn_flag;
	uint8_t last_flag;
	int16_t max_power;/* the maximum power expected */
  uint16_t power_buffer; /* 底盘功率缓冲 J */
}LOGIC;


extern int Logic_turn_flag;
void Chassis_SpeedSet(float YSpeed);
void Chassis_Contral(void);
void Normal_Move(void);
void Logic_Move(void);
void Judge_FPS(void);
void speed_buffer(void);

uint8_t Monitor_HP(void);

extern LOGIC Logic;

#define sensor_left  HAL_GPIO_ReadPin(sensor_left_GPIO_Port,sensor_left_Pin)
#define sensor_right HAL_GPIO_ReadPin(sensor_right_GPIO_Port,sensor_right_Pin)
#define abs_f(x) ((x)>0? (x):(-(x)))
#define MAXMOTORSPEED 		  3000  //遥控速度限幅(chassis_control)
#define chassis_Out_max 		1500  //输出电流限制
#define NORMALSPEED 1000     //自动速度限幅(chassis_control)
#define chassis_buffer_waring 30
#define chassis_speed_change 10


















#endif /* ACTION_H_ */