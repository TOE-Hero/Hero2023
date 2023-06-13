#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"

#define REDUCTION_RATIO_3508_1_19  	19.203208556f//拨弹电机(3508)1:19减速比
#define REDUCTION_RATIO_3508_1_27  	26.851f//拨弹电机(3508)1:27减速比
#define REDUCTION_RATIO_3508_1_51  	50.895f//拨弹电机(3508)1:51减速比

#define MOTOR_ONE_TURN_SCALE_3508	8191.0f//3508电机一圈的刻度
#define MOTOR_ONE_TURN_SCALE_6020	8191.0f//6020电机一圈的刻度
#define MOTOR_ONE_TURN_SCALE_2006	8191.0f//2006电机一圈的刻度

typedef struct
{
	/**电机基础信息**/
    int     ID;//电机所连电调CAN通信ID
	int16_t temperature;//电机温度
	/**连续圈数变量**/
	int64_t circle_num;//电机连续编码圈数
    uint8_t is_pos_ready;//电机上电时目标位置为返回的绝对编码值，只用在了连续编码函数里
	/**刻度转角度**/
	float	back_motor_ang;//电机当前编码器转换成角度（刻度转角度）
	float   serial_motor_ang;//电机连续编码转换成角度（刻度转角度）
	double	target_motor_ang;//电机目标角度（用在PID位置环）（刻度转角度）
	/**刻度**/
	int64_t serial_position;//电机连续编码值（刻度）
	int16_t back_position;//电机返回的编码器值（刻度）
	int16_t back_pos_last;//电机连续编码上一次值，只用在电机连续编码函数里了（刻度）
	int16_t back_motor_speed;//电机当前速度（刻度）
    double  target_pos;//电机目标编码器值（用在PID位置环）（刻度）
	float   target_motor_speed;//目标电机速度（用在PID单环速度环）（刻度）
	/**motor+IMU**/
	float   back_ang_imu;//返回的imu角度（motor+IMU）
	float   back_ang_last_imu;//返回的上一次imu角度（motor+IMU）
	float   back_ang_speed_imu;//返回的imu角速度（motor+IMU）
	float   back_ang_speed_last_imu;//返回的上一次imu角速度（motor+IMU）
	float   target_ang_imu;//目标imu角度（motor+IMU）
    float   target_ang_speed_imu;//目标imu角速度（motor+IMU）
	/**视觉**/
	float 	visTarget;//视觉目标值（视觉）
	float 	visPitTarget;//PIT轴视觉目标值（视觉）
	/**电流**/
    int16_t out_current;//输出电流值
    int16_t back_current;//返回电流值
	
} s_motor_data_t;//电机信息结构体类型
/*************************************** extern *******************************************/
/*************** Gimbal ******************/
extern s_motor_data_t YAW_motor;//YAW轴电机信息结构
extern s_motor_data_t YAW_motor_imu;//YAW轴用陀螺仪完全控制结构体
extern s_motor_data_t YAW_motor_encode;//YAW用编码器值控制位置环，陀螺仪角速度控制速度环结构体
extern s_motor_data_t PIT_motor;//PIT轴电机信息结构体
extern s_motor_data_t PIT_motor_imu;//PIT轴用陀螺仪完全控制结构体
/**************** Shoot ******************/
extern s_motor_data_t FIRE_L_motor;//左摩擦轮电机信息结构体
extern s_motor_data_t FIRE_R_motor;//右摩擦轮电机信息结构体
extern s_motor_data_t TRANS_motor;//播弹盘电机信息结构体
/**************** Chassis ****************/
extern s_motor_data_t LF_motor;//左前轮电机信息结构体
extern s_motor_data_t RF_motor;//右前轮电机信息结构体
extern s_motor_data_t LB_motor;//左后轮电机信息结构体
extern s_motor_data_t RB_motor;//右后轮电机信息结构体
/***********************************fuctionn declation *************************************/
void continue_motor_pos(s_motor_data_t *s_motor);

/*******************************************************************************************/
#endif
