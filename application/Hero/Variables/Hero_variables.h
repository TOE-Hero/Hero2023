#ifndef _HERO_VARIABLES_H_
#define _HERO_VARIABLES_H_

#include "pid.h"
#include "ramp.h"
#include "dji_motor.h"
/*************************************** extern *******************************************/
/********************************** Motor variable ****************************************/
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
extern s_motor_data_t Camera_motor;
/**************** Chassis ****************/
extern s_motor_data_t LF_motor;//左前轮电机信息结构体
extern s_motor_data_t RF_motor;//右前轮电机信息结构体
extern s_motor_data_t LB_motor;//左后轮电机信息结构体
extern s_motor_data_t RB_motor;//右后轮电机信息结构体





/********************************** PID variable ****************************************/
/************************** Gimbal ****************************/
extern s_pid_absolute_t YAW_motor_pid_pos;//云台pit轴双环
extern s_pid_absolute_t YAW_motor_pid_speed;
extern s_pid_absolute_t YAW_motor_pid_pos_imu;//云台yaw轴陀螺仪双环
extern s_pid_absolute_t YAW_motor_pid_speed_imu;
extern s_pid_absolute_t YAW_motor_pid_pos_encode;//云台yaw轴编码器外环
extern s_pid_absolute_t YAW_motor_pid_speed_encode;
extern s_pid_absolute_t YAW_motor_pid_pos_imu_vis;//云台yaw轴视觉
extern s_pid_absolute_t YAW_motor_pid_speed_imu_vis;

extern s_pid_absolute_t PIT_motor_pid_pos;//云台pit轴双环
extern s_pid_absolute_t PIT_motor_pid_speed;
extern s_pid_absolute_t PIT_motor_pid_pos_imu;//云台pit轴陀螺仪双环
extern s_pid_absolute_t PIT_motor_pid_speed_imu;
extern s_pid_absolute_t PIT_motor_pid_pos_imu_vis;//云台pit轴视觉
extern s_pid_absolute_t PIT_motor_pid_speed_imu_vis;

extern s_pid_absolute_t Camera_motor_pid_pos;//摄像头2006双环
extern s_pid_absolute_t Camera_motor_pid_speed;

/************************* Chassis ****************************/
/*****************底盘速度环*******************/
extern s_pid_absolute_t LF_motor_pid_speed;// +
extern s_pid_absolute_t RF_motor_pid_speed;// -
extern s_pid_absolute_t LB_motor_pid_speed;// +
extern s_pid_absolute_t RB_motor_pid_speed;// -
/*****************底盘吊射模式双环*******************/
extern s_pid_absolute_t LF_motor_pid_stop_pos;
extern s_pid_absolute_t RF_motor_pid_stop_pos;
extern s_pid_absolute_t LB_motor_pid_stop_pos;
extern s_pid_absolute_t RB_motor_pid_stop_pos;
extern s_pid_absolute_t LF_motor_pid_stop_speed;
extern s_pid_absolute_t RF_motor_pid_stop_speed;
extern s_pid_absolute_t LB_motor_pid_stop_speed;
extern s_pid_absolute_t RB_motor_pid_stop_speed;
/************************** Fire ******************************/
/*****************摩擦轮速度环*******************/
extern s_pid_absolute_t FIRE_L_motor_pid_speed;// (-)Value
extern s_pid_absolute_t FIRE_R_motor_pid_speed;// (+)Value
/*****************拨弹盘电机双环*****************/
extern s_pid_absolute_t TRANS_motor_pid_pos;
extern s_pid_absolute_t TRANS_motor_pid_speed;




/**************************** RAMP variable ***********************************/
extern ramp_t ramp_c;
extern ramp_t ramp_w;
extern ramp_t ramp_s;
extern ramp_t ramp_a;
extern ramp_t ramp_d;
extern ramp_t ramp_cFllow;

#endif //_HERO_VARIABLES_H_
