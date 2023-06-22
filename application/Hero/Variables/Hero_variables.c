#include "Hero_variables.h"
#include "pid.h"
#include "ramp.h"
#include "dji_motor.h"
/********************************** global variable ****************************************/
/********************************** Motor variable ****************************************/
/*************** Gimbal ******************/
s_motor_data_t YAW_motor;
s_motor_data_t YAW_motor_imu;
s_motor_data_t YAW_motor_encode;
s_motor_data_t PIT_motor;
s_motor_data_t PIT_motor_imu;
s_motor_data_t Camera_motor;
/**************** Shoot ******************/
s_motor_data_t FIRE_L_motor;
s_motor_data_t FIRE_R_motor;
s_motor_data_t TRANS_motor;
/**************** Chassis ****************/
s_motor_data_t LF_motor;
s_motor_data_t RF_motor;
s_motor_data_t LB_motor;
s_motor_data_t RB_motor;


/********************************** PID variable ****************************************/
/************************** Gimbal ****************************/
s_pid_absolute_t YAW_motor_pid_pos;//云台pit轴双环
s_pid_absolute_t YAW_motor_pid_speed;
s_pid_absolute_t YAW_motor_pid_pos_imu;//云台yaw轴陀螺仪双环
s_pid_absolute_t YAW_motor_pid_speed_imu;
s_pid_absolute_t YAW_motor_pid_pos_encode;//云台yaw轴编码器外环
s_pid_absolute_t YAW_motor_pid_speed_encode;
s_pid_absolute_t YAW_motor_pid_pos_imu_vis;//云台yaw轴视觉
s_pid_absolute_t YAW_motor_pid_speed_imu_vis;

s_pid_absolute_t PIT_motor_pid_pos;//云台pit轴双环
s_pid_absolute_t PIT_motor_pid_speed;
s_pid_absolute_t PIT_motor_pid_pos_imu;//云台pit轴陀螺仪双环
s_pid_absolute_t PIT_motor_pid_speed_imu;
s_pid_absolute_t PIT_motor_pid_pos_imu_vis;//云台pit轴视觉
s_pid_absolute_t PIT_motor_pid_speed_imu_vis;

s_pid_absolute_t Camera_motor_pid_pos;//摄像头2006双环
s_pid_absolute_t Camera_motor_pid_speed;

/************************* Chassis ****************************/
/*****************底盘速度环*******************/
s_pid_absolute_t LF_motor_pid_speed;// +
s_pid_absolute_t RF_motor_pid_speed;// -
s_pid_absolute_t LB_motor_pid_speed;// +
s_pid_absolute_t RB_motor_pid_speed;// -
/*****************底盘吊射模式双环*******************/
s_pid_absolute_t LF_motor_pid_stop_pos;
s_pid_absolute_t RF_motor_pid_stop_pos;
s_pid_absolute_t LB_motor_pid_stop_pos;
s_pid_absolute_t RB_motor_pid_stop_pos;
s_pid_absolute_t LF_motor_pid_stop_speed;
s_pid_absolute_t RF_motor_pid_stop_speed;
s_pid_absolute_t LB_motor_pid_stop_speed;
s_pid_absolute_t RB_motor_pid_stop_speed;
/************************** Fire ******************************/
/*****************摩擦轮速度环*******************/
s_pid_absolute_t FIRE_L_motor_pid_speed;// (-)Value
s_pid_absolute_t FIRE_R_motor_pid_speed;// (+)Value
/*****************拨弹盘电机双环*****************/
s_pid_absolute_t TRANS_motor_pid_pos;
s_pid_absolute_t TRANS_motor_pid_speed;


/**************************** RAMP variable ***********************************/
ramp_t ramp_cFllow = RAMP_SLOW_INIT;
ramp_t ramp_c = RAMP_SLOW_INIT;
ramp_t ramp_w = RAMP_DEFAULT_INIT;
ramp_t ramp_s = RAMP_DEFAULT_INIT;
ramp_t ramp_a = RAMP_DEFAULT_INIT;
ramp_t ramp_d = RAMP_DEFAULT_INIT;

