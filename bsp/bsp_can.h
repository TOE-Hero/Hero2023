#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "stm32f4xx.h"
#include "motor.h"
#include "pid.h"
#include "Mode_Switch.h"
/******************** Gimbal *****************/
#define YAW_ID			0x205//CAN1
#define PIT_ID			0	 //CAN2
#define PIT_6020_ID		0x205//CAN2
#define FIRE_L_ID		0x203//CAN2
#define FIRE_R_ID		0x204//CAN2
#define TRANS_ID		0x202//CAN2
#define Camera_ID		0x206//CAN1

extern s_motor_data_t YAW_motor;
extern s_motor_data_t YAW_motor_imu;
extern s_motor_data_t YAW_motor_top;
extern s_motor_data_t PIT_motor;
extern s_motor_data_t FIRE_L_motor;
extern s_motor_data_t FIRE_R_motor;
extern s_motor_data_t TRANS_motor;
/******************** Chassis *****************/
#define LF_ID 0x202//CAN1
#define RF_ID 0x201//CAN1
#define LB_ID 0x203//CAN1
#define RB_ID 0x204//CAN1
#if	ROBOT_ID == SUN
	#define Supercapacitors_ID 0x100//CAN1
#endif
#if	ROBOT_ID == MOON
	#define Supercapacitors_ID 0x100//CAN1
#endif
extern s_motor_data_t LF_motor;
extern s_motor_data_t RF_motor;
extern s_motor_data_t LB_motor;
extern s_motor_data_t RB_motor;
/******************** Judge *****************/
//#define JudgeRobotHP_ID                 0x102//机器人剩余血量,帧率比较低，没用
#define JudgeRobotState_ID				0x103//机器人ID，机器人等级，枪口热量上限，弹速上限，底盘功率上限
//#define JudgePowerHeat1_ID              0x104//底盘能量1
#define JudgePowerHeat2_ID              0x105//缓冲能量，实时功率，实时热量
#define JudgePowerState                 0x108//Chasiss 供电状态
//#define JudgeGameState_ID               0x101//比赛状态（没用上）
#define Judge_ShootData_ID              0x112//实时射击信息,主要接收弹速
/*********************** CAN 发送\接收 结构体数组 ***************/
typedef struct{
    uint8_t Data[8];
} s_CAN_Message;

typedef union{
	uint8_t arr2[2];
	int16_t integer;
	uint16_t u_integer;
}u_data_16bit;

typedef union{
	uint8_t arr4[4];
	int integer;
	float f;
	uint32_t u_integer;
}u_data_32bit;
/******************* 函数声明 ******************/
void MY_CAN_Init(void);
void CAN_Enable(CAN_HandleTypeDef *Target_hcan);
uint8_t CANTx_SendCurrent(CAN_HandleTypeDef *Target_hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void CAN_Send_bytes(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t data[8]);
void UI_API(void);
void UI_API_YAW(void);

#endif  /* BSP_CAN_H */

