/**************************************************************************************** 
  * @file       bsp_can.c
  * @brief      Initialization of CAN communication and receive data from the CAN bus
  * @author     RosenYin
  ***************************************************************************************
  * Version     Date           Author        Modification
  * V1.0.0      Dec-04-2020    RosenYin      未完成
  * -------------------------- Personal Add ---------------------------------------------
  * @initialize 
  * @funtion    
  * @note       
  ***************************************************************************************
  */
  
/****************************** Private includes *******************************/
#include <math.h>
#include <stdbool.h>
#include "can.h"
#include "bsp_can.h"
#include "hero_can.h"
#include "dji_motor.h"
#include "pid.h"
#include "INS_task.h"
#include "bsp_usart.h"
#include "nuc_interface.h"
#include "RM_Judge.h"

#include "Hero_control.h"
/*-------------------------------------- define --------------------------------------------*/
/******************** Gimbal *****************/
#define YAW_ID			0x205//CAN1
#define PIT_6020_ID		0x205//CAN2
#define FIRE_L_ID		0x203//CAN2
#define FIRE_R_ID		0x204//CAN2
#define TRANS_ID		0x202//CAN2
#define Camera_ID		0x206//CAN1
/******************** Chassis *****************/
#define LF_ID 			0x202//CAN1
#define RF_ID 			0x201//CAN1
#define LB_ID 			0x203//CAN1
#define RB_ID 			0x204//CAN1
/******************** Judge *****************/
//#define JudgeRobotHP_ID                 0x102//机器人剩余血量,帧率比较低，没用
#define JudgeRobotState_ID				0x103//机器人ID，机器人等级，枪口热量上限，弹速上限，底盘功率上限
//#define JudgePowerHeat1_ID              0x104//底盘能量1
#define JudgePowerHeat2_ID              0x105//缓冲能量，实时功率，实时热量
#define JudgePowerState                 0x108//Chasiss 供电状态
//#define JudgeGameState_ID               0x101//比赛状态（没用上）
#define Judge_ShootData_ID              0x112//实时射击信息,主要接收弹速
#if	ROBOT_ID == SUN
	#define Supercapacitors_ID 0x100//CAN1
#endif
#if	ROBOT_ID == MOON
	#define Supercapacitors_ID 0x100//CAN1
#endif
/*---------------------------------- global variable ----------------------------------------*/
/******************************* 裁判系统数据 ************************************/
uint8_t 		RobotId;//机器人ID
uint8_t			RobotLevel;//机器人等级
uint16_t 		shot_ball_amount=0;//发射弹丸数目
u_data_16bit 	chassis_power_buff;//缓冲能量,60J
u_data_16bit 	chassis_power_limit;//功率限制
u_data_16bit 	coolingheat;//拨弹盘（枪管）实时热量
u_data_16bit 	coolingheat_limit;//拨弹盘（枪管）最大冷却值
u_data_16bit 	friSped;//最大弹速
u_data_32bit 	chassis_power;//实时功率
u_data_32bit 	bullet_speed;//弹速
uint8_t			PowerState[3] = {0,0,0};
uint32_t 		JudgePowerStateCount = 0;

/*************************** extern declaration ********************************/
extern chassisMove_t		s_chassisMove;
extern s_FPS_monitor		Fps;
extern int16_t 				Fric_SpeedTarget;
extern uint8_t          	capExpect_txBuff[8];
extern s_FPS_monitor    	finalFps;
extern s_robo_Mode_Setting	robot_Mode;
extern su_PC_DATA 			pcData;
/*************** Gimbal ******************/
extern s_motor_data_t 		YAW_motor;
extern s_motor_data_t 		YAW_motor_imu;
extern s_motor_data_t 		YAW_motor_encode;
extern s_motor_data_t 		PIT_motor;
extern s_motor_data_t 		PIT_motor_imu;
extern s_motor_data_t 		Camera_motor;//摄像头调整使用的2006
/**************** Shoot ******************/
extern s_motor_data_t 		FIRE_L_motor;
extern s_motor_data_t 		FIRE_R_motor;
extern s_motor_data_t 		TRANS_motor;
/**************** Chassis ****************/
extern s_motor_data_t 		LF_motor;
extern s_motor_data_t 		RF_motor;
extern s_motor_data_t 		LB_motor;
extern s_motor_data_t 		RB_motor;
/***************** Judge *****************/
extern s_rm_judge_shoot_data_t	Judge_ShootData;
/****************************** Functions declaration *****************************/
void CAN_Enable(CAN_HandleTypeDef *Target_hcan);

/******************************** Private functions *******************************/

static void CANFilter_Enable(CAN_HandleTypeDef* hcan);
static void UI_API1(void);
static void UI_API2(void);
/**********************************************************************************/

/**
 * @brief     给底盘C板发送要显示在UI上的信息：
 * PIT轴电机角度、电容电压百分比、底盘模式、云台模式、发射机构模式、自瞄是否识别到
 * YAW轴电机角度、摩擦轮电机目标转速、摩擦轮电机温度、摩擦轮电机error标志位、UI刷新标志位
 * @param     none
 * @return    None
 */
void UI_API(void)
{
	UI_API1();
	UI_API2();
}

/**
 * @brief     给底盘C板发送要显示在UI上的信息：PIT轴电机角度、电容电压百分比、底盘模式、云台模式、发射机构模式、自瞄是否识别到
 * @param     none
 * @return    None
 */
static void UI_API1(void)
{
	uint8_t cBoard_txBuff[8];
	u_data_16bit cap_precentV;
	u_data_16bit pitAng;
	float capV=s_chassisMove.capVol;

    #if	ROBOT_ID == SUN
    cap_precentV.integer = capV * capV / ( 29.0 * 29.0) * 100;
    #endif
    #if	ROBOT_ID == MOON
    // cap_precentV.integer = capV * capV / ( 29.5 * 29.5) * 100;
	cap_precentV.integer = (uint16_t)capV;
    #endif

    if(cap_precentV.integer>100) cap_precentV.integer=100;
    if(cap_precentV.integer<0) cap_precentV.integer=0;
	pitAng.integer = (int16_t)PIT_motor.serial_motor_ang*100;
	cBoard_txBuff[0] = pitAng.arr2[0];//PIT轴电机角度（需除以100转换）
	cBoard_txBuff[1] = pitAng.arr2[1];
	cBoard_txBuff[2] = cap_precentV.arr2[0];//电容电压百分比（0-100）
	cBoard_txBuff[3] = cap_precentV.arr2[1];
	cBoard_txBuff[4] = robot_Mode.chassisMode;//底盘模式（0、1、2、3、4）
	cBoard_txBuff[5] = robot_Mode.gimbalMode;//云台模式（0、1、2、3、4、5）
	cBoard_txBuff[6] = robot_Mode.shootMode;//发射机构模式（0、1、2）
	cBoard_txBuff[7] = pcData.target.isExist;//自瞄是否识别到（0/1）

	CAN_Send_bytes(&hcan2, 0x300, cBoard_txBuff);
}


/**
 * @brief     给底盘C板发送要显示在UI上的信息：YAW轴电机角度、摩擦轮电机目标转速、摩擦轮电机温度、摩擦轮电机error标志位、UI刷新标志位
 * @param     none
 * @return    None
 */
static void UI_API2(void)
{
	uint8_t cBoard_txBuff_yaw[8];
	uint8_t UI_sync_flag = 0;//UI刷新标志位
	uint8_t firc_error_flag = 0;//摩擦轮状态为开启，并且两个电机返回转速的绝对值之和为1000以下的值时，该标志位置1
	u_data_16bit yawEncode;
	u_data_16bit fricTemp;
	u_data_16bit fricTargetSpd;
	float f_yaw_ang = (YAW_motor.back_position/8191.0f)*360.0f;//yaw轴刻度转角度
	yawEncode.integer = (int16_t)(f_yaw_ang);
	fricTargetSpd.u_integer = abs(FIRE_L_motor.target_motor_speed);//摩擦轮温度
	fricTemp.u_integer = FIRE_L_motor.temperature;
	//摩擦轮错误标志位置位
	if(robot_Mode.shootMode==S_SWING && (abs(FIRE_L_motor.back_motor_speed)+abs(FIRE_R_motor.back_motor_speed)) < 1000)
		firc_error_flag = 1;
	else firc_error_flag = 0;
	// UI刷新标志位
	if(PRESS_V && (LS_DOWN && RS_UP))
		UI_sync_flag = 1;
	else
		UI_sync_flag = 0;
	cBoard_txBuff_yaw[0] = yawEncode.arr2[0];//YAW轴电机角度（需除以100转换）
	cBoard_txBuff_yaw[1] = yawEncode.arr2[1];
	cBoard_txBuff_yaw[2] = fricTargetSpd.arr2[0];//摩擦轮电机目标转速
	cBoard_txBuff_yaw[3] = fricTargetSpd.arr2[1];
	cBoard_txBuff_yaw[4] = fricTemp.arr2[0];//摩擦轮电机温度
	cBoard_txBuff_yaw[5] = fricTemp.arr2[1];
	cBoard_txBuff_yaw[6] = firc_error_flag;//摩擦轮电机error标志位
	cBoard_txBuff_yaw[7] = UI_sync_flag;//UI刷新标志位
	CAN_Send_bytes(&hcan2, 0x301, cBoard_txBuff_yaw);
}
/* ----------------------CAN interruption callback function ------------------------------*/

/**
 * @brief     interrupt function in IRQ
 * 进入回调的条件：1.CAN FIFO 0消息挂起中断使能，CAN_IT_RX_FIFO0_MSG_PENDING置1（自定义的CAN_Enable中使能）
 *				  2.CAN接收FIFO 0寄存器CAN_RF0R的D1~D0位不为0，即FIFO 0中有消息被挂起
 *				  3.USE_HAL_CAN_REGISTER_CALLBACKS不为1
 * 中断回调如何被调用：当产生CAN接收中断时，进入HAL库的中断处理函数，在其中间接调用了中断回调，中断回调函数用__weak修饰，其中内容自定义
 * @param     CAN_HandleTypeDef *hcan
 * @return    None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t CAN1_RX_Buff[8];
	static uint8_t CAN2_RX_Buff[8];
	CAN_RxHeaderTypeDef CAN1RxHeader; //header we need to receive the data from CAN
	CAN_RxHeaderTypeDef CAN2RxHeader;
	//if the interruption source is CAN1, mainly the chassis
	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1RxHeader, CAN1_RX_Buff); //fetch data from FIFO to 'CANx_RX_Buff'
		//judge the ID to check which motor
		
		switch (CAN1RxHeader.StdId)
		{
			case YAW_ID:
						{
						YAW_motor.back_position     = CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						YAW_motor.back_motor_speed  = CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						YAW_motor.back_current      = CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						YAW_motor_imu.back_position     = CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						YAW_motor_imu.back_motor_speed  = CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						YAW_motor_imu.back_current      = CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						YAW_motor_encode.back_position     = CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						YAW_motor_encode.back_motor_speed  = CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						YAW_motor_encode.back_current      = CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						continue_motor_pos(&YAW_motor);
						Fps.yaw++;
						break;
						}
			case LF_ID:
						{
						LF_motor.back_position    =		CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						LF_motor.back_motor_speed =		CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						LF_motor.back_current     = 	CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						continue_motor_pos(&LF_motor);
						Fps.LF_motor++;
						break;
						}
			case RF_ID:
						{
						RF_motor.back_position    =		CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						RF_motor.back_motor_speed =		CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						RF_motor.back_current     =		CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						continue_motor_pos(&RF_motor);
						Fps.RF_motor++;
						break;
						}
			case LB_ID:
						{
						LB_motor.back_position    =		CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						LB_motor.back_motor_speed =		CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						LB_motor.back_current     = 	CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						continue_motor_pos(&LB_motor);
						Fps.LB_motor++;
						break;
						}
			case RB_ID:
						{
						RB_motor.back_position    =		CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						RB_motor.back_motor_speed =		CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						RB_motor.back_current     =		CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						continue_motor_pos(&RB_motor);
						Fps.RB_motor++;
						break;
						}
			case Supercapacitors_ID:
						{
						//uint16_t * pPowerdata = (uint16_t *) CAN1_RX_Buff;
						s_chassisMove.capInV 		 = 		(float)(CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1]) / 100.0f; //输入电压(can1_rx_message.Data[0]>>8 | can1_rx_message.Data[1]) / 100.0f;
						s_chassisMove.capVol		 = 		(float)(CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3]) / 100.0f; //电容电压(can1_rx_message.Data[2]>>8 | can1_rx_message.Data[3]) / 100.0f;
						s_chassisMove.capInC		 = 		(float)(CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5]) / 100.0f; //输入电流(can1_rx_message.Data[4]>>8 | can1_rx_message.Data[5]) / 100.0f;
						s_chassisMove.cap_percent	 =		(float)CAN1_RX_Buff[6];						  //电容容量(can1_rx_message.Data[6]);  当返回0的时候,电容电容控制板可能未与电容连接
						s_chassisMove.cap_expect	 = 		(float)CAN1_RX_Buff[7];							  //设定功率(can1_rx_message.Data[7]); 接受数据类型为整数,因此只能显示整数功率值
						Fps.cap_board++;
						break;
						}
			case Camera_ID:
						{
						Camera_motor.back_position    =		CAN1_RX_Buff[0]<<8 | CAN1_RX_Buff[1];
						Camera_motor.back_motor_speed =		CAN1_RX_Buff[2]<<8 | CAN1_RX_Buff[3];
						Camera_motor.back_current     = 	CAN1_RX_Buff[4]<<8 | CAN1_RX_Buff[5];
						continue_motor_pos(&Camera_motor);
						Fps.vedio_transmssion++;
						break;
						}
			
		}
	}
	
	if(hcan -> Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0,&CAN2RxHeader,CAN2_RX_Buff );//从FIFO中接收消息至CAN2_RX_Buff
		switch (CAN2RxHeader.StdId)
		{
			#if PIT_MOTOR == GM6020
				case PIT_6020_ID:
						{
						PIT_motor.back_position    =    CAN2_RX_Buff[0]<<8 | CAN2_RX_Buff[1];
						PIT_motor.back_motor_speed =    CAN2_RX_Buff[2]<<8 | CAN2_RX_Buff[3];
						PIT_motor.back_current     =    CAN2_RX_Buff[4]<<8 | CAN2_RX_Buff[5];
						continue_motor_pos(&PIT_motor);
						Fps.pitch++;
						break;
						}
			#endif
				case TRANS_ID:
						{
						TRANS_motor.back_position    =    CAN2_RX_Buff[0]<<8 | CAN2_RX_Buff[1];
						TRANS_motor.back_motor_speed =    CAN2_RX_Buff[2]<<8 | CAN2_RX_Buff[3];
						TRANS_motor.back_current     =    CAN2_RX_Buff[4]<<8 | CAN2_RX_Buff[5];
						continue_motor_pos(&TRANS_motor);
						Fps.trans++;
						break;
						}
				case FIRE_L_ID:
						{
						FIRE_L_motor.back_position    =   CAN2_RX_Buff[0]<<8 | CAN2_RX_Buff[1];
						FIRE_L_motor.back_motor_speed =   CAN2_RX_Buff[2]<<8 | CAN2_RX_Buff[3];
						FIRE_L_motor.back_current     =   CAN2_RX_Buff[4]<<8 | CAN2_RX_Buff[5];
						FIRE_L_motor.temperature	  =   CAN2_RX_Buff[6];
						continue_motor_pos(&FIRE_L_motor);
						Fps.fric_l++;
						break;
						}
				case FIRE_R_ID:
						{
						FIRE_R_motor.back_position    =   CAN2_RX_Buff[0]<<8 | CAN2_RX_Buff[1];
						FIRE_R_motor.back_motor_speed =   CAN2_RX_Buff[2]<<8 | CAN2_RX_Buff[3];
						FIRE_R_motor.back_current     =   CAN2_RX_Buff[4]<<8 | CAN2_RX_Buff[5];
						FIRE_R_motor.temperature      =   CAN2_RX_Buff[6];
						continue_motor_pos(&FIRE_R_motor);
						Fps.fric_r++;
						break;
						}
				case JudgePowerHeat2_ID://缓冲能量，实时功率，实时热量
						{
						chassis_power_buff.arr2[0] = CAN2_RX_Buff[0];
						chassis_power_buff.arr2[1] = CAN2_RX_Buff[1];
						s_chassisMove.chassis_power_buffer = chassis_power_buff.integer;//缓冲能量,60J
						chassis_power.arr4[0] = CAN2_RX_Buff[2];
						chassis_power.arr4[1] = CAN2_RX_Buff[3];
						chassis_power.arr4[2] = CAN2_RX_Buff[4];
						chassis_power.arr4[3] = CAN2_RX_Buff[5];
						s_chassisMove.chassis_power = chassis_power.f;//实时功率
						coolingheat.arr2[0] = CAN2_RX_Buff[6];
						coolingheat.arr2[1] = CAN2_RX_Buff[7];
						Judge_ShootData.coolingheat = coolingheat.u_integer;//拨弹盘（枪管）实时热量
						break;
						}
				case JudgeRobotState_ID://机器人ID，机器人等级，枪口热量上限，弹速上限，底盘功率上限
						{
						RobotId = CAN2_RX_Buff[0];//红方or蓝方,1-RED | 101-BLUE
						RobotLevel = CAN2_RX_Buff[1];//机器人等级（1，2，3）
						coolingheat_limit.arr2[0] = CAN2_RX_Buff[2];
						coolingheat_limit.arr2[1] = CAN2_RX_Buff[3];
						Judge_ShootData.coolingheat_limit = coolingheat_limit.u_integer;//拨弹盘（枪管）最大冷却值
						friSped.arr2[0] = CAN2_RX_Buff[4];
						friSped.arr2[1] = CAN2_RX_Buff[5];
						Fric_SpeedTarget = friSped.integer;//最大弹速
						chassis_power_limit.arr2[0] = CAN2_RX_Buff[6];
						chassis_power_limit.arr2[1] = CAN2_RX_Buff[7];
						s_chassisMove.chassis_power_limit = chassis_power_limit.u_integer;//功率限制
						Fps.judge++;
						break;
						}
				case Judge_ShootData_ID:
						{
						bullet_speed.arr4[0] = CAN2_RX_Buff[3];
						bullet_speed.arr4[1] = CAN2_RX_Buff[4];
						bullet_speed.arr4[2] = CAN2_RX_Buff[5];
						bullet_speed.arr4[3] = CAN2_RX_Buff[6];
						shot_ball_amount++;
						}
				case JudgePowerState:
						{
						PowerState[0] = CAN2_RX_Buff[0];
						PowerState[1] = CAN2_RX_Buff[1];
						PowerState[2] = CAN2_RX_Buff[2];
						if(finalFps.judge > 8)
						{
						capExpect_txBuff[4] = PowerState[1];
						}
						JudgePowerStateCount++;
						}
		}
	}
    
}

