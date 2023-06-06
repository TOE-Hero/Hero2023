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
#include "can.h"
#include "bsp_can.h"
#include "motor.h"
#include "pid.h"
#include "INS_task.h"
#include <math.h>
#include "bsp_usart.h"
#include "Chassis.h"
#include "Monitor.h"
#include "nuc_interface.h"
#include "Mode_Switch.h"
/********************************** AK-60 define ***********************************/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -41.87f
#define V_MAX 41.87f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -9.0f
#define T_MAX 9.0f
/********************************** global variable ****************************************/
int can1_sendY_N=0;
int can2_sendY_N=0;
/*************** Gimbal ******************/
extern s_motor_data_t YAW_motor;
extern s_motor_data_t YAW_motor_imu;
extern s_motor_data_t YAW_motor_encode;
extern s_motor_data_t PIT_motor;
extern s_motor_data_t PIT_motor_imu;

extern s_motor_data_t Camera_motor;//摄像头调整使用的2006
/**************** Shoot ******************/
extern s_motor_data_t FIRE_L_motor;
extern s_motor_data_t FIRE_R_motor;
extern s_motor_data_t TRANS_motor;
/**************** Chassis ****************/
extern s_motor_data_t LF_motor;
extern s_motor_data_t RF_motor;
extern s_motor_data_t LB_motor;
extern s_motor_data_t RB_motor;
/***************** Judge *****************/
u_data_16bit pitch_gyro_spd;
u_data_16bit chassis_c;
u_data_16bit chassis_v;
u_data_16bit chassis_power_buff;
u_data_16bit chassis_power_limit;
u_data_16bit coolingheat;
u_data_16bit coolingheat_limit;
u_data_16bit gimbPitDeg;
u_data_16bit capVol;
u_data_16bit friSped;

u_data_32bit chassis_power;
u_data_32bit chassis_gyro_spd;
u_data_32bit pit_gyro;
u_data_32bit bullet_speed;
uint8_t PowerState[3] = {0,0,0};
uint32_t JudgePowerStateCount = 0;

int16_t yawAng_to_Judge;

uint8_t RobotId, RobotLevel;

uint16_t shot_ball_amount=0;

uint16_t SuperCapDataArray[50];
uint8_t SuperCapDataArrayCount = 0;
/*************************** extern declaration ***********************************/
extern chassisMove_t	s_chassisMove;
extern s_FPS_monitor	Fps;
extern int16_t 			Fric_SpeedTarget;
extern uint8_t          capExpect_txBuff[8];
extern s_FPS_monitor       finalFps;
/****************************** Functions declaration *****************************/
void CAN_Enable(CAN_HandleTypeDef *Target_hcan);

/******************************** Private functions *******************************/
/**
 * @brief     Enable filter
 * @param     CAN_HandleTypeDef* hcan
 * @return    none
 * @attention filter bank 0 for CAN1 and filter bank 14 for CAN2
 */
static void CANFilter_Enable(CAN_HandleTypeDef* hcan);
#if PIT_MOTOR == AK60
static void Pitch_CanReceive(s_motor_data_t *motor, uint8_t RxData[8]);
#endif
static float uint_to_float(int x_int, float x_min, float x_max, int bits);

/**********************************************************************************/
// Some initialization function --------------------------

/**
  * @brief  Init CAN1、CAN2，放到main.c里
  * @param  None
  * @retval void
  * @usage  call in main() function
    
  */
void MY_CAN_Init(void)
{
    CAN_Enable(&hcan1);
    CAN_Enable(&hcan2);
}

/**
  * @brief     Enable CAN1 and CAN2
  * @param     None
  * @return    0 if succeed/ 1 if failed
  */
void CAN_Enable(CAN_HandleTypeDef *Target_hcan)
{
    HAL_CAN_Start(Target_hcan);// 激活对应的 CAN
//CAN_IT_RX_FIFO0_MSG_PENDING,CAN FIFO 0消息挂起中断使能（接收中断）CAN_IER寄存器D1位，地址0x00000002（32位），使能后（值为1），
//CAN接收FIFO 0寄存器中的FMP0[1:0]（接收消息中挂起的消息数），当存储一条新信息，FMP就会增加，当FMP值不为0时产生中断。
    HAL_CAN_ActivateNotification(Target_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    CANFilter_Enable(Target_hcan);// 目标 CAN 滤波器使能
    //printf("CAN initialized successfully!\r\n");
}
/**
 * @brief     Enable filter
 * @param     
 * @return    
 * @attention filter bank 0 for CAN1 and filter bank 14 for CAN2
 */
static void CANFilter_Enable(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef filter;
	
	if (hcan->Instance == CAN1)
	{
	filter.FilterActivation = ENABLE;
	filter.FilterBank = 0U;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan1, &filter);
	}
	if(hcan->Instance == CAN2)
	{
     filter.FilterActivation=ENABLE;
     filter.FilterBank=14;
     filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
     filter.FilterIdHigh=0x0000;
     filter.FilterIdLow=0x0000;
     filter.FilterMaskIdHigh=0x0000;
     filter.FilterMaskIdLow=0x0000;
     filter.FilterMode=CAN_FILTERMODE_IDMASK;
     filter.FilterScale=CAN_FILTERSCALE_32BIT;
     filter.SlaveStartFilterBank=14;
     
     HAL_CAN_ConfigFilter(&hcan2,&filter);
     //printf("CAN filter2 initialized successfully!\r\n");
    }

}


// CAN transmit data function --------------------------------

/**
 * @brief Send the message by Can bytes（can发送函数）
 * 为何需要(unsigned char)强制类型转换：与数制与码制有关，所有数字以补码存在计算机中，正数的补码是其本身，负数的补码是其去掉负号后的正
 * 数的二进制取反码再加1，若电流值为正数无影响，若为负数，在位移操作时，由于第一位为符号位，计算机为保持其负号，会始终在其第一位保持1
   ，因此，每位移1位其第一位都会变为1，如-16384（1100 0000 0000 0000），不强制类型转换无符号字节后位移8位（1111 1111 1100 0000）
   强制类型转换位无符号字节类型刚好截断其后8位并保留了符号位，具体细节可百度。
 * @param CAN_HandleTypeDef* Target_hcan
 * @param uint32_t id
 * @param int16_t current1
 * @param int16_t current2
 * @param int16_t current3
 * @param int16_t current4
 * @return 0 if succeed/ 1 if failed
 * @attention None
 */
uint8_t CANTx_SendCurrent(CAN_HandleTypeDef* Target_hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    //声明存储发送数据的数组
    uint8_t TX_message[8] = {0};
    //定义 CAN 的发送消息句柄
    CAN_TxHeaderTypeDef CanTxHeader;
    //定义发送邮箱
    uint32_t TX_MailBOX = CAN_TX_MAILBOX0;

    //配置 CAN 的发送消息句柄
    CanTxHeader.StdId = id;
    CanTxHeader.IDE = CAN_ID_STD;
    CanTxHeader.RTR = CAN_RTR_DATA;
    CanTxHeader.DLC = 0x08;

    TX_message[0] = (unsigned char)(current1 >> 8);
    TX_message[1] = (unsigned char)current1;
    TX_message[2] = (unsigned char)(current2 >> 8);
    TX_message[3] = (unsigned char)current2;
    TX_message[4] = (unsigned char)(current3 >> 8);
    TX_message[5] = (unsigned char)current3;
    TX_message[6] = (unsigned char)(current4 >> 8);
    TX_message[7] = (unsigned char)current4;
//	if(HAL_CAN_AddTxMessage(&hcan2, &CanTxHeader, TX_message, &TX_MailBOX)!=HAL_OK)
//	//if(HAL_CAN_IsTxMessagePending(&hcan2, TX_MailBOX)!=0)
//	{
//		printf("error\r\n");
//	}
//	else
//	{
//		printf("success\r\n");
//	}
    //将发送的信息添加到信箱，之后发送
    if(HAL_CAN_AddTxMessage(Target_hcan, &CanTxHeader, TX_message, &TX_MailBOX) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

void CAN_Send_bytes(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t data[8])
{
	CAN_TxHeaderTypeDef CanTxHeader;
	s_CAN_Message tx_message;
	uint32_t send_mail_box;
	
	CanTxHeader.StdId = id;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.DLC = 0x08;

	tx_message.Data[0] = data[0];
	tx_message.Data[1] = data[1];
	tx_message.Data[2] = data[2];
	tx_message.Data[3] = data[3];
	tx_message.Data[4] = data[4];
	tx_message.Data[5] = data[5];
	tx_message.Data[6] = data[6];
	tx_message.Data[7] = data[7];

	HAL_CAN_AddTxMessage(hcan, &CanTxHeader, tx_message.Data, &send_mail_box);
}

extern s_robo_Mode_Setting robot_Mode;
extern su_PC_DATA pcData;
u_data_16bit cap_precentV;
u_data_16bit pitAng;
/**
 * @brief     给底盘C板发送要显示在UI上的信息
 * @param     none
 * @return    None
 */
void UI_API(void)
{
	static uint8_t cBoard_txBuff[8];
	float capV=s_chassisMove.capVol;
	// if(capV>28) capV=28;
	// // 17-28映射为0-100，方便UI显示
	// cap_precentV.integer = ((capV-17)/11)*100;
	// if(cap_precentV.integer<0) cap_precentV.integer=0;
	    //cap_precentV.integer= ((capV-17)/5)*100;

    #if	ROBOT_ID == SUN
    cap_precentV.integer = capV * capV / ( 29.0 * 29.0) * 100;
    #endif
    #if	ROBOT_ID == MOON
    cap_precentV.integer = capV * capV / ( 29.5 * 29.5) * 100;
    #endif

    //cap_precentV.integer = (uint16_t)s_chassisMove.cap_percent;
    if(cap_precentV.integer>100) cap_precentV.integer=100;
    if(cap_precentV.integer<0) cap_precentV.integer=0;
	pitAng.integer = PIT_motor.serial_motor_ang*100;
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

u_data_16bit yawEncode;
u_data_16bit fricTargetSpd;
u_data_16bit fricTemp;
static uint8_t cBoard_txBuff_yaw[8];
static uint8_t firc_error_flag=0;//摩擦轮状态为开启，并且两个电机返回转速的绝对值之和为1000以下的值时，该标志位置1
void UI_API_YAW(void)
{
	float f_yaw_ang = (YAW_motor.back_position/8191.0f)*360.0f;
	yawEncode.integer = (int16_t)(f_yaw_ang);
	fricTargetSpd.u_integer = abs(FIRE_L_motor.target_motor_speed);
	fricTemp.u_integer = FIRE_L_motor.temperature;
	if(robot_Mode.shootMode==S_SWING && (abs(FIRE_L_motor.back_motor_speed)+abs(FIRE_R_motor.back_motor_speed)) < 1000)
	firc_error_flag = 1;
	else firc_error_flag = 0;

	cBoard_txBuff_yaw[0] = yawEncode.arr2[0];//YAW轴电机角度（需除以100转换）
	cBoard_txBuff_yaw[1] = yawEncode.arr2[1];
	cBoard_txBuff_yaw[2] = fricTargetSpd.arr2[0];//摩擦轮电机目标转速
	cBoard_txBuff_yaw[3] = fricTargetSpd.arr2[1];
	cBoard_txBuff_yaw[4] = fricTemp.arr2[0];//摩擦轮电机温度
	cBoard_txBuff_yaw[5] = fricTemp.arr2[1];
	cBoard_txBuff_yaw[6] = firc_error_flag;
	CAN_Send_bytes(&hcan2, 0x301, cBoard_txBuff_yaw);
}
/* ----------------------CAN interruption callback function ------------------------------*/
uint16_t test_canRx=0;
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
						TRANS_motor.coolingheat = coolingheat.u_integer;//拨弹盘（枪管）实时热量
						break;
						}
				case JudgeRobotState_ID://机器人ID，机器人等级，枪口热量上限，弹速上限，底盘功率上限
						{
						RobotId = CAN2_RX_Buff[0];//红方or蓝方,1-RED | 101-BLUE
						RobotLevel = CAN2_RX_Buff[1];//机器人等级（1，2，3）
						coolingheat_limit.arr2[0] = CAN2_RX_Buff[2];
						coolingheat_limit.arr2[1] = CAN2_RX_Buff[3];
						TRANS_motor.coolingheat_limit = coolingheat_limit.u_integer;//拨弹盘（枪管）最大冷却值
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

