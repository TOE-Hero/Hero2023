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
#include <math.h>
#include <stdbool.h>

/****************************** Functions declaration *****************************/
void CAN_Enable(CAN_HandleTypeDef *Target_hcan);
/******************************** Private functions *******************************/
static void CANFilter_Enable(CAN_HandleTypeDef* hcan);
/**********************************************************************************/

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
/**
 * @brief 发送数据到CAN
 * 
 * @param hcan 句柄
 * @param id CAN-id
 * @param data 存放数据的数组，长度为8
 */
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


