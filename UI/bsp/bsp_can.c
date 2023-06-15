/**
  * @file    bsp_can.c
  * @version 0.0
  * @data    2020/10/17
  * @author  sxq
  *
  * @usage   To initialze the CAN port and enable them in the
  *          STM32 F4 Series
  * @note    
  */

/****** Include in this source file ******/
#include "bsp_can.h"
#include <stdio.h>

/****** Function declared in this file ******/

/**
  * @brief  软件初始化 CAN
  * @param  None
  * @retval void
  * @usage  call in main() function
	
  */
UI_RX uiRx;

extern float motorAngle;
void MY_CAN_Init(void)
{
    CAN_Enable(&hcan1);
	CAN_Enable(&hcan2);
}

/**
  * @brief  使能 CAN 接口
  * @param  <-- hcan -->
  *         待使能的 CAN 句柄指针
  * @retval void
  */
void CAN_Enable(CAN_HandleTypeDef *Target_hcan)
{
    // 激活对应的 CAN
    HAL_CAN_Start(Target_hcan);
    // 使能 CAN 接收中断
    HAL_CAN_ActivateNotification(Target_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // 目标 CAN 滤波器使能
    CANFilter_Enable(Target_hcan);
}

/**
  * @brief  Enable CAN_Filter(0 for can1 and 14 for can2)
  *         滤波器初始化函数
  * @param  <-- Target_hcan -->
  *         我们使用这个变量名代替待初始化的句柄指针
  * @retval void
  * @usage  call in CAN_Enable() function
  */
void CANFilter_Enable(CAN_HandleTypeDef *Target_hcan)
{
    CAN_FilterTypeDef filter1;
	  CAN_FilterTypeDef filter2;
	 if(Target_hcan -> Instance == CAN1)
	{
	    filter1.FilterIdHigh = 0x0000;
	    filter1.FilterIdLow = 0x0000;
	    filter1.FilterMaskIdHigh = 0x0000;
	    filter1.FilterMaskIdLow = 0x0000;
	    filter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	    filter1.FilterBank = 0;
	    //filter1.SlaveStartFilterBank=14;
	    filter1.FilterMode = CAN_FILTERMODE_IDMASK;
	    filter1.FilterScale = CAN_FILTERSCALE_32BIT;
		  filter1.FilterActivation = ENABLE;
	    HAL_CAN_ConfigFilter(&hcan1, &filter1);
	}
	if(Target_hcan->Instance == CAN2)
	{
		filter2.FilterActivation=ENABLE;
		filter2.FilterBank=14;
		filter2.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter2.FilterIdHigh=0x0000;
		filter2.FilterIdLow=0x0000;
		filter2.FilterMaskIdHigh=0x0000;
		filter2.FilterMaskIdLow=0x0000;
		filter2.FilterMode=CAN_FILTERMODE_IDMASK;
		filter2.FilterScale=CAN_FILTERSCALE_32BIT;
		filter2.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan2,&filter2);
	}
}
/**
  * @brief  通过 CAN_Transmit_IT 发送信息至电机电调
  * @param  <-- hcan -->
  *         发送数据的 CAN 口句柄指针
  * 		current1: 1 to 2 byte of send data ID 为 0x201 的电机电流控制值，范围[-16384,16384]
  * 		current2: 3 to 4 byte of send data ID 为 0x202 的电机电流控制值，范围[-16384,16384]
  * 		current3: 5 to 6 byte of send data ID 为 0x203 的电机电流控制值，范围[-16384,16384]
  * 		current4: 7 to 8 byte of send data ID 为 0x204 的电机电流控制值，范围[-16384,16384]
  * @retval void
  * @usage  you can choose the suitable one to use 
  *         according to the data type you want to transmit
  */

/****** Callback Function used in this files ******/
uint8_t fric_change_mode_flag=0;
/**
  * @brief  从 FIFO0 发生的 CAN 接收中断函数
  * @param  <-- Target_hcan -->
  *         发生中断的 CAN 句柄指针
  * @retval void
  * @usage
  */
 uint8_t last_fric_mode;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *Target_hcan)
{
	 static uint16_t fric_mode_count=400;
    //printf("CAN Callbacks successfully.\r\n");
  //接收数据存储区
	s_CAN_Message can1_rx_message,can2_rx_message;
	CAN_RxHeaderTypeDef Can1RxHeader,Can2RxHeader;
	if(Target_hcan -> Instance == CAN1)
	{
		    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can1RxHeader, can1_rx_message.Data);
	}
	if(Target_hcan -> Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0,&Can2RxHeader,can2_rx_message.Data );//从FIFO中接收消息至can2_rx_message.Data
		switch (Can2RxHeader.StdId)
		{
			case 0x300:
			{
				uiRx.pitch.bit[0] = can2_rx_message.Data[0];//pit轴电机角度
				uiRx.pitch.bit[1] = can2_rx_message.Data[1];
				uiRx.power.bit[0] = can2_rx_message.Data[2];//电容电压（转换单位后）（0-100）
				uiRx.power.bit[1] = can2_rx_message.Data[3];
				uiRx.chasMode = can2_rx_message.Data[4];//底盘模式
				uiRx.gimbalMode = can2_rx_message.Data[5];//云台模式
				uiRx.fricMode = can2_rx_message.Data[6];//摩擦轮状态
				if(last_fric_mode!=uiRx.fricMode)	fric_change_mode_flag=1;
				if(fric_change_mode_flag) 
				{
					--fric_mode_count;
				}
				if(fric_mode_count==0)
				{
					fric_change_mode_flag=0;
					fric_mode_count=600;
				}
				uiRx.visMode = can2_rx_message.Data[7];//自瞄模式

				uiRx.realPitData = (int32_t)uiRx.pitch.data;
				uiRx.realPowData = (int32_t)uiRx.power.data;
				last_fric_mode = uiRx.fricMode;
				break;
			}
			case 0x301:
			{
				
				uiRx.yaw.bit[0] = can2_rx_message.Data[0];//yaw轴码盘值
				uiRx.yaw.bit[1] = can2_rx_message.Data[1];
				uiRx.fric_spd.bit[0] = can2_rx_message.Data[2];//fric目标转速
				uiRx.fric_spd.bit[1] = can2_rx_message.Data[3];
				uiRx.fric_temp.bit[0] = can2_rx_message.Data[4];//fric温度
				uiRx.fric_temp.bit[1] = can2_rx_message.Data[5];
				uiRx.firc_error = can2_rx_message.Data[6];//摩擦轮错误标志位，就是有球卡喉咙的时候会显示
				uiRx.UI_sync_flag = can2_rx_message.Data[7];//UI刷新标志位，只有该值为1才会开始画UI，否则UI是空的
				uiRx.realYawData = (int16_t)uiRx.yaw.data;
				uiRx.realfricSpdData = (int32_t)uiRx.fric_spd.data;
				uiRx.realfricTempData = (uint32_t)uiRx.fric_temp.data;
				break;
			}
			
			default: break;
		}
//		 switch (Can2RxHeader.StdId)
//		 {
//			case 0x300:
//			{
//			break;
//			}
//		}
	}
}
	

void CAN_Send_bytes(uint32_t id,uint8_t data[8])
{
	s_CAN_Message tx_message;
  	CAN_TxHeaderTypeDef CanTxHeader;
    uint32_t TX_MailBOX = CAN_TX_MAILBOX0;
	CanTxHeader.StdId = id;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.RTR = CAN_RTR_DATA;
  	CanTxHeader.DLC = 0x08;
  	tx_message.Data[0]=data[0];
	tx_message.Data[1]=data[1];
	tx_message.Data[2]=data[2];
	tx_message.Data[3]=data[3];
    tx_message.Data[4]=data[4];
  	tx_message.Data[5]=data[5];
	tx_message.Data[6]=data[6];
  	tx_message.Data[7]=data[7];
    if(HAL_CAN_AddTxMessage(&hcan2, &CanTxHeader, tx_message.Data, &TX_MailBOX) != HAL_OK)
    {
        //printf("CAN_judge send failed!\r\n");
    }
		/*can1 send
		if(HAL_CAN_AddTxMessage(&hcan2, &CanTxHeader, tx_message.Data, &TX_MailBOX) != HAL_OK)
    {
        printf("CAN_judge send failed!\r\n");
    }
		*/
}
