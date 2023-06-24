#include <cmsis_os.h>
#include <main.h>
#include "struct_typedef.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "Mode_Switch.h"
#include "Chassis.h"
/*************************** extern declaration ***********************************/

extern volatile uint8_t			gyro_flag;//陀螺仪标志位
extern chassisMove_t 			s_chassisMove;
extern uint8_t         			capExpect_txBuff[8];
/**********************************************************************************/

/**
  * @brief          底盘任务
  * @param[in]      argument: NULL
  * @retval         none
  */
void Chassis_task(void const * argument)
{
/******************************** static variable **********************************/

static TickType_t 	chassisLastWakeTime;
static uint32_t 	SuperCAP_CAN_Send_Count;//控制给电容控制板发送CAN的帧率
static uint32_t 	SuperCAPDataGoodCount;
/**********************************************************************************/
    while(1)
    {
		chassisLastWakeTime = xTaskGetTickCount();
		taskENTER_CRITICAL();
		ModeSwitch();//模式选择
		if(gyro_flag)
		{
		Chassis_Move();
		}
		else
		{
		// 电机发送0电流
		CANTx_SendCurrent(&hcan1,0x200, 0 , 0 , 0 , 0 );
		}
		// 超级电容发送
		if(SuperCAP_CAN_Send_Count % 10 == 0)
		{
		CAN_Send_bytes(&hcan1,0x101,capExpect_txBuff);//给电容控制板发送控制信息
		SuperCAP_CAN_Send_Count = 0;
		}
		SuperCAPDataGoodCount++;
		if(SuperCAPDataGoodCount == 200)
		{
			SuperCAPDataGoodCount = 0;
		}
		taskEXIT_CRITICAL();

		vTaskDelayUntil(&chassisLastWakeTime, 5);//200Hz

    }
}
