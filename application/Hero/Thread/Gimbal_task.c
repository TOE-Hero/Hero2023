#include "Gimbal_task.h"
#include "struct_typedef.h"
#include "Gimbal.h"
#include <cmsis_os.h>
#include <main.h>
#include "bsp_can.h"
#include "INS_task.h"
#include "Mode_Switch.h"
#include "nuc_interface.h"
/*------------------------ define ----------------------------*/

#define SendPcData_FREQUENCE	10//发送PC信息频率(单位：Hz)
/*------------------ static declaration  ---------------------*/

static uint8_t 					pcTick = 0;//PC计数值
/*------------------ extern declaration  ---------------------*/

extern volatile uint8_t 		gyro_flag;//陀螺仪标志位
extern uint8_t 					RobotId;
/*-------------------- global variable  ----------------------*/

uint16_t 						fps_gimbal_task = 0;//云台线程帧率计数值
/*------------------------------------------------------------*/


/**
  * @brief          云台任务,Yaw轴电机&Pitch轴电机
  * @param[in]      
  * @retval        
  */
void Gimbal_task(void const * argument)
{
  static TickType_t gimLastWakeTime;
	
  while(1)
    {
		fps_gimbal_task++;
		gimLastWakeTime = xTaskGetTickCount();
		pcTick++;
		if(!(pcTick % (500/SendPcData_FREQUENCE)))
		{
			switch (RobotId)
			{
				case 1://red，红方英雄
					if(PRESS_E)
					{
						sendMessageToPc_Python(1,3);
					}
					else
					{
						sendMessageToPc_Python(1,0);
					}
					//sendMessageToPc(1);
				break;
				case 101://blue，蓝方英雄
					if(PRESS_E)
					{
						sendMessageToPc_Python(101,3);
					}
					else
					{
						sendMessageToPc_Python(101,0);
					}
					//sendMessageToPc(101);
				break;
				default:
				break;
			}
		}
		taskENTER_CRITICAL();//中断保护开启(屏蔽中断)
		ModeSwitch();//模式选择
		if(gyro_flag)
		{
			Gimbal_Move();
		}
		else
		{
			CANTx_SendCurrent(&hcan1,0x1FF, 0 , 0 , 0 , 0 );
			CANTx_SendCurrent(&hcan2,0x1FF, 0 , 0 , 0 , 0 );
		}
		
		taskEXIT_CRITICAL();//中断保护关闭(开启中断)
		vTaskDelayUntil(&gimLastWakeTime, 2);//500Hz
    }
	

}

