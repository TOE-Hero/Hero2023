#include "Shoot_task.h"

#include <cmsis_os.h>
#include <main.h>
#include "bsp_can.h"
#include "pid.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "dji_motor.h"
#include "bsp_remote.h"
#include "math.h"
#include "Shoot.h"

/**
  * @brief          射击任务,2速度环m3508 & 1串级环m3508(1:19)
  * @param[in]      argument: NULL
  * @retval         none
  */

void Shoot_task(void const * argument)
{
	
	static TickType_t shootLastWakeTime;
    while(1)
    {
		shootLastWakeTime = xTaskGetTickCount();
    ModeSwitch();//模式选择
		taskENTER_CRITICAL();//中断保护

		Shoot_Move();
		
		taskEXIT_CRITICAL();
		vTaskDelayUntil(&shootLastWakeTime, 1);
    }
}




