#include "myTask02.h"
	uint8_t debugdata[8] = {1,2,3,4,5,6,7,8};


void StartTask02(void const * argument)
{  //portTickType task_start_time;
  /* Infinite loop */
	for(;;)
	{
	Start_RM_Task();
	//task_start_time = xTaskGetTickCount();
	//taskENTER_CRITICAL();
	//printf("task02ok");
	/*测试与A板CAN2通信是否好使*/
	//CAN_Send_bytes(0x105,debugdata);
	//taskEXIT_CRITICAL();
	//osDelayUntil(&task_start_time,2);
	osDelay(2);
	}
}
