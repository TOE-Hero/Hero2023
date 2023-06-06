#include "myTask01.h"

void myTask01_Start(void const * argument)
{   portTickType task_start_time;
  /* Infinite loop */
  for(;;)
  { task_start_time = xTaskGetTickCount();
    taskENTER_CRITICAL();
		
		printf("task01ok");

		//printf("[\t power:%f\r\n]",Judge_PowerHeatData.chassis_power);
		//printf("[\t current:%d\r\n]",Judge_PowerHeatData.chassis_current);
	  taskEXIT_CRITICAL();
    osDelayUntil(&task_start_time,5);
	 }
 }

	