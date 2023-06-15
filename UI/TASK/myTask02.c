#include "myTask02.h"
	uint8_t debugdata[8] = {1,2,3,4,5,6,7,8};


void StartTask02(void const * argument)
{  //portTickType task_start_time;
  /* Infinite loop */
	for(;;)
	{
		Start_RM_Task();
		
		osDelay(2);
	}
}
