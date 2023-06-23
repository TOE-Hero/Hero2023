#include "Monitor_task.h"
#include "Monitor.h"
#include "FreeRTOS.h"
#include "bsp_buzzer.h"
#include <cmsis_os.h>
#include <main.h>
/*************************** extern declaration ***********************************/
extern s_FPS_monitor	finalFps;
extern uint16_t fps_gimbal_task;
int final_fps_gimbal;
/**********************************************************************************/
void Monitor_task(void const * argument)
{
  /* USER CODE BEGIN Monitor_task */
  /* Infinite loop */
	while(1)
	{
		ModeSwitch();//模式选择
		static TickType_t monitorLastWakeTime;
		for(;;)
		{
			
			start_Monitor();//FPS赋值给startFPS
			monitorLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&monitorLastWakeTime, 1000);//在此阻塞1000ms，FPS依旧在加，startFPS赋值被阻塞，以便下一次计算差值
			final_fps_gimbal = fps_gimbal_task;
			fps_gimbal_task = 0;
			
			final_Monitor();//计算每秒进入CAN的次数
			Monitor_FPS_state(&finalFps);
			//buzzer_Monitor_FPS_state(&finalFps);
		}
	}
  /* USER CODE END ChassisTop_task */

}

