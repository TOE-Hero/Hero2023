#include "myTask03.h"

fp32 gyro[3], accel[3], temp;
CHAS_IMU chas;
void StartTask03(void const * argument)
{  
  /* Infinite loop */
	for(;;)
	{ 
      BMI088_read(gyro, accel, &temp);
      chas.spd.data = gyro[2];
	/*debug*/
    //printf("task03ok");
		
	/*chassis_power test*/
    //printf("current=%d    ",s_chassis_motor.out_current);
    //printf("back_speed=%d \r\n   ",s_chassis_motor.back_speed);
    //printf("target_speed=%f\r\n   ",s_chassis_motor.target_speed);
    //printf("real_power=%f\r\n", Judge_PowerHeatData.chassis_power);
    //printf("buffer=%d\r\n", Judge_PowerHeatData.chassis_power_buffer);
    //printf("Logic.turn=%d\r\n",Logic.turn);
    osDelay(2);
	}
}
