#include "bsp_timer.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

#define LASER_TIMER    htim3
#define LASER_TIMER_CHANNEL TIM_CHANNEL_3

void Timer_Init(void)
{
	HAL_TIM_Base_Start(&LASER_TIMER);
	HAL_TIM_PWM_Start(&LASER_TIMER, LASER_TIMER_CHANNEL);
}

void laser_on(void)
{
    __HAL_TIM_SetCompare(&LASER_TIMER, LASER_TIMER_CHANNEL, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&LASER_TIMER, LASER_TIMER_CHANNEL, 0);
}

