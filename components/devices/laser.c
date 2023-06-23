#include "bsp_timer.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

#define LASER_TIMER    htim3
#define LASER_TIMER_CHANNEL TIM_CHANNEL_3
/**
 * @brief 激光定时器初始化
 * 
 */
void LaserInit(void)
{
	HAL_TIM_Base_Start(&LASER_TIMER);
	HAL_TIM_PWM_Start(&LASER_TIMER, LASER_TIMER_CHANNEL);
}
/**
 * @brief 打开激光
 * 
 */
void LaserOn(void)
{
    __HAL_TIM_SetCompare(&LASER_TIMER, LASER_TIMER_CHANNEL, 8399);
}
/**
 * @brief 关闭激光
 * 
 */
void LaserOff(void)
{
    __HAL_TIM_SetCompare(&LASER_TIMER, LASER_TIMER_CHANNEL, 0);
}

