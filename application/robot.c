#include "robot.h"
#if _ROBOT_ == _HERO_
#include "Hero_control.h"
#endif // _HERO_
/**
 * @brief 机器人总变量和模式初始化
 * 
 */
void RobotInit(void)
{
    #if _ROBOT_ == _HERO_
    LaserInit();
    ModeInit();
    Chassis_Init();
    Shoot_Init();
    Gimbal_Init();
    ramp_init(&ramp_w);
	ramp_init(&ramp_s);
	ramp_init(&ramp_a);
	ramp_init(&ramp_d);
	ramp_init(&ramp_c);
	ramp_init(&ramp_cFllow);
    #endif // _HERO_
}
/**
 * @brief 机器人处理C板USB信息函数
 * 
 * @param Buf 
 */
void RobotDealUSBData(uint8_t *Buf)
{
    #if _ROBOT_ == _HERO_
    DealPcData(&pcData,Buf);
    #endif // _HERO_
}
/**
 * @brief 机器人线程初始化函数
 * 
 */
void RobotTaskInit(void)
{
    #if _ROBOT_ == _HERO_
    HERO_FREERTOS_Init();
    #endif // _HERO_
}