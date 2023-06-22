#include "robot.h"
#if _ROBOT_ == _HERO_
#include "Hero_control.h"
#endif // _HERO_

void RobotInit(void)
{
    #if _ROBOT_ == _HERO_
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

void RobotDealUSBData(uint8_t *Buf)
{
    #if _ROBOT_ == _HERO_
    DealPcData(&pcData,Buf);
    #endif // _HERO_
}