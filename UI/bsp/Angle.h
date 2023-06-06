#ifndef __ANGLE_H__
#define __ANGLE_H__

#include "stm32f4xx.h"
typedef struct
{
    float aglOriginal;               //电调初始角位置
	float aglNow;                    //电调当前角度值
	float aglLast;                    //电调上次返回速度值
	
	float ctrOut;                      //通过公式计算后得到的电机的实际转角
}motor_AngleTypeDef;         //电机转角计算公式结构体定义

float motorAngle_deal(int16_t angle);

#endif

