#ifndef ROBOT_H
#define ROBOT_H

#include "struct_typedef.h"
/*------------------------ define ----------------------------*/
#define _NONE_ROBOT_ 0
#define _HERO_ 1
/*----------------- 选择机器人 ----------------*/
#define _ROBOT_ _HERO_
/*-------------- 选择机器人头文件 --------------*/
#if _ROBOT_ == _HERO_
#include "Hero_control.h"
#endif // _HERO_


/*------------------- function declaration ---------------------*/
void RobotInit(void);
void RobotDealUSBData(uint8_t *Buf);
/*--------------------------------------------------------------*/
#endif //ROBOT_H

