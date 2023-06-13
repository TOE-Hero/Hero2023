#ifndef __SHOOT_H
#define __SHOOT_H

#include "motor.h"

#define TRANS_REDUCTION_RATIO   REDUCTION_RATIO_3508_1_19//宏定义拨弹盘电机减速比
#define TRANS_STEP			    MOTOR_ONE_TURN_SCALE_3508/6*TRANS_REDUCTION_RATIO
// #define TRANS_STEP			    26215.6//8191/6*19.203208556

void Shoot_Init(void);
void Shoot_Move(void);

#endif
