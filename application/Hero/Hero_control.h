#ifndef HERO_CTRL_H
#define HERO_CTRL_H

#include <cmsis_os.h>
#include <main.h>
#include <math.h>
#include "struct_typedef.h"

/*----------- bsp -------------*/
#include "hero_can.h"
/*--------- devices -----------*/
#include "nuc_interface.h"
#include "laser.h"
#include "RM_Judge.h"
/*----------- Mode ------------*/
#include "Chassis.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "Monitor.h"
#include "Mode_Switch.h"
/*---------- Thread -----------*/
#include "ConfigHeroTask.h"
/*-------- Variables ----------*/
#include "Hero_variables.h"
/*-----------------------------*/

#endif //HERO_CTRL_H

