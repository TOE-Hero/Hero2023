/**
 ******************************************************************************
 * @FilePath: \Hero\Lib\ramp.h
 * @Brief: 
 * @Date: 2021-04-20 08:39:27
 * @Author: Rio
 ******************************************************************************
 */
#ifndef _RAMP_H_
#define _RAMP_H_

#include "main.h"

#define RAMP_INIT   130

typedef struct{
	int32_t count;
	int32_t scale;
	int16_t interval;
	float out;
}ramp_t;

#define RAMP_DEFAULT_INIT \
{ \
	.count = 0, \
	.scale = 600, \
	.interval = 0, \
	.out = 0, \
	}

#define RAMP_SLOW_INIT \
{ \
	.count = 0, \
	.scale = 1200, \
	.interval = 0, \
	.out = 0,\
	}
void ramp_All_init(void);
void ramp_init1(ramp_t *ramp);
float ramp_cal(ramp_t *ramp);
extern ramp_t ramp_c,ramp_w,ramp_s,ramp_a,ramp_d,ramp_cFllow;

#endif
