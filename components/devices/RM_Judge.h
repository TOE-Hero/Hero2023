#ifndef __RM_JUDGE_H
#define __RM_JUDGE_H

#include "motor.h"

typedef struct
{
    
	uint16_t coolingheat_limit;//（枪管）最大冷却值
	uint16_t coolingheat;//（枪管）实时热量
	uint16_t coolingheat_every_second;// （枪管）每秒冷却值
	
} s_rm_judge_shoot_data_t;//rm裁判系统发射机构数据结构体类型

#endif//__RM_JUDGE_H
