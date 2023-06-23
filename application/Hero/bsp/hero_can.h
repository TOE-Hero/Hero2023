#ifndef HERO_CAN_H
#define HERO_CAN_H

#include "stm32f4xx.h"
#include "Hero_control.h"
/*---------------------- CAN 发送状态结构体 --------------*/
typedef struct
{
	uint8_t pit;
	uint8_t yaw;
	uint8_t shoot;
	uint8_t chassis;
}s_can_send_state_t;

extern s_can_send_state_t can_send_state;
/*---------------------- CAN 发送\接收 结构体数组 --------------*/

typedef union{
	uint8_t arr2[2];
	int16_t integer;
	uint16_t u_integer;
}u_data_16bit;

typedef union{
	uint8_t arr4[4];
	int integer;
	float f;
	uint32_t u_integer;
}u_data_32bit;
/*----------------------------- 函数声明 ----------------------*/
void UI_API(void);
/*------------------------------------------------------------*/
#endif  /* HERO_CAN_H */

