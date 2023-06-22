#ifndef HERO_CAN_H
#define HERO_CAN_H

#include "stm32f4xx.h"
#include "Hero_control.h"

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

