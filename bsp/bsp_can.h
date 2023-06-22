#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "stm32f4xx.h"

/******************* 类型声明 ******************/
typedef struct{
    uint8_t Data[8];
} s_CAN_Message;
/******************* 函数声明 ******************/
void MY_CAN_Init(void);
void CAN_Enable(CAN_HandleTypeDef *Target_hcan);
uint8_t CANTx_SendCurrent(CAN_HandleTypeDef *Target_hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void CAN_Send_bytes(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t data[8]);

#endif  /* BSP_CAN_H */

