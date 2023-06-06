#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include "can.h"
#include "pid.h"

extern CAN_HandleTypeDef hcan1;

typedef struct
{
    union
    {
        uint8_t     bit[2];
        short       data;
    }pitch;
    union
    {
        uint8_t     bit[2];
        short      data;
    }power;
    union
	{
		uint8_t     bit[2];
		short      data;
	}yaw;
	union
	{
		uint8_t     bit[2];
		short      data;
	}fric_spd;
	union
	{
		uint8_t     bit[2];
		short      data;
	}fric_temp;
    uint8_t         gimbalMode;
    uint8_t         chasMode;
    uint8_t         fricMode;
    uint8_t         visMode;
	uint8_t			firc_error;
    int32_t         realPitData;
    int32_t         realPowData;
	int32_t         realYawData;
	int32_t         realfricSpdData;
	int32_t         realfricTempData;

}UI_RX;

typedef struct
{
    uint8_t Data[10];
} s_CAN_Message;

void MY_CAN_Init(void);
void CAN_Enable(CAN_HandleTypeDef *Target_hcan);
void CANFilter_Enable(CAN_HandleTypeDef *Target_hcan);
void CAN_Send_bytes(uint32_t id,uint8_t data[8]);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *Target_hcan);

#endif  /* BSP_CAN_H_ */
