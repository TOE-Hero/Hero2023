#ifndef MYTASK_03_H
#define MYTASK_03_H
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "RM_Task.h"
#include "bsp_can.h"
#include "stdio.h"
#include "Judge.h"
#include "BMI088driver.h"

typedef struct
{
    union
    {
        float data;
        uint8_t bit[4];
    }spd;
    
}CHAS_IMU;
extern CHAS_IMU chas;
extern fp32 gyro[3];








#endif
