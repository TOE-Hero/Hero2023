/**
 ******************************************************************************
 * @FilePath: \for_infantry\App\nuc_interface.h
 * @Brief: 
 * @Date: 2021-01-21 18:56:28
 * @Author: Rio
 ******************************************************************************
 */
#ifndef __NUC_INTERFACE__H__
#define __NUC_INTERFACE__H__

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"

typedef struct
{
    union
    {
        uint8_t          bit[2];
        bool             isExist;
    }target;
    union
    {
        uint8_t          bit[4];
        float            data;
    }x;
    union
    {
        uint8_t          bit[4];
        float            data;
    }y;
    union
    {
        uint8_t          bit[4];
        float            data;
    }z;
    union
    {
        uint8_t          bit[2];
        unsigned char    data;
    }checkCode;

}su_PC_DATA;

typedef struct
{
    /**视觉**/
	float 	visYawTarget;//YAW轴视觉目标值（视觉）
	float 	visPitTarget;//PIT轴视觉目标值（视觉）
}s_vision_t;
/*************************************** extern *******************************************/

extern su_PC_DATA pcData;
extern s_vision_t visTarget_t;//视觉目标值，传给PID目标位置
/*********************************** function declaration *********************************/
void sendMessageToPc(uint8_t selectRB);
void DealPcData(su_PC_DATA*pcdata, uint8_t *visionBuf);
void sendMessageToPc_Python(uint8_t selectRB,uint8_t visionMode);
#endif
