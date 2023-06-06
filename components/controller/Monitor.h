#ifndef MONITOR_H
#define MONITOR_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Mode_Switch.h"
//FPS结构体
typedef struct
{
	uint16_t dbus;
	uint16_t RF_motor;
	uint16_t LF_motor;
	uint16_t LB_motor;
	uint16_t RB_motor;
	uint16_t yaw;
	uint16_t pitch;
	uint16_t trans;
	uint16_t fric_l;
	uint16_t fric_r;
	uint16_t board_imu;//the imu in stm32f427IIHx typeC
	uint16_t judge;
	uint16_t cap_board;
	uint16_t pc;
} s_FPS_monitor;

void start_Monitor(void);
void final_Monitor(void);
uint8_t isProcessOn(void);
void isInit_Ok(void);
void Monitor_FPS_state(s_FPS_monitor* final_fps);
void buzzer_Monitor_FPS_state(s_FPS_monitor* final_fps);

#endif

