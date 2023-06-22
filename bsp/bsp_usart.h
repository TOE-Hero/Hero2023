#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx.h"
#include <stdio.h>
#include "struct_typedef.h"
#include "usart.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

/************************* extern *****************************/
extern uint8_t uart7_rx_buffer[30];

/******************** function declaration ********************/
void MY_USART_Init(void);
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_address,int length);
int fputc(int ch, FILE* f);
void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart);
/**************************************************************/
#endif
