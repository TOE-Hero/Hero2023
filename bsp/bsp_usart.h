#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx.h"
#include <stdio.h>
#include "struct_typedef.h"
#include "usart.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

extern uint8_t uart7_rx_buffer[30];


void MY_USART_Init(void);
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_address,int length);
int fputc(int ch, FILE* f);
extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart);



void USART_IDLE_Init(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t dma_buf_num);
void USART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
void RC_Restart(uint16_t dma_buf_num);

__weak void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart);
#endif
