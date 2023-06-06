#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx.h"

#define DEBUG_USART_PORT       UART1
#define DEBUG_USART_HANDLE     huart1
#define DEBUG_BUFFER_LENGTH    1
extern uint8_t uart6_rx_buffer;  //作为串口缓冲区 
extern uint8_t uart1_rx_buffer[30];//作为串口缓冲区 
void MY_USART_Init(void);
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_address);
void Usart_SendString(uint8_t* str);

#endif
