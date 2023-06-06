/**
  * @file      bsp_usart.c
  * @version   0.0
  * @data      2020/10/17
  * @author    Jack
  *
  * @usage     To initialze the USART port and enable them
  * @hardware  STM32 F1 Series
  * @note      In this version no DMA was used
  */

/****** Macros used in this file ******/
#include "usart.h"
#include "bsp_usart.h"
#include "STMGood.h"
#include <stdio.h>
//#include "uart_DBUS.h"
#include "judge.h"
/****** Create a buffer for the Recived data ******/
/***************** 创建串口缓存区 *****************/
uint8_t uart1_rx_buffer[30] = {0};  //作为串口缓冲区 
uint8_t uart6_rx_buffer;  //作为串口缓冲区 


/******** Functions declared in this file ********/
/**
* @brief  Software peripherals USART initialization
* @param  None
* @retval None
*/
void MY_USART_Init(void)
{
    // 使能串口7
    USART_Enable(&huart1, uart1_rx_buffer);
	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer,1);

//  USART_Enable(&huart6, &uart6_rx_buffer);
//USART_Enable(&huart6, &uart6_rx_buffer);
	USART_Enable(&huart6, &uart6_rx_buffer);

  // HAL_UART_Receive_DMA(&huart6, &uart6_rx_buffer,1);
}

/**
* @brief  使能串口
* @param  <-- huart -->
*         待使能的串口句柄
* @param  <-- buffer_address -->
*         接收串口数据的缓存地址
* @retval None
*/
void USART_Enable(UART_HandleTypeDef* huart, uint8_t* buffer_address)
{
    // 初始化使能目标串口的接收中断，；里面看到使能了3类接受中断，其中一个是RXNE接收数据中断
    HAL_UART_Receive_IT(huart, buffer_address, 1);
    // 打开目标串口的错误中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
}

/**
  * @brief  对printf函数的重定义
  * @param
  * @retval
  * @usage
  */
int fputc(int ch, FILE *f)
{
    while((USART1->SR&0X40)==0);//发送状态寄存器是否转移完
    USART1->DR = (uint8_t) ch;  //数据寄存器赋给数据
   return ch;
 }

/**
  * @brief  通过串口发送字符串
  * @param  要发送的字符串地址
  * @retval None
  */
void Usart_SendString(uint8_t* str)
{
    unsigned int k = 0;
    do
    {
        HAL_UART_Transmit(&DEBUG_USART_HANDLE, (uint8_t*)(str + k), 1, 1000);
        k++;
    }
    while(*(str + k) != '\0');
}

/******** Callback Functions used USART ********/
/**
  * @brief  重定义串口接收中断回调函数
  * @param  <-- huart -->
  *         系统自动填入的发生中断的串口句柄指针
  * @retval None
  * @note   未开启DMA用此回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)

{   
	if(huart->Instance == USART1)  
		{
			 Dealdata(uart1_rx_buffer[0]);
			//printf("pidok");

			__HAL_UART_CLEAR_PEFLAG(&huart1);
			 //printf("pidok");
		 // 重新使能串口接收中断
		 HAL_UART_Receive_IT(&huart1, uart1_rx_buffer, 1);
			//HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
		}
			
  if(huart->Instance==USART6)
		{
			__HAL_UART_CLEAR_PEFLAG(&huart6);
			JudgeData(uart6_rx_buffer);
			HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer, 1);
		 // HAL_UART_Receive_DMA(&huart6,&uart6_rx_buffer,1);		
	  }
}



 
/**
  * @brief  串口的错误中断回调函数
  * @param  系统自动填入的发生中断的串口地址句柄指针
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    // 若检测到错误发生则清除错误标志位，清空SR、DR寄存器
    if (huart -> ErrorCode == HAL_UART_ERROR_ORE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

}

