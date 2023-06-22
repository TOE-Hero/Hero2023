/**
  * @file      bsp_usart.c
  * @version   0.0
  * @data      2020/10/17
  * @author    Jack
  *
  * @usage     To initialze the USART port and enable them
  * @hardware  STM32 F4 Series
  * @note      In this version no DMA was used
  */
/****** Macros used in this file ******/
#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "STMGood.h"
#include "printf.h"
/*----------------* define declaration --------------*/
#define USART1_DEBUG 1
#define USART6_DEBUG 0
#define USART_ALL_DEBUG 0


#define NUC_USART USART6
#define NUC_USART_HANDLE huart6
#define NUC_DMA_USART hdma_usart6_rx
#define NUC_REC_LEN 16u

/*---------------- extern declaration ----------------*/
extern DMA_HandleTypeDef hdma_usart6_rx;
/*------------------- 创建串口缓存区 ------------------*/
uint8_t usart1_rx_buffer[30] = {0};  // For debug
uint8_t usart6_rx_buffer[30] = {0};
// extern uint8_t visionData_buff[16u];
/*------------ Functions declared in this file ------*/
void MY_UART_ENABLE_DMA_IDLE(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma, uint8_t *buff, uint8_t bufflen);
/*-------------------------------------------------------------------------------------------------------------*/
/**
* @brief  Software peripherals USART initialization，放到main.c里
* @param  None
* @retval None
*/
void MY_USART_Init(void)
{
    // 使能串口1
	HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, 1);
  HAL_UART_Receive_IT(&huart6, usart6_rx_buffer, 1);
//	MY_UART_ENABLE_DMA_IDLE(&huart6, &hdma_usart6_rx, visionData_buff, 16u);//原来接收pc串口的方式
}

/**
  * @brief  串口接收中断回调函数
  * @param  UART_HandleTypeDef* huart:系统自动填入的发生中断的串口句柄指针
  * @retval None
  * @note   未开启DMA用此回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
/*----------------------------------------------- 串口接收 ----------------------------------------------------*/
	if(huart -> Instance == USART1)
	{
		// 如果格式符合上位机的格式，则对相应变量进行赋值（p，i，d）
		Dealdata(usart1_rx_buffer[0]);
		//清除中断标志位
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		// 重新使能串口接收中断
		HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, 1);
	}
  	if(huart -> Instance == USART6)
	{
		// 如果格式符合上位机的格式，则对相应变量进行赋值（p，i，d）
		Dealdata(usart6_rx_buffer[0]);
		//清除中断标志位
		__HAL_UART_CLEAR_PEFLAG(&huart6);
		// 重新使能串口接收中断
		HAL_UART_Receive_IT(&huart6, usart6_rx_buffer, 1);
	}
}


/*---------------------------------- 弃用函数，DMA串口接收PC数据，有需要可以改一改用 -----------------------------*/
void MY_UART_ENABLE_DMA_IDLE(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma, uint8_t *buff, uint8_t bufflen)
{
	HAL_DMA_Start_IT(hdma, (uint32_t)huart->Instance->DR,(uint32_t)buff, bufflen);
	CLEAR_BIT(hdma->Instance->CR,DMA_IT_HT);
	huart->Instance->CR3 |= USART_CR3_DMAR;
	__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
	HAL_UART_Receive_DMA(huart, buff, bufflen);
	__HAL_UART_ENABLE_IT(huart,UART_IT_ERR);
}


void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart)//接收pc串口处理数据与中断函数，放到中断里执行，后来改用USB
{
	if(huart->Instance == NUC_USART)
	{
			if(__HAL_UART_GET_FLAG(&NUC_USART_HANDLE,UART_FLAG_IDLE) != RESET)
			{
			__HAL_UART_CLEAR_IDLEFLAG(&NUC_USART_HANDLE);
			HAL_UART_DMAStop(&NUC_USART_HANDLE);
			// HAL_UART_Receive_DMA(&NUC_USART_HANDLE,visionData_buff,NUC_REC_LEN);
			CLEAR_BIT(hdma_usart6_rx.Instance->CR,DMA_IT_HT);
			}
	}
}

/*----------------------------------------------- 串口重定向 ----------------------------------------------------*/
#ifdef __CC_ARM //Keil编译器
#if	USART1_DEBUG == 1
/**
  * @brief  对printf函数的重定义(USART1)
  * @param
  * @retval
  * @usage
  */
int fputc(int ch, FILE* f)
{
    /* 发送一个字节数据到串口 DEBUG_USART_PORT */
    // DR, SR 寄存器并不依附于句柄结构体，而是设备结构体
    while(((USART1 -> SR) & 0X40) == 0);//发送状态寄存器是否转移完
    USART1 -> DR = (uint8_t) ch;//数据寄存器赋给数据
    return ch;
}
#endif

#if	USART6_DEBUG == 1
int fputc(int ch, FILE* f)
{
    /* 发送一个字节数据到串口 DEBUG_USART_PORT */
    // DR, SR 寄存器并不依附于句柄结构体，而是设备结构体
    while(((USART6 -> SR) & 0X40) == 0);//发送状态寄存器是否转移完
    USART6 -> DR = (uint8_t) ch;//数据寄存器赋给数据
    return ch;
}
#endif
#endif

#ifdef __GNUC__ //arm-none-gcc-eabi编译器
#if	USART1_DEBUG == 1
/**
  * @brief  对printf函数的重定义(USART1)
  * @param  char character
  * @retval
  */
void _putchar(char character)
{
    /* 发送一个字节数据到串口 DEBUG_USART_PORT */
    // DR, SR 寄存器并不依附于句柄结构体，而是设备结构体
    while(((USART1 -> SR) & 0X40) == 0);//发送状态寄存器是否转移完
    USART1 -> DR = (uint8_t) character;//数据寄存器赋给数据
}
#endif
#if	USART6_DEBUG == 1
/**
  * @brief  对printf函数的重定义(USART1)
  * @param  char character
  * @retval
  */
void _putchar(char character)
{
    /* 发送一个字节数据到串口 DEBUG_USART_PORT */
    // DR, SR 寄存器并不依附于句柄结构体，而是设备结构体
    while(((USART6 -> SR) & 0X40) == 0);//发送状态寄存器是否转移完
    USART6 -> DR = (uint8_t) character;//数据寄存器赋给数据
}
#endif

#if USART_ALL_DEBUG == 1
/**
  * @brief  对printf函数的重定义(USART1 && USART6)
  * @param  char character
  * @retval
  */
void _putchar(char character)
{
    /* 发送一个字节数据到串口 DEBUG_USART_PORT */
    // DR, SR 寄存器并不依附于句柄结构体，而是设备结构体
    while((((USART1 -> SR) & 0X40) == 0) && (((USART6 -> SR) & 0X40) == 0));//发送状态寄存器是否转移完
    USART1 -> DR = (uint8_t) character;//数据寄存器赋给数据
    USART6 -> DR = (uint8_t) character;
}
#endif // DEBUG

#endif

