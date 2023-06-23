/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId GIMBALHandle;
osThreadId CHASSISHandle;
osThreadId SHOOTHandle;
osThreadId MONITORHandle;
osThreadId PRINTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void Gimbal_task(void const * argument);
void Chassis_task(void const * argument);
void Shoot_task(void const * argument);
void Monitor_task(void const * argument);
void Print_task(void const * argument);

extern void HERO_FREERTOS_Init(void);
void HERO_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void HERO_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* definition and creation of GIMBAL */
  osThreadDef(GIMBAL, Gimbal_task, osPriorityHigh, 0, 512);
  GIMBALHandle = osThreadCreate(osThread(GIMBAL), NULL);

  /* definition and creation of CHASSIS */
  osThreadDef(CHASSIS, Chassis_task, osPriorityHigh, 0, 512);
  CHASSISHandle = osThreadCreate(osThread(CHASSIS), NULL);

  /* definition and creation of SHOOT */
  osThreadDef(SHOOT, Shoot_task, osPriorityHigh, 0, 512);
  SHOOTHandle = osThreadCreate(osThread(SHOOT), NULL);

  /* definition and creation of MONITOR */
  osThreadDef(MONITOR, Monitor_task, osPriorityLow, 0, 128);
  MONITORHandle = osThreadCreate(osThread(MONITOR), NULL);

  /* definition and creation of PRINT */
  osThreadDef(PRINT, Print_task, osPriorityIdle, 0, 1024);
  PRINTHandle = osThreadCreate(osThread(PRINT), NULL);

}

/* USER CODE BEGIN Header_Gimbal_task */
/**
* @brief Function implementing the Task01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_task */
__weak void Gimbal_task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_task */
}

/* USER CODE BEGIN Header_Chassis_task */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_task */
__weak void Chassis_task(void const * argument)
{
  /* USER CODE BEGIN Chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_task */
}

/* USER CODE BEGIN Header_Shoot_task */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_task */
__weak void Shoot_task(void const * argument)
{
  /* USER CODE BEGIN Shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_task */
}

/* USER CODE BEGIN Header_Monitor_task */
/**
* @brief Function implementing the Task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Monitor_task */
__weak void Monitor_task(void const * argument)
{
  /* USER CODE BEGIN Monitor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Monitor_task */
}

/* USER CODE BEGIN Header_Print_task */
/**
* @brief Function implementing the Task06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Print_task */
__weak void Print_task(void const * argument)
{
  /* USER CODE BEGIN Print_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Print_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */
