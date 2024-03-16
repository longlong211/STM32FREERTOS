/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
//#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define  QUEUE_LEN    4   /* 队列的长度，�?大可包含多少个消�? */
#define  QUEUE_SIZE   4   /* 队列中每个消息大小（字节�? */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t Test_Queue =NULL;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTaskLED1Handle;
osThreadId myTaskUSARTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskLED1(void const * argument);
void StartTaskUSART(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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
    /* 创建Test_Queue */
  Test_Queue = xQueueCreate((UBaseType_t ) QUEUE_LEN,/* 消息队列的长度 */
                            (UBaseType_t ) QUEUE_SIZE);/* 消息的大小 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTaskLED1 */
  osThreadDef(myTaskLED1, StartTaskLED1, osPriorityNormal, 0, 128);
  myTaskLED1Handle = osThreadCreate(osThread(myTaskLED1), NULL);

  /* definition and creation of myTaskUSART */
  osThreadDef(myTaskUSART, StartTaskUSART, osPriorityNormal, 0, 128);
  myTaskUSARTHandle = osThreadCreate(osThread(myTaskUSART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskLED1 */
/**
* @brief Function implementing the myTaskLED1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED1 */
void StartTaskLED1(void const * argument)
{
  /* USER CODE BEGIN StartTaskLED1 */
  /* Infinite loop */
    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  uint32_t r_queue;	/* 定义一个接收消息的变量 */

  /* Infinite loop */
  for(;;)
  {

    /* 队列读取（接收），等待时间为一直等待 */
    xReturn = xQueueReceive( Test_Queue,    /* 消息队列的句柄 */
                             &r_queue,      /* 发送的消息内容 */
                             portMAX_DELAY); /* 等待时间 一直等 */
								
		if(pdPASS == xReturn)
		{
      if(r_queue==1)
      LED1_ON;
      else
      LED1_OFF;
			//printf("触发中断的是 KEY%d !\n",(uint8_t)r_queue);
      HAL_UART_Transmit(&huart1,(uint8_t*)&r_queue,1,100);
		}
		else
		{
			//printf("数据接收出错\n");
		}
		
   
  }

  /* USER CODE END StartTaskLED1 */
}

/* USER CODE BEGIN Header_StartTaskUSART */
/**
* @brief Function implementing the myTaskUSART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUSART */
void StartTaskUSART(void const * argument)
{
  /* USER CODE BEGIN StartTaskUSART */

  for(;;)
{   uint8_t TxData[8]= "abds344";
    //  printf("Characters: %s\n", TxData);
    HAL_UART_Transmit(&huart1,(uint8_t*)&TxData,8,100);

  osDelay(500);
}

  
  //  uint8_t TxData[8]= "1111111";
  //  HAL_UART_Transmit(&huart1,(uint8_t*)&TxData,8,100);
    
 /*  LED2_OFF;
  
     LED2_ON;
      osDelay(500);*/ 

  /* USER CODE END StartTaskUSART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
