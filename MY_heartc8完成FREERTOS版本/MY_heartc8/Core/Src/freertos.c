/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stdio.h"
#include "max30102.h"
#include "max30102_fir.h"
#include "oled.h"
#include "ws2812.h"
#include "usart.h"
#include "tim.h"


int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
	return ch;
}

#define START_TASK_PRIO 		1
#define START_TASK_STACK_SIZE   128
TaskHandle_t   start_task_handler;
void start_task(void * pvParameters );

#define TASK1_PRIO 3
#define TASK1_STACK_SIZE 128
TaskHandle_t task1_handler;
void task1(void * pvParameters );

#define TASK2_PRIO 2
#define TASK2_STACK_SIZE 128
TaskHandle_t task2_handler;
void task2(void * pvParameters );
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t max30102_int_flag = 0; // 中断标志
float ppg_data_cache_RED[CACHE_NUMS] = {0}; // 缓存区
float ppg_data_cache_IR[CACHE_NUMS] = {0};  // 缓存区
uint16_t cache_counter = 0; // 缓存计数器
int touch_30102=0;
float max30102_data[2], fir_output[2];
uint16_t HeartRate = 0;
float SpO2 = 0; 

int heart_cycle=750;//心率周期
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void start_task(void * pvParameters )
{
  taskENTER_CRITICAL();
     /*******************************************/
  //OLED初始化
  OLED_Init();
  OLED_Clear();
  OLED_ShowCHinese24(0+6,0,0);
  OLED_ShowCHinese24(28+6,0,1);
  OLED_ShowNUM24(56+6,0,10);
  OLED_ShowNUM24(70+6,0,0);
  /*******************************************/
//心率检测模块相关初始化
  max30102_init();
  max30102_fir_init();
  printf("Max30102 Init\r\n");
/******************************************/
  xTaskCreate(task1,"task1",TASK1_STACK_SIZE,NULL,TASK1_PRIO,&task1_handler);
  xTaskCreate(task2,"task2",TASK2_STACK_SIZE,NULL,TASK2_PRIO,&task2_handler);
  
  vTaskDelete(NULL);
  taskEXIT_CRITICAL();
}

//ws2812及蜂鸣器控制 感觉touch_30102和heart_cycle需要进行保护，有个问题，想要保护信号就不能调用vTaskDelay，否则无法立刻返回

//但是要实现可变规律性ws2812闪烁，感觉只能vTaskDelay，或者因为这两个信号都在一个任务中改写，可以在那一部分进入临界区保护，退而求其次，然而实践发现
//由于MAX30102时间未知，导致临界区时间太长，直接系统崩溃

//再者，可以调用HAL_Delay,既能实现灯闪烁，又不影响调度，但是总的还是影响信号量的接收

//最后，感觉只能采用计数的方式，将延时时间往小改
/*void task1( void * pvParameters ) 
{                                 
    while(1)
    {
		if(touch_30102)
		{
		   rgb_show(17,RED);
		   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
		   vTaskDelay(heart_cycle/8);
			
			rgb_show(17,BLACK);
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
			vTaskDelay(heart_cycle/8*7);             //这种固定时间，再返回任务的时候不能立马切换到else部分
		}
		else
		{
		  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
		  rainbowCycle(1);
		  vTaskDelay(1);
		}
	}
}*/

//优化后的task1，无法实现二值信号量保护，因为task2中信号发送太慢了，导致led闪烁出现问题
//还是采用临界区代码保护吧，但是只保护赋值那一行
void task1( void * pvParameters ) 
{    
    uint32_t count=0;	
    while(1)
    {
		if(touch_30102)
		{
			count++;
			if(count<=heart_cycle/8)
			{
		      rgb_show(17,RED);
		      HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
			}
			else if(count>heart_cycle)
			  count=0;
			else
			{
			  rgb_show(17,BLACK);
			  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
			}	
			vTaskDelay(1);
		}
		else
		{
		  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
		  rainbowCycle(1);
		  vTaskDelay(1);
		}
	}
}

void task2( void * pvParameters )   //MAX30102控制，设置比task1低级
{
    while(1)
    {
	  //心率检测，完成OLED显示及心率周期计算
	// taskENTER_CRITICAL();               /* 进入临界区 */ //
     if(MAX30102_Get_DATA(&HeartRate,&SpO2,max30102_data,fir_output) == MAX30102_DATA_OK)
	 {
		//printf("心率：%d  次/min   ", HeartRate);
		if(HeartRate>=60&&HeartRate<=110)
		{
			taskENTER_CRITICAL();               /* 进入临界区 */ 
			heart_cycle=60000/HeartRate; //计算心率周期（ms）
			taskEXIT_CRITICAL();                //退出临界区
			if(HeartRate>=100)
			{
			 OLED_ShowNUM24(70+6,0,HeartRate/100);
			 OLED_ShowNUM24(86+6,0,HeartRate%100/10);
			 OLED_ShowNUM24(102+6,0,HeartRate%100%10);
			}
			else if(HeartRate>=60)
			{
			  OLED_ShowNUM24(70+6,0,HeartRate%100/10);
			  OLED_ShowNUM24(86+6,0,HeartRate%100%10);
			  OLED_ShowNUM24(102+6,0,11);	
			}
		}		
	 }
	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  xTaskCreate(start_task,"start_task",START_TASK_STACK_SIZE,NULL,START_TASK_PRIO,&start_task_handler);//创建初始化任务
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

