/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "max30102.h"
#include "max30102_fir.h"
#include "oled.h"
#include "ws2812.h"
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
	return ch;
}
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t max30102_int_flag = 0; // 中断标志
float ppg_data_cache_RED[CACHE_NUMS] = {0}; // 缓存区
float ppg_data_cache_IR[CACHE_NUMS] = {0};  // 缓存区
uint16_t cache_counter = 0; // 缓存计数器
int touch_30102=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
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
  float max30102_data[2], fir_output[2];
  uint16_t HeartRate = 0;
  float SpO2 = 0;  
  printf("Max30102 Init\r\n");
/******************************************/  
  int ws2812_start=0; //时间间隔初始化
  int TIME_ago=0;     //时间间隔起始点
  uint32_t time_now=0;//当前时间
  int heart_cycle=750;//心率周期
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	   
	  //ws2812控制及蜂鸣器
	if(touch_30102)
	{
	   time_now= HAL_GetTick();
	  if(ws2812_start==0)
	  {
		  ws2812_start=1;  
		  TIME_ago=time_now;		
      }  
	  if(time_now-TIME_ago<=heart_cycle/8) //暂且设置为闪烁为1:7
	  {
	   rgb_show(17,RED);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	  }	  
	  else if(time_now-TIME_ago>heart_cycle)
		 ws2812_start=0; 
	  else 
	  {
	   rgb_show(17,BLACK);
	    HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	  }

	}
	else
	{
	  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	  rainbowCycle(1);
	}
	  //心率检测，完成OLED显示及心率周期计算
    if(MAX30102_Get_DATA(&HeartRate,&SpO2,max30102_data,fir_output) == MAX30102_DATA_OK)
	{
		printf("心率：%d  次/min   ", HeartRate);
		if(HeartRate>=60&&HeartRate<=110)
		{
			heart_cycle=60000/HeartRate; //计算心率周期（ms）
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

