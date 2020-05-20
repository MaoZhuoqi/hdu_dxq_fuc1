/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GUI.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define K1        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)  //K1按键PA4
#define K2        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)  //K2按键PA5
#define K3        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)  //K3按键PA8
#define K4        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)  //K2按键PA15
#define K1_PRES 1
#define K2_PRES 2
#define K3_PRES 0x04
#define K4_PRES 0x08

#define WS_MAIN 0
#define WS_TEST 1
#define WS_MPU 2
#define WS_PWM 3
#define WS_WIFI 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

osThreadId defaultTaskHandle;
osThreadId myTaskGUIHandle;
osThreadId myTaskKeyScanHandle;
/* USER CODE BEGIN PV */
extern GUI_FLASH const GUI_FONT GUI_FontHZ_SimSun_16;
extern GUI_CONST_STORAGE GUI_BITMAP bmyc;

uint8_t g_ws =WS_MAIN;
uint8_t g_ws_next=WS_TEST;
uint16_t val[20];
uint16_t adval=0;
unsigned char key_up=1;     //按键松开标志
uint8_t g_key_cur=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskGUI(void const * argument);
void StartTaskKeyScan(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setLED(unsigned char val)
{
	HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,(val&0x01)?GPIO_PIN_RESET:GPIO_PIN_SET);
	HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,(val&0x02)?GPIO_PIN_RESET:GPIO_PIN_SET);
	HAL_GPIO_WritePin(D3_GPIO_Port,D3_Pin,(val&0x04)?GPIO_PIN_RESET:GPIO_PIN_SET);
	HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,(val&0x08)?GPIO_PIN_RESET:GPIO_PIN_SET);
}
unsigned char KEY_Scan(unsigned char mode)
{
    
    if(mode==1)key_up=1;    //支持连按
    if(key_up&&(K1==0||K2==0||K3==0||K4==0))
    {
        HAL_Delay(10);
        key_up=0;
        if(K1==0)       return K1_PRES;
        else if(K2==0)  return K2_PRES;
        else if(K3==0) return K3_PRES;
				else if(K4==0) return K4_PRES;			
    }else if(K1==1&&K2==1&&K3==1&&K4==1)key_up=1;
    return 0;   //无按键按下
}
void ShowMainGUI(void)
{
  GUI_Clear();
	GUI_DispStringHCenterAt("主菜单",64,0);
	if(g_ws_next>=WS_PWM)
	{
		if((HAL_GetTick()%1000)>500)
		{
			if(WS_MPU==g_ws_next)
        GUI_DispStringAt("*",0,16);
      if(g_ws_next==WS_PWM)
        GUI_DispStringAt("*",0,32);
      if(g_ws_next==WS_WIFI)
        GUI_DispStringAt("*",0,48);
		}
		
		GUI_DispStringAt("姿态解算",16,16);
		GUI_DispStringAt("呼吸灯",16,32);
		GUI_DispStringAt("WIFI通信",16,48);
	}
	else {
    	if((HAL_GetTick()%1000)>500)
		{
			if(WS_TEST==g_ws_next)
        GUI_DispStringAt("*",0,16);
      if(g_ws_next==WS_MPU)
        GUI_DispStringAt("*",0,32);
      if(g_ws_next==WS_PWM)
        GUI_DispStringAt("*",0,48);
		}
		GUI_DispStringAt("系统测试",16,16);
		GUI_DispStringAt("姿态解算",16,32);
		GUI_DispStringAt("呼吸灯",16,48);
	}
  GUI_Update();
}	
void ShowTestGUI(void)
{
  GUI_Clear();
  GUI_DispStringHCenterAt("系统测试",64,0);
  GUI_SetFont(&GUI_Font8_ASCII);

  char buf[30];
  sprintf(buf,"AD:%d",adval);
  GUI_DispStringAt(buf,10,16);

  GUI_FillRect(50,16,50+(adval%1000)*70/1000,17);
  if(adval>=4000)
  GUI_FillRect(50,18,120,19);
   if(adval>=3000)
  GUI_FillRect(50,20,120,21);
   if(adval>=2000)
  GUI_FillRect(50,22,120,23);
   if(adval>=1000)
  GUI_FillRect(50,24,120,25);

  if(K1==0)
  {
    GUI_DispStringAt("■■",10,30);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("K1",10,30);
    GUI_SetColor(GUI_COLOR_WHITE);
  }
  else {
    GUI_DispStringAt("K1",10,30);
  }

  if(K2==0)
  {
    GUI_DispStringAt("■■",30,30);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("K2",30,30);
    GUI_SetColor(GUI_COLOR_WHITE);
  }
  else {
    GUI_DispStringAt("K2",30,30);
  }
  if(K3==0)
  {
    GUI_DispStringAt("■■",50,30);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("K3",50,30);
    GUI_SetColor(GUI_COLOR_WHITE);
  }
  else {
    GUI_DispStringAt("K3",50,30);
  }
  if(K4==0)
  {
    GUI_DispStringAt("■■",70,30);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("K4",70,30);
    GUI_SetColor(GUI_COLOR_WHITE);
  }
  else {
    GUI_DispStringAt("K4",70,30);
  }
  GUI_Update();
  GUI_SetFont(&GUI_FontHZ_SimSun_16);
}
void ShowMPUGUI(void)
{
  GUI_Clear();
  GUI_DispStringHCenterAt("姿态解算",64,0);
  GUI_Update();
}
void ShowPWMGUI(void)
{
  GUI_Clear();
  GUI_DispStringHCenterAt("呼吸灯",64,0);
  GUI_Update();
}
void ShowWiFiGUI(void)
{
  GUI_Clear();
  GUI_DispStringHCenterAt("WIFI通信",64,0);
  GUI_Update();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
{ 
  int i; 
  uint16_t sum = 0; // 20 次采集数据累加和 
  for (i = 0; i < 20; ++i) 
  sum += val[i]; 
  adval = sum / 20;// 均值滤波 
  }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	unsigned char key;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	setLED(0);
	HAL_Delay(250);
	setLED(0x0f);
	HAL_Delay(250);

	setLED(0x00);
	HAL_Delay(250);
	setLED(0x0f);
	HAL_Delay(250);
	setLED(0x00);

	GUI_Init();
	GUI_SetFont(&GUI_FontHZ_SimSun_16);
	GUI_DispStringAt("专业设计实践II",0,0);
	GUI_DispStringAt("杨宇晨17041129",0,20);
	GUI_Update();
	GUI_Delay(1000);
	GUI_Clear();
	GUI_DrawBitmap(&bmyc,(128-bmyc.XSize)/2,(64-bmyc.YSize)/2);
	GUI_Update();
	
	
	while(K1==1&&K2==1&&K3==1&&K4==1);
	HAL_Delay(250);
	GUI_Clear();
	GUI_Update();
  HAL_ADCEx_Calibration_Start(&hadc1); 
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)val, 20); // 启动 DMA 方式连续采集，每采 20 个数据触发中断
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTaskGUI */
  osThreadDef(myTaskGUI, StartTaskGUI, osPriorityIdle, 0, 128);
  myTaskGUIHandle = osThreadCreate(osThread(myTaskGUI), NULL);

  /* definition and creation of myTaskKeyScan */
  osThreadDef(myTaskKeyScan, StartTaskKeyScan, osPriorityIdle, 0, 128);
  myTaskKeyScanHandle = osThreadCreate(osThread(myTaskKeyScan), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  while (1)
  {
		 
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
 
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_Pin|D2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D3_Pin|D4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : K1_Pin K2_Pin K3_Pin K4_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K2_Pin|K3_Pin|K4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    

  /* USER CODE BEGIN 5 */
  uint8_t led_sta=0x01;
  uint32_t led_tick=HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
 
		
      setLED(led_sta);
  if(HAL_GetTick()>=led_tick+300)
  {
    led_tick=HAL_GetTick();
     led_sta<<=1;
  if(led_sta>0x08)
  led_sta=0x01;
  }
        
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTaskGUI */
/**
* @brief Function implementing the myTaskGUI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGUI */
void StartTaskGUI(void const * argument)
{
  /* USER CODE BEGIN StartTaskGUI */
  /* Infinite loop */
  for(;;)
  {
    switch(g_ws)
		{
			default:
			case WS_MAIN:
      ShowMainGUI();
				break;
			case WS_TEST:
        ShowTestGUI();
				break;
			case WS_MPU:
        ShowMPUGUI();
				break;
			case WS_PWM:
        ShowPWMGUI();
				break;
			case WS_WIFI:
      ShowWiFiGUI();
				break;
		}
    osDelay(1);
  }
  /* USER CODE END StartTaskGUI */
}

/* USER CODE BEGIN Header_StartTaskKeyScan */
/**
* @brief Function implementing the myTaskKeyScan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskKeyScan */
void StartTaskKeyScan(void const * argument)
{
  /* USER CODE BEGIN StartTaskKeyScan */
  /* Infinite loop */
  for(;;)
  {
     g_key_cur=KEY_Scan(0);            //按键扫描
		if(g_key_cur==K4_PRES)
		{
			g_ws=WS_MAIN;
		}
    	switch(g_ws)
		{
			default:
			case WS_MAIN:

					switch(g_key_cur)
						{				 
						case K1_PRES:
							if(g_ws_next>WS_TEST)
								--g_ws_next;
							break;
						case K2_PRES:
							if(g_ws_next<WS_WIFI)
								++g_ws_next; 
							break;
						case K3_PRES:				
							g_ws=g_ws_next;			
							break;
					;
						default:
							HAL_Delay(10);
								break;
						}
						
				break;
			case WS_TEST:

				break;
			case WS_MPU:

				break;
			case WS_PWM:

				break;
			case WS_WIFI:

				break;
		}
    osDelay(1);
  }
  /* USER CODE END StartTaskKeyScan */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
