/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t 		delay;
	uint32_t 		pin;
	GPIO_TypeDef* 	port;
}Ala_Struct_LedConfig;
Ala_Struct_LedConfig led1 = {200, GPIO_PIN_12, GPIOD};
Ala_Struct_LedConfig led2 = {400, GPIO_PIN_13, GPIOD};
Ala_Struct_LedConfig led3 = {600, GPIO_PIN_14, GPIOD};
Ala_Struct_LedConfig led4 = {800, GPIO_PIN_15, GPIOD};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char buffer[100];
uint32_t lastSuspendTime 	= 0;
uint8_t suspendedLedCount 	= 0;
uint8_t mycounter 			= 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED1_Task */
osThreadId_t LED1_TaskHandle;
const osThreadAttr_t LED1_Task_attributes = {
  .name = "LED1_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED2_Task */
osThreadId_t LED2_TaskHandle;
const osThreadAttr_t LED2_Task_attributes = {
  .name = "LED2_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED3_Task */
osThreadId_t LED3_TaskHandle;
const osThreadAttr_t LED3_Task_attributes = {
  .name = "LED3_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED4_Task */
osThreadId_t LED4_TaskHandle;
const osThreadAttr_t LED4_Task_attributes = {
  .name = "LED4_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedGenericTask */
osThreadId_t LedGeneric1TaskHandle;
osThreadId_t LedGeneric2TaskHandle;
osThreadId_t LedGeneric3TaskHandle;
osThreadId_t LedGeneric4TaskHandle;
const osThreadAttr_t LedGenericTask_attributes = {
  .name = "LedGenericTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartLED1_Task(void *argument);
void StartLED2_Task(void *argument);
void StartLED3_Task(void *argument);
void StartLED4_Task(void *argument);
void StartLedGenericTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  /* creation of LED1_Task */
  LED1_TaskHandle = osThreadNew(StartLED1_Task, NULL, &LED1_Task_attributes);

  /* creation of LED2_Task */
  LED2_TaskHandle = osThreadNew(StartLED2_Task, NULL, &LED2_Task_attributes);

  /* creation of LED3_Task */
  LED3_TaskHandle = osThreadNew(StartLED3_Task, NULL, &LED3_Task_attributes);

  /* creation of LED4_Task */
  LED4_TaskHandle = osThreadNew(StartLED4_Task, NULL, &LED4_Task_attributes);

  /* creation of LedGenericTask */
  LedGeneric1TaskHandle = osThreadNew(StartLedGenericTask, &led1, &LedGenericTask_attributes);
  LedGeneric2TaskHandle = osThreadNew(StartLedGenericTask, &led2, &LedGenericTask_attributes);
  LedGeneric3TaskHandle = osThreadNew(StartLedGenericTask, &led3, &LedGenericTask_attributes);
  LedGeneric4TaskHandle = osThreadNew(StartLedGenericTask, &led4, &LedGenericTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
      osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLED1_Task */
/**
* @brief Function implementing the LED1_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED1_Task */
void StartLED1_Task(void *argument)
{
  /* USER CODE BEGIN StartLED1_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED1_Task */
}

/* USER CODE BEGIN Header_StartLED2_Task */
/**
* @brief Function implementing the LED2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED2_Task */
void StartLED2_Task(void *argument)
{
  /* USER CODE BEGIN StartLED2_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED2_Task */
}

/* USER CODE BEGIN Header_StartLED3_Task */
/**
* @brief Function implementing the LED3_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED3_Task */
void StartLED3_Task(void *argument)
{
  /* USER CODE BEGIN StartLED3_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED3_Task */
}

/* USER CODE BEGIN Header_StartLED4_Task */
/**
* @brief Function implementing the LED4_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED4_Task */
void StartLED4_Task(void *argument)
{
  /* USER CODE BEGIN StartLED4_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLED4_Task */
}

/* USER CODE BEGIN Header_StartLedGenericTask */
/**
* @brief Function implementing the Led_ToggleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedGenericTask */
void StartLedGenericTask(void *argument)
{
  /* USER CODE BEGIN StartLedGenericTask */
	ala:
	Ala_Struct_LedConfig ala;
	if (argument != NULL)
	{
		ala = *(Ala_Struct_LedConfig*) argument;
	}
	else
	{
		// Default Configuration If NULL Is Passed
		ala.delay   = 1000;
		ala.pin     = GPIO_PIN_0;
		ala.port    = GPIOA;
	}
	osThreadId_t currentTaskHandle 	= osThreadGetId();
	uint8_t suspendThreshold 		= 0;
	uint8_t taskId 					= 0;
	if (currentTaskHandle == LedGeneric1TaskHandle)
	{
		taskId = 1;
		suspendThreshold = 10; // LED1 suspends after 10 cycles
	}
	else if (currentTaskHandle == LedGeneric2TaskHandle)
	{
		taskId = 2;
		suspendThreshold = 20; // LED2 suspends after 20 cycles
	}
	else if (currentTaskHandle == LedGeneric3TaskHandle)
	{
		taskId = 3;
		suspendThreshold = 30; // LED3 suspends after 30 cycles
	}
	else if (currentTaskHandle == LedGeneric4TaskHandle)
	{
		taskId = 4;
		suspendThreshold = 40; // LED4 suspends after 40 cycles
	}
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(ala.port, ala.pin);
		osDelay(ala.delay);
		mycounter++;
		if (mycounter >= suspendThreshold)
		{
			HAL_GPIO_WritePin(ala.port, ala.pin, GPIO_PIN_RESET);
			taskENTER_CRITICAL();
			snprintf(buffer, sizeof(buffer), "LED %d Is OFF\r\n", taskId);
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			taskEXIT_CRITICAL();
			suspendedLedCount++;
			if (suspendedLedCount >= 4)
			{
				taskENTER_CRITICAL();
				snprintf(buffer, sizeof(buffer), "All LEDs Suspended ... Auto-Resume After 10 Seconds\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				taskEXIT_CRITICAL();
				osDelay(10000);
				suspendedLedCount = 0;
				if (LedGeneric1TaskHandle != NULL) osThreadResume(LedGeneric1TaskHandle);
				if (LedGeneric2TaskHandle != NULL) osThreadResume(LedGeneric2TaskHandle);
				if (LedGeneric3TaskHandle != NULL) osThreadResume(LedGeneric3TaskHandle);
				if (LedGeneric4TaskHandle != NULL) osThreadResume(LedGeneric4TaskHandle);
				goto ala;
			}
			osThreadSuspend(currentTaskHandle);
			mycounter = 0;
			if (suspendedLedCount > 0)
			{
				suspendedLedCount--;
			}
		}
	}
  /* USER CODE END StartLedGenericTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
