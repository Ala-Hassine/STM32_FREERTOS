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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// LED command structure
typedef struct
{
    uint8_t ledType;      // 0: GREEN (PD13), 1: ORANGE (PD12)
    uint8_t action;       // 0: TOGGLE, 1: BLINK
    uint16_t blinkCount;
    uint32_t blinkDelay;
} LED_Command_t;

// Button press structure for interrupt handling
typedef struct
{
    uint8_t buttonId;     // 0: PA0, 1: PA1
    uint32_t pressTime;
} Button_Press_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS 	200
#define BUTTON_PA0 			0
#define BUTTON_PA1 			1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BTN1_Task */
osThreadId_t BTN1_TaskHandle;
const osThreadAttr_t BTN1_Task_attributes = {
  .name = "BTN1_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BTN2_Task */
osThreadId_t BTN2_TaskHandle;
const osThreadAttr_t BTN2_Task_attributes = {
  .name = "BTN2_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* USER CODE BEGIN PV */
osMessageQueueId_t ledQueueHandle;
osMessageQueueId_t buttonQueueHandle;
uint32_t lastPressTimePA0 = 0;
uint32_t lastPressTimePA1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void StartBTN1_Task(void *argument);
void StartBTN2_Task(void *argument);
void StartLEDTask(void *argument);

/* USER CODE BEGIN PFP */
void LED_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t IsButtonPressed(uint8_t buttonId, uint32_t* lastPressTime);
void ProcessButtonPress(uint8_t buttonId);
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
  /* USER CODE BEGIN 2 */
  // Create message queues
  ledQueueHandle 	= osMessageQueueNew(10, sizeof(LED_Command_t), NULL);
  buttonQueueHandle = osMessageQueueNew(10, sizeof(Button_Press_t), NULL);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // Orange LED (PD12)
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); // Green LED (PD13)
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

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

  /* creation of BTN1_Task */
  BTN1_TaskHandle = osThreadNew(StartBTN1_Task, NULL, &BTN1_Task_attributes);

  /* creation of BTN2_Task */
  BTN2_TaskHandle = osThreadNew(StartBTN2_Task, NULL, &BTN2_Task_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

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
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN0_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN0_Pin|BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LED_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}
uint8_t IsButtonPressed(uint8_t buttonId, uint32_t* lastPressTime)
{
    uint32_t currentTime = osKernelGetTickCount();
    if ((currentTime - *lastPressTime) > DEBOUNCE_DELAY_MS)
    {
        *lastPressTime = currentTime;
        return 1;
    }
    return 0;
}
void ProcessButtonPress(uint8_t buttonId)
{
    LED_Command_t ledCmd;
    ledCmd.ledType 	= 0;
    ledCmd.action 	= 0;
    osMessageQueuePut(ledQueueHandle, &ledCmd, 0, 0);
    ledCmd.ledType 	= 1;
    ledCmd.action 	= 1;
    if (buttonId == BUTTON_PA0)
    {
        ledCmd.blinkCount = 30;
        ledCmd.blinkDelay = 200;
    }
    else
    {
        ledCmd.blinkCount = 20;
        ledCmd.blinkDelay = 1000;
    }
    osMessageQueuePut(ledQueueHandle, &ledCmd, 0, 0);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Button_Press_t buttonPress;
    if (GPIO_Pin == BTN0_Pin)
    {
        if (IsButtonPressed(BUTTON_PA0, &lastPressTimePA0))
        {
            buttonPress.buttonId 	= BUTTON_PA0;
            buttonPress.pressTime 	= osKernelGetTickCount();
            osMessageQueuePut(buttonQueueHandle, &buttonPress, 0, 0);
        }
    }
    else if (GPIO_Pin == BTN1_Pin)
    {
        if (IsButtonPressed(BUTTON_PA1, &lastPressTimePA1))
        {
            buttonPress.buttonId 	= BUTTON_PA1;
            buttonPress.pressTime 	= osKernelGetTickCount();
            osMessageQueuePut(buttonQueueHandle, &buttonPress, 0, 0);
        }
    }
}
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
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBTN1_Task */
/**
* @brief Function implementing the BTN1_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBTN1_Task */
void StartBTN1_Task(void *argument)
{
  /* USER CODE BEGIN StartBTN1_Task */
  Button_Press_t buttonPress;
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(buttonQueueHandle, &buttonPress, NULL, osWaitForever) == osOK)
    {
        ProcessButtonPress(buttonPress.buttonId);
    }
  }
  /* USER CODE END StartBTN1_Task */
}

/* USER CODE BEGIN Header_StartBTN2_Task */
/**
* @brief Function implementing the BTN2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBTN2_Task */
void StartBTN2_Task(void *argument)
{
  /* USER CODE BEGIN StartBTN2_Task */
  /* This task can be used for other functionality */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartBTN2_Task */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  LED_Command_t ledCmd;
  /* Infinite loop */
  for(;;)
  {
	if (osMessageQueueGet(ledQueueHandle, &ledCmd, NULL, osWaitForever) == osOK)
	{
		if (osMutexAcquire(myMutex01Handle, osWaitForever) == osOK)
		{
			switch (ledCmd.action)
			{
				case 0: // TOGGLE
					if (ledCmd.ledType == 0)
					{
						LED_Toggle(LED2_GPIO_Port, LED2_Pin);
					}
					else
					{
						LED_Toggle(LED1_GPIO_Port, LED1_Pin);
					}
					break;
				case 1: // BLINK
					if (ledCmd.ledType == 1)
					{
						for (int i = 0; i < ledCmd.blinkCount * 2; i++)
						{
							LED_Toggle(LED1_GPIO_Port, LED1_Pin);
							osDelay(ledCmd.blinkDelay);
						}
					}
					break;
				default: break;
			}
			osMutexRelease(myMutex01Handle);
		}
	}
  }
  /* USER CODE END StartLEDTask */
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
