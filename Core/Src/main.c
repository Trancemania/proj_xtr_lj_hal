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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Blink */
osThreadId_t BlinkHandle;
const osThreadAttr_t Blink_attributes = {
  .name = "Blink",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for KeepState */
osThreadId_t KeepStateHandle;
const osThreadAttr_t KeepState_attributes = {
  .name = "KeepState",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

uint8_t previous_command = 0;
uint8_t current_command = 0;
uint8_t UART_send_buffer[16];
uint8_t UART_recv_buffer[16];
uint8_t UART_field_buffer[16];
uint8_t ch;
int uart_recv_cnt = 0;
xSemaphoreHandle uart_A1_xSemaphore = NULL;
xSemaphoreHandle exti_xSemaphore = NULL;
portTickType time_origin;
volatile int pwm_count = 0;

osThreadId_t FieldHandle;
const osThreadAttr_t field_attributes = {
  .name = "FieldTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ETH_Init(void);
void StartDefaultTask(void *argument);
void StartBlink(void *argument);
void StartKeepState(void *argument);


/* USER CODE BEGIN PFP */
HAL_StatusTypeDef process_command (void);
void field_task(void *argument);

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  vSemaphoreCreateBinary(uart_A1_xSemaphore);
	vSemaphoreCreateBinary(exti_xSemaphore);
	
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

  /* creation of Blink */
  BlinkHandle = osThreadNew(StartBlink, NULL, &Blink_attributes);

  /* creation of KeepState */
  KeepStateHandle = osThreadNew(StartKeepState, NULL, &KeepState_attributes);

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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
	
	SystemCoreClockUpdate();
	
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (SystemCoreClock  / 2 / 10000 - 1);
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1333 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ETH_NRST_Pin|DO7_Pin|DO9_Pin|DO10_Pin
                          |DO5_Pin|DO0_Pin|DO12_Pin|DO11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DO1_Pin|DO2_Pin|DO3_Pin|DO6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ETH_NRST_Pin DO7_Pin DO9_Pin DO10_Pin
                           DO5_Pin DO0_Pin DO12_Pin DO11_Pin */
  GPIO_InitStruct.Pin = ETH_NRST_Pin|DO7_Pin|DO9_Pin|DO10_Pin
                          |DO5_Pin|DO0_Pin|DO12_Pin|DO11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DI13_Pin DI8_Pin DI12_Pin DI11_Pin */
  GPIO_InitStruct.Pin = DI13_Pin|DI8_Pin|DI12_Pin|DI11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DO1_Pin DO2_Pin DO3_Pin DO6_Pin */
  GPIO_InitStruct.Pin = DO1_Pin|DO2_Pin|DO3_Pin|DO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO4_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = DO4_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DI10_Pin DI9_Pin */
  GPIO_InitStruct.Pin = DI10_Pin|DI9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DI0_Pin DI1_Pin DI2_Pin DI3_Pin */
  GPIO_InitStruct.Pin = DI0_Pin|DI1_Pin|DI2_Pin|DI3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DI4_Pin */
  GPIO_InitStruct.Pin = DI4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DI4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI5_Pin DI6_Pin DI7_Pin */
  GPIO_InitStruct.Pin = DI5_Pin|DI6_Pin|DI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
  if(GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_3 || GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_5 || GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_8 || GPIO_Pin == GPIO_PIN_9)
	{
//		if(exti_lock == 0) {
//			exti_lock = 1;
//			process_command();
//		HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
//			exti_lock = 0;
	}
	else if (GPIO_Pin == GPIO_PIN_0) //DI10 PD
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
	}
	else if (GPIO_Pin == GPIO_PIN_1) //DI9	PD
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		}
	}
	else if (GPIO_Pin == GPIO_PIN_10) //DI8	PC
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	}
	else if (GPIO_Pin == GPIO_PIN_11) //DI12	PC
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		}
	}
	else if (GPIO_Pin == GPIO_PIN_12) //DI11	PC
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		}
	}
	else if (GPIO_Pin == GPIO_PIN_13) //DI13	PC
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
			xSemaphoreGiveFromISR( exti_xSemaphore, &xHigherPriorityTaskWoken );
		}
	}
}



HAL_StatusTypeDef process_command (void)
{
	unsigned int code_verify;
	unsigned int frame_verify;
	
	switch (current_command) {
		case 0x00:
//			HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
//			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&UART_send_buffer, 13) != HAL_OK)
//			{
//			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3| GPIO_PIN_5| GPIO_PIN_8| GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6| GPIO_PIN_7, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			if( FieldHandle != NULL ) {
				osThreadTerminate( FieldHandle );
			}
			return HAL_OK;
		
		case 0xff:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3| GPIO_PIN_5| GPIO_PIN_8| GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6| GPIO_PIN_7, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			if( FieldHandle != NULL ) {
				osThreadTerminate( FieldHandle );
			}
			return HAL_OK;
		
		case 0x01:
			if( FieldHandle != NULL ) {
				osThreadTerminate( FieldHandle );
			}
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	//				pwm_count = 0;
				
				//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1| GPIO_PIN_3| GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_5, GPIO_PIN_RESET);
		
			if (xSemaphoreTake( uart_A1_xSemaphore, 10000) == pdTRUE) {
					// hand shake
					// fill buffer as in tab3B1
					// function start: repeated code of hand shake sent tab3B1
					UART_send_buffer[0] = 0xA6;
					UART_send_buffer[1] = 0x0D;
					UART_send_buffer[2] = 0xAA;
					//https://zhidao.baidu.com/question/1638675882443131580.html?qbpn=2_2&tx=&word=%E4%BB%8E%E4%B8%80%E4%B8%AA16%E4%BD%8D%E5%8F%98%E9%87%8F%E4%B8%AD%E5%8F%96%E5%87%BA%E9%AB%988%E4%BD%8D%E5%92%8C%E4%BD%8E8%E4%BD%8D%EF%BC%8C%E6%B1%82C%E8%AF%AD%E8%A8%80%E7%A8%8B%E5%BA%8F%EF%BC%9F&fr=newsearchlist
					code_verify = (UART_recv_buffer[3] + UART_recv_buffer[4]) * UART_recv_buffer[5];
					UART_send_buffer[3] = code_verify & 0xff;
					UART_send_buffer[4] = (code_verify >> 8) & 0xff;
					UART_send_buffer[5] = (code_verify >> 16) & 0xff;
					UART_send_buffer[6] = (code_verify >> 24) & 0xff;
					UART_send_buffer[7] = 0x0B;
					UART_send_buffer[8] = 0x00;
					UART_send_buffer[9] = 0x00;
					UART_send_buffer[10] = 0x00;
					UART_send_buffer[11] = 0x00;
					frame_verify = UART_send_buffer[2] + UART_send_buffer[3] + UART_send_buffer[4] + UART_send_buffer[5] + \
											   UART_send_buffer[6] + UART_send_buffer[7] + UART_send_buffer[8] + UART_send_buffer[9] + \
												 UART_send_buffer[10];
					UART_send_buffer[12] = frame_verify & 0xff;
					UART_send_buffer[13] = 0x86;
					
					// send buffer
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&UART_send_buffer, 13);   //choose uart line
					if (xSemaphoreTake( exti_xSemaphore, 10000) == pdTRUE) {
						time_origin = xTaskGetTickCount() * portTICK_RATE_MS;
						FieldHandle = osThreadNew(field_task, NULL, &field_attributes);
//						xTaskCreate(field_task, "FIELD", configMINIMAL_STACK_SIZE * 4, NULL, FIELD_TASK_PRIO, &field_xHandle);
					}
					else {
						//error message
					}
			}
			return HAL_OK;
		
		case 0x02:
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_3| GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			return HAL_OK;
		
		case 0x03:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x04:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|  GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x05:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|  GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x06:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1500 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|  GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x07:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1500 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|  GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x08:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1500 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|  GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x09:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1500 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|  GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x0A:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1400 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;		

		case 0x0B:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1400 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;		

		case 0x0C:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1400 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;			

		case 0x0D:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1400 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;		

		case 0x0E:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1000 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;		
		
		case 0x0F:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 2000 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x10:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1000 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;			

		case 0x11:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 2000 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x12:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333- 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;		
		
		case 0x13:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x14:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;		
		
		case 0x15:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x16:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;			

		case 0x17:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x18:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;			

		case 0x19:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;	
		
		case 0x1A:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x1B:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;
	
		case 0x1D:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1400 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1| GPIO_PIN_3, GPIO_PIN_RESET);
		
//			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
		//varied frequency, same as 0x1F
			return HAL_OK;

		case 0x1F:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1400 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0| GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1, GPIO_PIN_RESET);
			return HAL_OK;

		case 0x20:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1|GPIO_PIN_3 , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5 , GPIO_PIN_RESET);
			return HAL_OK;	

		case 0x21:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5 , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 , GPIO_PIN_RESET);
			return HAL_OK;		
		
		case 0x22:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 |GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0 |GPIO_PIN_4|GPIO_PIN_5  , GPIO_PIN_RESET);
			return HAL_OK;

		case 0x23:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100 - 1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_3 |GPIO_PIN_4|GPIO_PIN_5 , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 , GPIO_PIN_RESET);
			return HAL_OK;

		case 0x24:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	//				pwm_count = 0;

		//config DO6/7/9/10/11/12
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1|GPIO_PIN_5 , GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_3 |GPIO_PIN_4 , GPIO_PIN_RESET);
			return HAL_OK;
		
		default:
//			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
			break;
	}
	return HAL_OK;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//	 portBASE_TYPE xHigherPriorityTaskWoken;
	 if(htim->Instance==TIM3){
      switch(pwm_count){
			case 0 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1510 - 1);
						pwm_count++;
						break;
			case 1 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1600 - 1);
						pwm_count++;
						break;
			case 2 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1570 - 1);
						pwm_count++;
						break;
			case 3 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1510 - 1);
						pwm_count++;
						break;
			case 4 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1470 - 1);
						pwm_count++;
						break;
			case 5 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1380 - 1);
						pwm_count++;
						break;
			case 6 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1300 - 1);
						pwm_count++;
						break;
			case 7 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1210 - 1);
						pwm_count++;
						break;
			case 8 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1160 - 1);
						pwm_count++;
						break;
			case 9 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1130 - 1);
						pwm_count++;
						break;
			case 10 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1170 - 1);
						pwm_count++;
						break;
			case 11 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1090 - 1);
						pwm_count++;
						break;
			case 12 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1090 - 1);
						pwm_count++;
						break;
			case 13 : 
						__HAL_TIM_SET_AUTORELOAD(&htim3, 1333 - 1);
						pwm_count=0;
						HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
//						xSemaphoreGiveFromISR( pwm_xSemaphore, &xHigherPriorityTaskWoken );
						break;
			default:
						pwm_count=0;
						HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
						break;
			}		
	 }
}

void field_task(void *argument){
	portTickType temp_t;
	int16_t epsilon;
	int16_t beta;
	unsigned int frame_verify;
	do {
		temp_t = xTaskGetTickCount() * portTICK_RATE_MS - time_origin;
		//large field
		if ( temp_t <= 3520 ) {
			UART_field_buffer[0] = 0xA6;
			UART_field_buffer[1] = 0x10;
			UART_field_buffer[2] = 0x81;
			UART_field_buffer[3] = temp_t & 0xff;
			UART_field_buffer[4] = (temp_t >> 8) & 0xff;
			UART_field_buffer[5] = (temp_t >> 16) & 0xff;
			UART_field_buffer[6] = (temp_t >> 24) & 0xff;
			switch (current_command) {
				case 0x1D:
					epsilon = 0;
					beta = -21537;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
					break;
								
				case 0x1F:
					epsilon = 0;
					beta = -21537;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
					break;
				case 0x01:
				  break;
				case 0x02:
				  break;
				case 0x03:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x04:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x05:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x06:
					epsilon = 10338;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x07:
					epsilon = 11199;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x08:
					epsilon = 14645;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x09:
					epsilon = 15506;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0A:
					epsilon = 10338;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0B:
					epsilon = 11199;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0C:
					epsilon = 12491;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0D:
					epsilon = 13353;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0E:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0F:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x10:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x11:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x12:
					epsilon = -1738;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x13:
					epsilon = 1738;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x14:
					epsilon = 0;
					beta = 1738;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x15:
					epsilon = 0;
					beta = -1738;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x16:
					epsilon = -1738;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x17:
					epsilon = 1738;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x18:
					epsilon = 0;
					beta = 1738;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x19:
					epsilon = 0;
					beta = -1738;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x1A:
					epsilon = 21537;
					beta = -108;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x1B:
					epsilon = 21537;
					beta = -108;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x20:
				  break;
				case 0x21:
				  break;
				case 0x22:
				  break;
				case 0x23:
				  break;
				case 0x24:
				  break;
				default:
					UART_field_buffer[7] = 0;
					UART_field_buffer[8] = 0;
					UART_field_buffer[9] = 0;
					UART_field_buffer[10] = 0;
					break;
			}
			UART_field_buffer[11] = 0x00;
			UART_field_buffer[12] = 2;
			UART_field_buffer[13] = 0x01;
			frame_verify = UART_field_buffer[2] + UART_field_buffer[3] + UART_field_buffer[4] + UART_field_buffer[5] + \
										 UART_field_buffer[6] + UART_field_buffer[7] + UART_field_buffer[8] + UART_field_buffer[9] + \
										 UART_field_buffer[10] + UART_field_buffer[11] + UART_field_buffer[12] + UART_field_buffer[13];
			UART_field_buffer[14] = frame_verify & 0xff;
			UART_field_buffer[15] = 0x86;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&UART_field_buffer, 16);   //choose uart line
		}
		//small field
		else {
			UART_field_buffer[0] = 0xA6;
			UART_field_buffer[1] = 0x10;
			UART_field_buffer[2] = 0x81;
			UART_field_buffer[3] = temp_t & 0xff;
			UART_field_buffer[4] = (temp_t >> 8) & 0xff;
			UART_field_buffer[5] = (temp_t >> 16) & 0xff;
			UART_field_buffer[6] = (temp_t >> 24) & 0xff;
			switch (current_command) {
				case 0x1D:
					epsilon = 0;
					beta = -16383;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
					break;
				case 0x1F:
					epsilon = 0;
					beta = -16383;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
					break;
				case 0x01:
				  break;
				case 0x02:
				  break;
				case 0x03:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x04:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x05:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x06:
					epsilon = 7864;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x07:
					epsilon = 8519;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x08:
					epsilon = 11140;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x09:
					epsilon = 11796;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0A:
					epsilon = 7864;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0B:
					epsilon = 8619;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0C:
					epsilon = 9502;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0D:
					epsilon = 10158;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x0E:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;	
				case 0x0F:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x10:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x11:
					epsilon = 0;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;	
				case 0x12:
					epsilon = -1322;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x013:
					epsilon = 1322;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x014:
					epsilon = 0;
					beta = 1322;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x15:
					epsilon = 0;
					beta = -1322;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x16:
					epsilon = -1322;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x17:
					epsilon = 1322;
					beta = 0;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;	
				case 0x18:
					epsilon = 0;
					beta = 1322;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x19:
					epsilon = 0;
					beta = -1322;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x1A:
					epsilon = 16383;
					beta = -81;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x1B:
					epsilon = 16383;
					beta = -81;
					UART_field_buffer[7] = epsilon & 0xff;
					UART_field_buffer[8] = (epsilon >> 8) & 0xff;
					UART_field_buffer[9] = beta & 0xff;
					UART_field_buffer[10] = (beta >> 8) & 0xff;
				  break;
				case 0x20:
				  break;
				case 0x21:
				  break;
				case 0x22:
				  break;
				case 0x23:
				  break;
				case 0x24:
				  break;					
				default:
					UART_field_buffer[7] = 0;
					UART_field_buffer[8] = 0;
					UART_field_buffer[9] = 0;
					UART_field_buffer[10] = 0;
					break;
			}
			UART_field_buffer[11] = 0x00;
			UART_field_buffer[12] = 2;
			UART_field_buffer[13] = 0x00;
			frame_verify = UART_field_buffer[2] + UART_field_buffer[3] + UART_field_buffer[4] + UART_field_buffer[5] + \
										 UART_field_buffer[6] + UART_field_buffer[7] + UART_field_buffer[8] + UART_field_buffer[9] + \
										 UART_field_buffer[10] + UART_field_buffer[11] + UART_field_buffer[12] + UART_field_buffer[13];
			UART_field_buffer[14] = frame_verify & 0xff;
			UART_field_buffer[15] = 0x86;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&UART_field_buffer, 16);   //choose uart line
		}
		vTaskDelay(20 * portTICK_RATE_MS);
	} while(xSemaphoreTake( uart_A1_xSemaphore, 0) != pdTRUE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	 portBASE_TYPE uart_xHigherPriorityTaskWoken;
	 if(UartHandle->Instance==USART1){
			HAL_UART_Receive_IT(&huart1, (uint8_t*) &ch, 1); 		//write ch
			//cnt = 0 of cnt != 0
		  if (uart_recv_cnt == 0)
			{
				if (ch == 0xA6) {
					UART_recv_buffer[uart_recv_cnt] = ch;
					uart_recv_cnt++;
				}
			}
			else if (uart_recv_cnt != 0)
			{
				UART_recv_buffer[uart_recv_cnt] = ch;
				uart_recv_cnt++;
				if (uart_recv_cnt == 13)
				{
					if ( UART_recv_buffer[12] == 0x86)
					{
						switch ( UART_recv_buffer[2]) {
							case 0x55:
								xSemaphoreGiveFromISR( uart_A1_xSemaphore, &uart_xHigherPriorityTaskWoken );
								uart_recv_cnt = 0;
								break;
							case 0x71:
								switch ( UART_recv_buffer[7]) {
									case 1:
	//									xSemaphoreGiveFromISR( uart_A2_xSemaphore, &xHigherPriorityTaskWoken );
										uart_recv_cnt = 0;
										break;
									case 2:
	//									xSemaphoreGiveFromISR( uart_A3_xSemaphore, &xHigherPriorityTaskWoken );
										uart_recv_cnt = 0;
										break;
									default:
										uart_recv_cnt = 0;
										break;
								}
								break;
							default:
								uart_recv_cnt = 0;
								break;
						}
					}
					else
					{
						uart_recv_cnt = 0;
					}
				}
				else if (uart_recv_cnt > 13)
				{
					uart_recv_cnt = 0;
				}
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
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlink */
/**
* @brief Function implementing the Blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink */
void StartBlink(void *argument)
{
  /* USER CODE BEGIN StartBlink */
  /* Infinite loop */
  for(;;)
  {
    osDelay(250);
		HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
  }
  /* USER CODE END StartBlink */
}

/* USER CODE BEGIN Header_StartKeepState */
/**
* @brief Function implementing the KeepState thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeepState */
void StartKeepState(void *argument)
{
  /* USER CODE BEGIN StartKeepState */
  /* Infinite loop */
  for(;;)
  {
		current_command = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)<<0) |  \
											(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)<<1) |  \
										  (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)<<2) |  \
										  (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)<<3) |  \
										  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)<<4) |  \
										  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)<<5) |  \
										  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)<<6) |  \
											(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)<<7);
		if (current_command != previous_command)
		{
			previous_command = current_command;
			if (process_command() != HAL_OK )
			{
				Error_Handler();
			}
		}
    osDelay(10);
  }
  /* USER CODE END StartKeepState */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
