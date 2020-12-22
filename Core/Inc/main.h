/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETH_NRST_Pin GPIO_PIN_2
#define ETH_NRST_GPIO_Port GPIOE
#define DO7_Pin GPIO_PIN_3
#define DO7_GPIO_Port GPIOE
#define DO9_Pin GPIO_PIN_4
#define DO9_GPIO_Port GPIOE
#define DO10_Pin GPIO_PIN_5
#define DO10_GPIO_Port GPIOE
#define DO5_Pin GPIO_PIN_6
#define DO5_GPIO_Port GPIOE
#define DI13_Pin GPIO_PIN_13
#define DI13_GPIO_Port GPIOC
#define DI13_EXTI_IRQn EXTI15_10_IRQn
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define DO1_Pin GPIO_PIN_3
#define DO1_GPIO_Port GPIOA
#define DO2_Pin GPIO_PIN_5
#define DO2_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define DO0_Pin GPIO_PIN_7
#define DO0_GPIO_Port GPIOE
#define DO4_Pin GPIO_PIN_11
#define DO4_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define DO3_Pin GPIO_PIN_8
#define DO3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DO6_Pin GPIO_PIN_15
#define DO6_GPIO_Port GPIOA
#define DI8_Pin GPIO_PIN_10
#define DI8_GPIO_Port GPIOC
#define DI8_EXTI_IRQn EXTI15_10_IRQn
#define DI12_Pin GPIO_PIN_11
#define DI12_GPIO_Port GPIOC
#define DI12_EXTI_IRQn EXTI15_10_IRQn
#define DI11_Pin GPIO_PIN_12
#define DI11_GPIO_Port GPIOC
#define DI11_EXTI_IRQn EXTI15_10_IRQn
#define DI10_Pin GPIO_PIN_0
#define DI10_GPIO_Port GPIOD
#define DI10_EXTI_IRQn EXTI0_IRQn
#define DI9_Pin GPIO_PIN_1
#define DI9_GPIO_Port GPIOD
#define DI9_EXTI_IRQn EXTI1_IRQn
#define DI0_Pin GPIO_PIN_2
#define DI0_GPIO_Port GPIOD
#define DI0_EXTI_IRQn EXTI2_IRQn
#define DI1_Pin GPIO_PIN_3
#define DI1_GPIO_Port GPIOD
#define DI1_EXTI_IRQn EXTI3_IRQn
#define DI2_Pin GPIO_PIN_4
#define DI2_GPIO_Port GPIOD
#define DI2_EXTI_IRQn EXTI4_IRQn
#define DI3_Pin GPIO_PIN_7
#define DI3_GPIO_Port GPIOD
#define DI3_EXTI_IRQn EXTI9_5_IRQn
#define DI4_Pin GPIO_PIN_3
#define DI4_GPIO_Port GPIOB
#define DO8_Pin GPIO_PIN_4
#define DO8_GPIO_Port GPIOB
#define DI5_Pin GPIO_PIN_5
#define DI5_GPIO_Port GPIOB
#define DI5_EXTI_IRQn EXTI9_5_IRQn
#define DI6_Pin GPIO_PIN_8
#define DI6_GPIO_Port GPIOB
#define DI6_EXTI_IRQn EXTI9_5_IRQn
#define DI7_Pin GPIO_PIN_9
#define DI7_GPIO_Port GPIOB
#define DI7_EXTI_IRQn EXTI9_5_IRQn
#define DO12_Pin GPIO_PIN_0
#define DO12_GPIO_Port GPIOE
#define DO11_Pin GPIO_PIN_1
#define DO11_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
