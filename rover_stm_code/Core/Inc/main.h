/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2C.h"
#include "uart.h"
#include "acs.h"
#include "stepper.h"
#include "quad_gpio.h"
#include "enc_poll.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR1_Pin GPIO_PIN_13
#define DIR1_GPIO_Port GPIOC
#define DIR4_Pin GPIO_PIN_2
#define DIR4_GPIO_Port GPIOC
#define DIR9_Pin GPIO_PIN_3
#define DIR9_GPIO_Port GPIOC
#define DIR5_Pin GPIO_PIN_3
#define DIR5_GPIO_Port GPIOA
#define PULSE2_Pin GPIO_PIN_4
#define PULSE2_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_0
#define DIR2_GPIO_Port GPIOB
#define DIR8_Pin GPIO_PIN_1
#define DIR8_GPIO_Port GPIOB
#define EM_BRAKE_S_Pin GPIO_PIN_2
#define EM_BRAKE_S_GPIO_Port GPIOB
#define DIR7_Pin GPIO_PIN_10
#define DIR7_GPIO_Port GPIOB
#define DIR10_Pin GPIO_PIN_11
#define DIR10_GPIO_Port GPIOB
#define DIR11_Pin GPIO_PIN_12
#define DIR11_GPIO_Port GPIOB
#define DIR_STEP_2_Pin GPIO_PIN_13
#define DIR_STEP_2_GPIO_Port GPIOB
#define EM_BRAKE_E_Pin GPIO_PIN_14
#define EM_BRAKE_E_GPIO_Port GPIOB
#define PULSE1_Pin GPIO_PIN_15
#define PULSE1_GPIO_Port GPIOB
#define DRILL_QUAD_A_Pin GPIO_PIN_8
#define DRILL_QUAD_A_GPIO_Port GPIOC
#define DRILL_QUAD_A_EXTI_IRQn EXTI9_5_IRQn
#define DRILL_QUAD_B_Pin GPIO_PIN_9
#define DRILL_QUAD_B_GPIO_Port GPIOC
#define DRILL_QUAD_B_EXTI_IRQn EXTI9_5_IRQn
#define DIR3_Pin GPIO_PIN_13
#define DIR3_GPIO_Port GPIOA
#define DIR_STEP_1_Pin GPIO_PIN_12
#define DIR_STEP_1_GPIO_Port GPIOC
#define DIR6_Pin GPIO_PIN_9
#define DIR6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
