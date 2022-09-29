/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define LedRun_Pin GPIO_PIN_0
#define LedRun_GPIO_Port GPIOA
#define LedFault_Pin GPIO_PIN_1
#define LedFault_GPIO_Port GPIOA
#define LedRamps_Pin GPIO_PIN_2
#define LedRamps_GPIO_Port GPIOA
#define X_StepPin_Pin GPIO_PIN_6
#define X_StepPin_GPIO_Port GPIOA
#define X_DirPin_Pin GPIO_PIN_7
#define X_DirPin_GPIO_Port GPIOA
#define X_EnablePin_Pin GPIO_PIN_0
#define X_EnablePin_GPIO_Port GPIOB
#define X_MinPin_Pin GPIO_PIN_1
#define X_MinPin_GPIO_Port GPIOB
#define Y_StepPin_Pin GPIO_PIN_2
#define Y_StepPin_GPIO_Port GPIOB
#define Y_DirPin_Pin GPIO_PIN_10
#define Y_DirPin_GPIO_Port GPIOB
#define Y_EnablePin_Pin GPIO_PIN_12
#define Y_EnablePin_GPIO_Port GPIOB
#define Y_MinPin_Pin GPIO_PIN_13
#define Y_MinPin_GPIO_Port GPIOB
#define Z_StepPin_Pin GPIO_PIN_14
#define Z_StepPin_GPIO_Port GPIOB
#define Z_DirPin_Pin GPIO_PIN_15
#define Z_DirPin_GPIO_Port GPIOB
#define Z_EnablePin_Pin GPIO_PIN_8
#define Z_EnablePin_GPIO_Port GPIOA
#define Z_MinPin_Pin GPIO_PIN_11
#define Z_MinPin_GPIO_Port GPIOA
#define E0_StepPin_Pin GPIO_PIN_12
#define E0_StepPin_GPIO_Port GPIOA
#define E0_MinPin_Pin GPIO_PIN_15
#define E0_MinPin_GPIO_Port GPIOA
#define E1_StepPin_Pin GPIO_PIN_3
#define E1_StepPin_GPIO_Port GPIOB
#define E1_DirPin_Pin GPIO_PIN_4
#define E1_DirPin_GPIO_Port GPIOB
#define E1_EnablePin_Pin GPIO_PIN_5
#define E1_EnablePin_GPIO_Port GPIOB
#define E0_DirPin_Pin GPIO_PIN_8
#define E0_DirPin_GPIO_Port GPIOB
#define E0_EnablePin_Pin GPIO_PIN_9
#define E0_EnablePin_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
