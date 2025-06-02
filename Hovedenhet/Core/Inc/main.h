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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "util.h"

#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DELAY_NS 100
#define GOPRO_MAX 100

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define csLOW() HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define csHIGH() HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAM2_PWR_Pin GPIO_PIN_3
#define CAM2_PWR_GPIO_Port GPIOE
#define CAM1_PWR_Pin GPIO_PIN_1
#define CAM1_PWR_GPIO_Port GPIOC
#define CS_PIN_Pin GPIO_PIN_4
#define CS_PIN_GPIO_Port GPIOA
#define CAM3_PWR_Pin GPIO_PIN_11
#define CAM3_PWR_GPIO_Port GPIOB
#define GOPRO_Pin GPIO_PIN_10
#define GOPRO_GPIO_Port GPIOD
#define YELLOW_LED_Pin GPIO_PIN_3
#define YELLOW_LED_GPIO_Port GPIOG
#define GREEN_LED_Pin GPIO_PIN_4
#define GREEN_LED_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */
#define ENABLE_SERIAL_PRINTF // Enables printf to console via huart

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
