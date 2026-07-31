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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CON_U_N_Pin GPIO_PIN_7
#define CON_U_N_GPIO_Port GPIOA
#define CON_V_N_Pin GPIO_PIN_0
#define CON_V_N_GPIO_Port GPIOB
#define CON_W_N_Pin GPIO_PIN_1
#define CON_W_N_GPIO_Port GPIOB
#define K4_Pin GPIO_PIN_7
#define K4_GPIO_Port GPIOE
#define INV_U_N_Pin GPIO_PIN_8
#define INV_U_N_GPIO_Port GPIOE
#define INV_U_Pin GPIO_PIN_9
#define INV_U_GPIO_Port GPIOE
#define INV_V_N_Pin GPIO_PIN_10
#define INV_V_N_GPIO_Port GPIOE
#define INV_V_Pin GPIO_PIN_11
#define INV_V_GPIO_Port GPIOE
#define INV_W_N_Pin GPIO_PIN_12
#define INV_W_N_GPIO_Port GPIOE
#define INV_W_Pin GPIO_PIN_13
#define INV_W_GPIO_Port GPIOE
#define K11_Pin GPIO_PIN_14
#define K11_GPIO_Port GPIOE
#define K12_Pin GPIO_PIN_15
#define K12_GPIO_Port GPIOE
#define K9_Pin GPIO_PIN_12
#define K9_GPIO_Port GPIOB
#define K10_Pin GPIO_PIN_13
#define K10_GPIO_Port GPIOB
#define K13_Pin GPIO_PIN_8
#define K13_GPIO_Port GPIOD
#define K14_Pin GPIO_PIN_9
#define K14_GPIO_Port GPIOD
#define K15_Pin GPIO_PIN_10
#define K15_GPIO_Port GPIOD
#define K8_Pin GPIO_PIN_13
#define K8_GPIO_Port GPIOD
#define K0_Pin GPIO_PIN_14
#define K0_GPIO_Port GPIOD
#define K1_Pin GPIO_PIN_15
#define K1_GPIO_Port GPIOD
#define CON_U_Pin GPIO_PIN_6
#define CON_U_GPIO_Port GPIOC
#define CON_V_Pin GPIO_PIN_7
#define CON_V_GPIO_Port GPIOC
#define CON_W_Pin GPIO_PIN_8
#define CON_W_GPIO_Port GPIOC
#define K2_Pin GPIO_PIN_0
#define K2_GPIO_Port GPIOD
#define K3_Pin GPIO_PIN_1
#define K3_GPIO_Port GPIOD
#define K6_Pin GPIO_PIN_4
#define K6_GPIO_Port GPIOD
#define K5_Pin GPIO_PIN_5
#define K5_GPIO_Port GPIOD
#define K7_Pin GPIO_PIN_7
#define K7_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
