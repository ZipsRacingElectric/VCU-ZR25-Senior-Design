/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define APPS_1_Pin GPIO_PIN_0
#define APPS_1_GPIO_Port GPIOA
#define APPS_2_Pin GPIO_PIN_1
#define APPS_2_GPIO_Port GPIOA
#define BPS_FRONT_Pin GPIO_PIN_2
#define BPS_FRONT_GPIO_Port GPIOA
#define BPS_REAR_Pin GPIO_PIN_3
#define BPS_REAR_GPIO_Port GPIOA
#define GLV_BATTERY_Pin GPIO_PIN_4
#define GLV_BATTERY_GPIO_Port GPIOA
#define HEARTBEAT_LED_Pin GPIO_PIN_5
#define HEARTBEAT_LED_GPIO_Port GPIOA
#define STATUS_LED_1_Pin GPIO_PIN_6
#define STATUS_LED_1_GPIO_Port GPIOA
#define STATUS_LED_2_Pin GPIO_PIN_7
#define STATUS_LED_2_GPIO_Port GPIOA
#define VCU_SHUTDOWN_LOOP_Pin GPIO_PIN_4
#define VCU_SHUTDOWN_LOOP_GPIO_Port GPIOC
#define WATER_PUMP_1_Pin GPIO_PIN_5
#define WATER_PUMP_1_GPIO_Port GPIOC
#define WATER_PUMP_2_Pin GPIO_PIN_0
#define WATER_PUMP_2_GPIO_Port GPIOB
#define FAN_1_Pin GPIO_PIN_1
#define FAN_1_GPIO_Port GPIOB
#define FAN_2_Pin GPIO_PIN_2
#define FAN_2_GPIO_Port GPIOB
#define CAN2_STANDBY_Pin GPIO_PIN_4
#define CAN2_STANDBY_GPIO_Port GPIOB
#define CAN1_STANDBY_Pin GPIO_PIN_5
#define CAN1_STANDBY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
