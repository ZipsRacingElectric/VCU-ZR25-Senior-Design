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
#define DEBUG_LED_1_Pin GPIO_PIN_0
#define DEBUG_LED_1_GPIO_Port GPIOC
#define DEBUG_LED_2_Pin GPIO_PIN_1
#define DEBUG_LED_2_GPIO_Port GPIOC
#define DEBUG_LED_3_Pin GPIO_PIN_2
#define DEBUG_LED_3_GPIO_Port GPIOC
#define CAN_1_STANDBY_Pin GPIO_PIN_3
#define CAN_1_STANDBY_GPIO_Port GPIOC
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
#define VCU_SHUTDOWN_LOOP_IN_Pin GPIO_PIN_5
#define VCU_SHUTDOWN_LOOP_IN_GPIO_Port GPIOA
#define VCU_SHUTDOWN_LOOP_IN_EXTI_IRQn EXTI9_5_IRQn
#define VCU_SHUTDOWN_LOOP_RESET_Pin GPIO_PIN_6
#define VCU_SHUTDOWN_LOOP_RESET_GPIO_Port GPIOA
#define VCU_SHUTDOWN_LOOP_RESET_EXTI_IRQn EXTI9_5_IRQn
#define START_BUTTON_Pin GPIO_PIN_7
#define START_BUTTON_GPIO_Port GPIOA
#define START_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define CAN_2_STANDBY_Pin GPIO_PIN_4
#define CAN_2_STANDBY_GPIO_Port GPIOC
#define RAIL_POWER_ENABLE_5V_Pin GPIO_PIN_5
#define RAIL_POWER_ENABLE_5V_GPIO_Port GPIOC
#define FAN_1_CONTROL_Pin GPIO_PIN_0
#define FAN_1_CONTROL_GPIO_Port GPIOB
#define FAN_2_CONTROL_Pin GPIO_PIN_1
#define FAN_2_CONTROL_GPIO_Port GPIOB
#define BOOT_1_Pin GPIO_PIN_2
#define BOOT_1_GPIO_Port GPIOB
#define VCU_FAULT_Pin GPIO_PIN_10
#define VCU_FAULT_GPIO_Port GPIOB
#define PUMP_1_CONTROL_Pin GPIO_PIN_6
#define PUMP_1_CONTROL_GPIO_Port GPIOC
#define PUMP_2_CONTROL_Pin GPIO_PIN_7
#define PUMP_2_CONTROL_GPIO_Port GPIOC
#define DASH_INPUT_1_Pin GPIO_PIN_8
#define DASH_INPUT_1_GPIO_Port GPIOA
#define DASH_INPUT_2_Pin GPIO_PIN_9
#define DASH_INPUT_2_GPIO_Port GPIOA
#define DASH_INPUT_2_EXTI_IRQn EXTI9_5_IRQn
#define DASH_INPUT_3_Pin GPIO_PIN_10
#define DASH_INPUT_3_GPIO_Port GPIOA
#define DASH_INPUT_3_EXTI_IRQn EXTI15_10_IRQn
#define DASH_INPUT_4_Pin GPIO_PIN_15
#define DASH_INPUT_4_GPIO_Port GPIOA
#define DASH_INPUT_4_EXTI_IRQn EXTI15_10_IRQn
#define BRAKE_LIGHT_CONTROL_Pin GPIO_PIN_4
#define BRAKE_LIGHT_CONTROL_GPIO_Port GPIOB
#define BUZZER_CONTROL_Pin GPIO_PIN_5
#define BUZZER_CONTROL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
