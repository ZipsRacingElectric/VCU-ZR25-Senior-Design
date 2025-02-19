/*
 * gpio.h
 *
 *  Created on: Feb 12, 2025
 *      Author: bre17
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "main.h"

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
#define VCU_SHUTDOWN_LOOP_RESET_Pin GPIO_PIN_6
#define VCU_SHUTDOWN_LOOP_RESET_GPIO_Port GPIOA
#define START_BUTTON_Pin GPIO_PIN_7
#define START_BUTTON_GPIO_Port GPIOA
#define DASH_INPUT_1_Pin GPIO_PIN_8
#define DASH_INPUT_1_GPIO_Port GPIOA
#define DASH_INPUT_2_Pin GPIO_PIN_9
#define DASH_INPUT_2_GPIO_Port GPIOA
#define DASH_INPUT_3_Pin GPIO_PIN_10
#define DASH_INPUT_3_GPIO_Port GPIOA
#define DASH_INPUT_4_Pin GPIO_PIN_15
#define DASH_INPUT_4_GPIO_Port GPIOA

#define FAN_1_CONTROL_Pin GPIO_PIN_0
#define FAN_1_CONTROL_GPIO_Port GPIOB
#define FAN_2_CONTROL_Pin GPIO_PIN_1
#define FAN_2_CONTROL_GPIO_Port GPIOB
#define BOOT_1_Pin GPIO_PIN_2
#define BOOT_1_GPIO_Port GPIOB
#define BRAKE_LIGHT_CONTROL_Pin GPIO_PIN_4
#define BRAKE_LIGHT_CONTROL_GPIO_Port GPIOB
#define BUZZER_CONTROL_Pin GPIO_PIN_5
#define BUZZER_CONTROL_GPIO_Port GPIOB
#define VCU_FAULT_Pin GPIO_PIN_10
#define VCU_FAULT_GPIO_Port GPIOB

#define DEBUG_LED_1_Pin GPIO_PIN_0
#define DEBUG_LED_1_GPIO_Port GPIOC
#define DEBUG_LED_2_Pin GPIO_PIN_1
#define DEBUG_LED_2_GPIO_Port GPIOC
#define DEBUG_LED_3_Pin GPIO_PIN_2
#define DEBUG_LED_3_GPIO_Port GPIOC
#define CAN_1_STANDBY_Pin GPIO_PIN_3
#define CAN_1_STANDBY_GPIO_Port GPIOC
#define CAN_2_STANDBY_Pin GPIO_PIN_4
#define CAN_2_STANDBY_GPIO_Port GPIOC
#define RAIL_POWER_ENABLE_5V_Pin GPIO_PIN_5
#define RAIL_POWER_ENABLE_5V_GPIO_Port GPIOC
#define PUMP_1_CONTROL_Pin GPIO_PIN_6
#define PUMP_1_CONTROL_GPIO_Port GPIOC
#define PUMP_2_CONTROL_Pin GPIO_PIN_7
#define PUMP_2_CONTROL_GPIO_Port GPIOC



#endif /* INC_GPIO_H_ */
