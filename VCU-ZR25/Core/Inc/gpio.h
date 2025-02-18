/*
 * gpio.h
 *
 *  Created on: Feb 12, 2025
 *      Author: bre17
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f4xx_hal.h"

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

#endif /* INC_GPIO_H_ */
