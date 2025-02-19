/*
 * power_supply.h
 *
 *  Created on: Feb 14, 2025
 *      Author: John
 */

#ifndef INC_POWER_SUPPLY_H_
#define INC_POWER_SUPPLY_H_

#include "stdbool.h"
#include "main.h"
#include "cmsis_os.h"

#define POWER_SUPPLY_TASK_PERIOD 50

typedef struct {
  ADC_HandleTypeDef hadc1;
  ADC_ChannelConfTypeDef sConfig;
} powSupTaskArgs_t;

void StartPwrSupTask(powSupTaskArgs_t* args);

extern const osThreadAttr_t powsupTask_attributes;

typedef struct {
  bool value5V;
  bool value3V;
} PowSupData_t;

#endif /* INC_POWER_SUPPLY_H_ */
