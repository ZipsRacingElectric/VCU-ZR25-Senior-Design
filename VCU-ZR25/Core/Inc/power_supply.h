/*
 * power_supply.h
 *
 *  Created on: Feb 14, 2025
 *      Author: John
 */

#ifndef INC_POWER_SUPPLY_H_
#define INC_POWER_SUPPLY_H_

#include "stdbool.h"

void StartPwrSupTask(
		ADC_HandleTypeDef hadc1,
		ADC_ChannelConfTypeDef sConfig
);

const osThreadAttr_t powsupTask_attributes = {
  .name = "powsupTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

typedef struct {
  bool value5V;
  bool value3V;
} PowSupData_t;

#endif /* INC_POWER_SUPPLY_H_ */
