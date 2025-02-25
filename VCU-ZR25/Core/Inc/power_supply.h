/*
 * power_supply.h
 *
 *  Created on: Feb 14, 2025
 *      Author: John
 */

#ifndef INC_POWER_SUPPLY_H_
#define INC_POWER_SUPPLY_H_

#include "stdbool.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define POWER_SUPPLY_TASK_PERIOD 50

#define ADC_5V_MIN_VOLTAGE    4950  // V * 1000
#define ADC_5V_MAX_VOLTAGE    5050  // V * 1000
#define ADC_3V_MIN_VOLTAGE    3250  // V * 1000
#define ADC_3V_MAX_VOLTAGE    3350  // V * 1000


typedef struct {
  ADC_HandleTypeDef hadc1;
  ADC_ChannelConfTypeDef sConfig;
} powSupTaskArgs_t;

static const osThreadAttr_t powsupTask_attributes = {
  .name = "powsupTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityNormal
};

typedef struct {
  bool value5V;
  bool value3V;
} PowSupData_t;

void StartPwrSupTask(powSupTaskArgs_t* args);
void fsm_power_callback(PowSupData_t powsup);


#endif /* INC_POWER_SUPPLY_H_ */
