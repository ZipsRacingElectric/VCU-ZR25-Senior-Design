/*
 * cooling_system.h
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

#ifndef INC_COOLING_SYSTEM_H_
#define INC_COOLING_SYSTEM_H_

#include "cmsis_os.h"

#define COOLING_TASK_PERIOD 50
#define MAX_TEMP_THRESHOLD 45 // Max and Min based off requirement 1.1.12
#define MIN_TEMP_THRESHOLD 30

static const osThreadAttr_t coolingTask_attributes = {
  .name = "coolingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityNormal
};

typedef struct {
  uint32_t fl_inverter_temp;
  uint32_t fr_inverter_temp;
  uint32_t rl_inverter_temp;
  uint32_t rr_inverter_temp;
  uint32_t fl_motor_temp;
  uint32_t fr_motor_temp;
  uint32_t rl_motor_temp;
  uint32_t rr_motor_temp;
} CoolingData_t;

void StartCoolingTask(void *argument);
void CoolingSystemTurnOnLeft();
void CoolingSystemTurnOnRight();
void CoolingSystemTurnOffLeft();
void CoolingSystemTurnOffRight();

#endif /* INC_COOLING_SYSTEM_H_ */
