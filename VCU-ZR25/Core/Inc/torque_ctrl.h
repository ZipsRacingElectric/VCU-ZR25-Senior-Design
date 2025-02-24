/*
 * torque_ctrl.h
 *
 *  Created on: Feb 23, 2025
 *      Author: John
 */

#ifndef INC_TORQUE_CTRL_H_
#define INC_TORQUE_CTRL_H_

#include "stdbool.h"
#include "cmsis_os.h"

#define TORQUE_CONTROL_TASK_PERIOD 50

#define MAX_TORQUE_VARIABLE    4950


static const osThreadAttr_t torquectrlTask_attributes = {
  .name = "torquectrlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityNormal
};

typedef struct {
  uint8_t controlmode;
  uint8_t torque_percent;
} TorqueCtrlData_t;

void StartTorqueCtrlTask(void *argument);
void increment_torque_limit();
void decrement_torque_limit();

#endif /* INC_TORQUE_CTRL_H_ */
