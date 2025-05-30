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

typedef enum {
    CONTROL_MODE_LIMP = 0,
} ControlMode_t;

typedef struct {
  ControlMode_t controlmode;
  uint8_t torque_percent;
  uint32_t torque_value;
} TorqueCtrlData_t;

void StartTorqueCtrlTask(void *argument);
void update_control_mode(ControlMode_t);
void update_torque_output();
void increment_torque_limit();
void decrement_torque_limit();
void torque_fault_callback(uint8_t value);

#endif /* INC_TORQUE_CTRL_H_ */
