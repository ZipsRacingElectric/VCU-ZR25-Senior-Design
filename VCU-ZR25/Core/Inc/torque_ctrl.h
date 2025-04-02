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

typedef struct {} torqueControlTorqueDistParams_t;
typedef struct {} torqueControlPIDParams_t;

typedef struct {
	torqueControlTorqueDistParams_t td_params;
	volatile torqueControlPIDParams_t pid_params;
} torqueControlParameters_t;

extern const torqueControlParameters_t torque_control_params;

// Programs new PID parameters to the flash memory.
// These new values will persist across reboots, but will be overwritten when
// the VCU is reprogrammed. Permanent changes should be committed in
// torque_ctrl.c.
void program_pid_params(torqueControlPIDParams_t *);

typedef enum {
    CONTROL_MODE_LIMP = 0,
} ControlMode_t;

typedef struct {
  ControlMode_t controlmode;
  uint8_t torque_percent;
} TorqueCtrlData_t;

void StartTorqueCtrlTask(void *argument);
void update_control_mode(ControlMode_t);
void increment_torque_limit();
void decrement_torque_limit();
void torque_fault_callback(uint8_t value);

#endif /* INC_TORQUE_CTRL_H_ */
