/*
 * vehicle_fsm.h
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#ifndef INC_VEHICLE_FSM_H_
#define INC_VEHICLE_FSM_H_

void StartFSMTask(void *argument);
void TransitionState(VCU_State_t newState);
void FSM_GPIO_Callback(uint16_t GPIO_Pin);

const osThreadAttr_t fsmTask_attributes = {
  .name = "fsmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

typedef enum {
  VEHICLE_OFF,
  LOW_VOLTAGE_STATE,
  TRACTIVE_SYSTEM_ACTIVE_STATE,
  READY_TO_DRIVE_STATE,
  LOCKOUT_STATE
} VCU_State_t;

#endif /* INC_VEHICLE_FSM_H_ */
