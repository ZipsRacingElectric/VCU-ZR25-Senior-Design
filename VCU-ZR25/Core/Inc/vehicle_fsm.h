/*
 * vehicle_fsm.h
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#ifndef INC_VEHICLE_FSM_H_
#define INC_VEHICLE_FSM_H_

#include "cmsis_os.h"
#include "gpio.h"

#define FLAG_INDEX_GLVMS_TURNED_ON 0
#define FLAG_INDEX_SHUTDOWN_LOOP_OPEN 1
#define FLAG_INDEX_EXTERNAL_BUTTON_PRESSED 2
#define FLAG_INDEX_BRAKE_PRESSED 3
#define FLAG_INDEX_START_BUTTON_PRESSED 4
#define FLAG_INDEX_FAULT_DETECTED 5
#define FLAG_INDEX_EXTERNAL_RESET_PRESSED 6

static const osThreadAttr_t fsmTask_attributes = {
  .name = "fsmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal
};

typedef enum {
  VEHICLE_OFF,
  LOW_VOLTAGE_STATE,
  TRACTIVE_SYSTEM_ACTIVE_STATE,
  READY_TO_DRIVE_STATE
} VCU_State_t;

void StartFSMTask(void *argument);
void TransitionState(VCU_State_t newState);
void FSM_GPIO_Callback(uint16_t GPIO_Pin);
void fsm_flag_callback(uint8_t flag, uint8_t value);

#endif /* INC_VEHICLE_FSM_H_ */
