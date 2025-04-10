/*
 * vehicle_fsm.h
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#ifndef INC_VEHICLE_FSM_H_
#define INC_VEHICLE_FSM_H_

#include "cmsis_os.h"
#define VEHICLE_FSM_TASK_PERIOD 50

#define FLAG_INDEX_GLVMS_TURNED_ON 0
#define FLAG_INDEX_SHUTDOWN_LOOP_OPEN 1
#define FLAG_INDEX_EXTERNAL_BUTTON_PRESSED 2
#define FLAG_INDEX_BRAKE_PRESSED 3
#define FLAG_INDEX_START_BUTTON_PRESSED 4
#define FLAG_INDEX_FAULT_DETECTED 5

typedef enum {
  VEHICLE_OFF,
  LOW_VOLTAGE_STATE,
  TRACTIVE_SYSTEM_ACTIVE_STATE,
  READY_TO_DRIVE_STATE
} VCU_State_t;

/* Interrupt flags */
typedef union {
	struct FSMInterruptFlagBits{
		uint8_t GLVMS_Turned_On : 1;
		uint8_t Shutdown_Loop_Open : 1;
		uint8_t External_Button_Pressed : 1;
		uint8_t Brake_Pressed : 1;
		uint8_t Start_Button_Pressed : 1;
		uint8_t Fault_Detected : 1;
	} flagBits;
	uint32_t flagInt;
} FSMInterruptFlags_t;

const static struct FSMInterruptFlagBits FSM_FLAGS_ALL = {1,1,1,1,1,1};
const static struct FSMInterruptFlagBits FSM_FLAGS_NONE = {0,0,0,0,0,0};

void StartFsmTask(void *argument);
void TransitionState(VCU_State_t newState);
void FSM_GPIO_Callback(uint16_t GPIO_Pin);
void fsm_flag_callback(uint8_t flag, uint8_t value);
void fsm_clear_flags();
const char* fsm_state_string(VCU_State_t state);

#endif /* INC_VEHICLE_FSM_H_ */
