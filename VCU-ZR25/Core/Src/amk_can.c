/*
 * amk_can.c
 *
 * Defines the tasks in charge of CAN bus communication
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */
#include "amk_can.h"
#include "vehicle_data.h"
#include "cmsis_os.h"

static AMKState_t state;

void update_vehicle_state(uint32_t timeout) {
	osMutexAcquire(vdb_inverter_lockHandle, timeout);
	VehicleData.inverter = state;
	osMutexRelease(vdb_inverter_lockHandle);
}

void StartAMKTask(void *argument) {
	// Initialize state machines
	state.motor_state_fl = WAITING_FOR_SYSTEM_READY;
	state.motor_state_fr = WAITING_FOR_SYSTEM_READY;
	state.motor_state_rl = WAITING_FOR_SYSTEM_READY;
	state.motor_state_rr = WAITING_FOR_SYSTEM_READY;
	state.controller_state = MOTORS_DISABLED;
	update_vehicle_state(osWaitForever);

	while (1) {

		osDelay(50);
	}
}
