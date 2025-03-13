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

enum MotorId {
	MOTOR_FL = 0,
	MOTOR_FR = 1,
	MOTOR_RL = 2,
	MOTOR_RR = 3
};

AMKMotorState_t * MotorState(enum MotorId mid) {
	switch (mid) {
	case MOTOR_FL: return &state.motor_state_fl;
	case MOTOR_FR: return &state.motor_state_fr;
	case MOTOR_RL: return &state.motor_state_rl;
	case MOTOR_RR: return &state.motor_state_rr;
	default: return NULL;
	}
}

int16_t * MotorTorqueSetpoint(enum MotorId mid) {
	switch (mid) {
	case MOTOR_FL: return &state.torqueSetpoints.front_left;
	case MOTOR_FR: return &state.torqueSetpoints.front_right;
	case MOTOR_RL: return &state.torqueSetpoints.rear_left;
	case MOTOR_RR: return &state.torqueSetpoints.rear_right;
	default: return NULL;
	}
}

void update_vehicle_state(uint32_t timeout) {
	osMutexAcquire(vdb_inverter_lockHandle, timeout);
	VehicleData.inverter = state;
	osMutexRelease(vdb_inverter_lockHandle);
}

void StartAMKTask(void *argument) {
	// Initialize state machines
	state.motor_state_fl = MOTOR_DISABLED;
	state.motor_state_fr = MOTOR_DISABLED;
	state.motor_state_rl = MOTOR_DISABLED;
	state.motor_state_rr = MOTOR_DISABLED;
	state.controller_state = MOTORS_DISABLED;
	update_vehicle_state(osWaitForever);

	while (1) {
		AMKControllerEventFlags_t flags;
		flags.flagInt = osEventFlagsWait(amkEventFlagsHandle, ~1, osFlagsWaitAny, osWaitForever);

		if (state.controller_state == MOTORS_DISABLED && flags.flagBits.start_motors) {
			// Begin startup sequence
			state.controller_state = STARTING_MOTORS;
			for (enum MotorId id=0; id<4; id++)
				*MotorState(id) = WAITING_FOR_SYSTEM_READY;
		}
		if (state.controller_state == STARTING_MOTORS)

		update_vehicle_state(osWaitForever);
	}
}
