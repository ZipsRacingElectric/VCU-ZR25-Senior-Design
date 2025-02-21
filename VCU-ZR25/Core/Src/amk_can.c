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

static int16_t torqueLimitPos = -1; // TODO: Torque limits
static int16_t torqueLimitNeg = -1; // TODO: Torque limits

static amkTorqueSetpoints torqueSetpoints;

typedef enum {
	WAITING_FOR_SYSTEM_READY,
	WAITING_FOR_QUIT_DC_ON,
	WAITING_FOR_QUIT_INVERTER_ON,
	MOTOR_READY,
} AMKMotorState_t;

typedef enum {
	MOTORS_DISABLED,
	STARTING_MOTORS,
	MOTORS_READY,
	STOPPING_MOTORS
} AMKState_t;

static AMKMotorState_t motor_state_fl;
static AMKMotorState_t motor_state_fr;
static AMKMotorState_t motor_state_rl;
static AMKMotorState_t motor_state_rr;
static AMKState_t controller_state;

void StartAMKTask(void *argument) {
	// Initialize state machines
	motor_state_fl = WAITING_FOR_SYSTEM_READY;
	motor_state_fr = WAITING_FOR_SYSTEM_READY;
	motor_state_rl = WAITING_FOR_SYSTEM_READY;
	motor_state_rr = WAITING_FOR_SYSTEM_READY;
	controller_state = MOTORS_DISABLED;



	while (1) {

		osDelay(50);
	}
}
