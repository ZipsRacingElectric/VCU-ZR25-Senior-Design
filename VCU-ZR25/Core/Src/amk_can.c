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

void StartAMKTask(void *argument) {
	while (1) {
		osDelay(50);
	}
}
