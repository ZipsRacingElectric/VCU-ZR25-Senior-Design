/*
 * fault_mgmt.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

/*
 * TODO:
 * - fault_check: CAN comm for motor inverter
 * - Add all faults
 * - fault_callback: make motor torques 0 for apps/bps implausibilities
 */

#include "fault_mgmt.h"
#include "vehicle_fsm.h"
#include "driver_sensors.h"
#include "torque_ctrl.h"

#define IMPLAUSIBILITY_TIMEOUT 100

const uint8_t fault_critical[NUM_FAULTS] = {
    1, 1, 1, // critical
    0, 0, 0, 0, 0, 0, 0, 0 // non-critical
};

static uint32_t implausibility_timer = 0;
static bool implausibility_detected = 0;

static osThreadId_t thread_id;

void StartFaultTask(void *argument){
	thread_id = osThreadGetId();
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	osThreadFlagsSet(thread_id, fault.faultInt);
	for(;;){
		fault.faultInt = osThreadFlagsGet();

		fault_check();
		fault_callback();

		const FaultType_t mask = {.faultBits = FAULTS_ALL};
		fault.faultInt = osThreadFlagsWait(mask.faultInt, osFlagsWaitAny, 10);
	}
}

void fault_callback(){
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	fault.faultInt = osThreadFlagsGet();

	uint32_t criticalFaults = 0;
	uint32_t nonCriticalFaults = 0;

	for (uint8_t i = 0; i < NUM_FAULTS; i++) {
		if (fault.faultInt & (1 << i)) {
			if (fault_critical[i]) {
				criticalFaults |= (1 << i);
			} else {
				nonCriticalFaults |= (1 << i);
			}
		}
	}

	if(criticalFaults){
		fsm_flag_callback(FLAG_INDEX_FAULT_DETECTED, 1);
	}

	if(nonCriticalFaults){
		DashboardFaultCallback();
	}
}

void fault_check(){
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	VehicleData_t vehicle_data = get_vehicle_data();

	apps_bps_implausibility_check(fault, vehicle_data);
	sas_implausibility_check(fault, vehicle_data);

	osThreadFlagsSet(thread_id, fault.faultInt);
}

void apps_bps_implausibility_check(FaultType_t fault, VehicleData_t vehicle_data){
	if (!vehicle_data.apps.plausible | !vehicle_data.bps_front.plausible
			| !vehicle_data.bps_rear.plausible) {

		if (!implausibility_detected) {
			implausibility_timer = osKernelSysTick();
			implausibility_detected = 1;
		}
		if (osKernelSysTick() - implausibility_timer >= IMPLAUSIBILITY_TIMEOUT) {
			fault.faultBits.Fault_apps_bps = 1;
		}
	}
	else {
		implausibility_detected = 0;
		osThreadFlagsClear(1 << FAULT_INDEX_APPS_BPS_FAILURE);
	}
}

void sas_implausibility_check(FaultType_t fault, VehicleData_t vehicle_data){
	if (!vehicle_data.sas.plausible) {

		if (!implausibility_detected) {
			implausibility_timer = osKernelSysTick();
			implausibility_detected = 1;
		}
		if (osKernelSysTick() - implausibility_timer >= IMPLAUSIBILITY_TIMEOUT) {
			ControlMode_t ctrl_mode = 0;
			fault.faultBits.Fault_ss = 1;
			update_control_mode(ctrl_mode);
		}
	}
	else {
		implausibility_detected = 0;
		osThreadFlagsClear(1 << FAULT_INDEX_SS_FAILURE);
	}
}
