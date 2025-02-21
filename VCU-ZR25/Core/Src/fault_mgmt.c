/*
 * fault_mgmt.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

#include "fault_mgmt.h"
#include "vehicle_fsm.h"
#include "driver_sensors.h"

#define IMPLAUSIBILITY_TIMEOUT 100

typedef union {
	struct FaultTypeBits{
		uint8_t Fault_implausibility : 1;
	} faultBits;
	uint32_t faultInt;
} FaultType_t;

const struct FaultTypeBits FAULTS_ALL = {1};
const struct FaultTypeBits FAULTS_NONE = {0};

static uint32_t MASK_ALL = ~0;
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
		fsm_fault_callback();

		const FaultType_t mask = {.faultBits = FAULTS_ALL};
		fault.faultInt = osThreadFlagsWait(mask.faultInt, osFlagsWaitAny, 10);
	}
}

void fsm_fault_callback(){
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	fault.faultInt = osThreadFlagsGet();
	uint8_t flag = FLAG_INDEX_FAULT_DETECTED;
	uint8_t value = 0;
	if((fault.faultInt & MASK_ALL) != 0){
		value = 1;
	}
	fsm_flag_callback(flag, value);
}

void fault_check(){
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	VehicleData_t vehicle_data = get_vehicle_data();

	if (!vehicle_data.apps.plausible | !vehicle_data.bps_front.plausible
			| !vehicle_data.bps_rear.plausible | vehicle_data.sas.plausible) {

		if (!implausibility_detected) {
			implausibility_timer = osKernelSysTick();
			implausibility_detected = 1;
		}

		if (osKernelSysTick() - implausibility_timer >= IMPLAUSIBILITY_TIMEOUT) {
			fault.faultBits.Fault_implausibility = 1;
		}
	}
	else {
		implausibility_detected = 0;
		osThreadFlagsClear(1 << FAULT_INDEX_IMPLAUSIBILITY);
	}

	osThreadFlagsSet(thread_id, fault.faultInt);
}
