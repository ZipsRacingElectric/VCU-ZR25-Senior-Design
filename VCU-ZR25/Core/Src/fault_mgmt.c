/*
 * fault_mgmt.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

/*
 * TODO:
 * - CAN comm for bms, gps, vim
 * - fault_flag_callback: vim can - add status field in vehicle data, call torque handler to decide action to take
 */

#include "fault_mgmt.h"
#include "vehicle_fsm.h"
#include "driver_sensors.h"
#include "torque_ctrl.h"
#include "vehicle_data.h"

#define IMPLAUSIBILITY_TIMEOUT osKernelGetSysTimerFreq()/10

const uint8_t fault_critical[NUM_FAULTS] = {
	1, 0, 1, // critical
	0, 0, 0, 0, 0, 0, 0 // non-critical
    //1, // critical
    //0, 0, 0, 0, 0, 0, 0, 0, 0 // non-critical
};

static uint32_t apps_bps_implausibility_timer = 0;
static uint32_t sas_implausibility_timer = 0;
static bool apps_bps_implausibility_detected = 0;
static bool sas_implausibility_detected = 0;
static osThreadId_t thread_id;

FaultType_t faultsToClear = {.faultBits = FAULTS_NONE};

void update_fault_management_data(FaultType_t fault){
	osMutexAcquire(vdb_faulttask_lockHandle, osWaitForever);
	VehicleData.faultmgmt = fault;
	osMutexRelease(vdb_faulttask_lockHandle);
}

void StartFaultTask(void *argument){
	thread_id = osThreadGetId();
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	osThreadFlagsSet(thread_id, fault.faultInt);
	for(;;){
		fault.faultInt = osThreadFlagsGet();

		fault_check();
		fault_callback();
		fault_clear_flags();
		update_fault_management_data(fault);

		osDelay(FAULT_MGMT_TASK_PERIOD);
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
	else {
		fsm_flag_callback(FLAG_INDEX_FAULT_DETECTED, 0);
	}

	if(nonCriticalFaults || criticalFaults){
		DashboardFaultCallback(1);
	}
	else {
		DashboardFaultCallback(0);
	}
}

void fault_check(){
	FaultType_t fault = {.faultBits = FAULTS_NONE};

	apps_bps_implausibility_check(&fault);
	sas_implausibility_check(&fault);
	gps_check(&fault);
	inverter_check(&fault);
	glv_check(&fault);
	vcu_check(&fault);

	osThreadFlagsSet(thread_id, fault.faultInt);
}

void apps_bps_implausibility_check(FaultType_t *fault){
	if (!VehicleData.apps.plausible | !VehicleData.bps_front.plausible
			| !VehicleData.bps_rear.plausible) {

		if (!apps_bps_implausibility_detected) {
			apps_bps_implausibility_timer = osKernelSysTick();
			apps_bps_implausibility_detected = 1;
		}
		if (osKernelSysTick() - apps_bps_implausibility_timer >= IMPLAUSIBILITY_TIMEOUT) {
			fault->faultBits.Fault_apps_bps = 1;
			torque_fault_callback(1);
			// CAN motor callback to have 0 nm
		}
	}
	else {
		apps_bps_implausibility_detected = 0;
		torque_fault_callback(0);
		faultsToClear.faultInt |= (1 << FAULT_INDEX_APPS_BPS_FAILURE);
	}
}

void sas_implausibility_check(FaultType_t *fault){
	if (!VehicleData.sas.plausible) {

		if (!sas_implausibility_detected) {
			sas_implausibility_timer = osKernelSysTick();
			sas_implausibility_detected = 1;
		}
		if (osKernelSysTick() - sas_implausibility_timer >= IMPLAUSIBILITY_TIMEOUT) {
			ControlMode_t ctrl_mode = 0;
			fault->faultBits.Fault_ss = 1;
			update_control_mode(ctrl_mode);
		}
	}
	else {
		sas_implausibility_detected = 0;
		faultsToClear.faultInt |= (1 << FAULT_INDEX_SS_FAILURE);

	}
}

void gps_check(FaultType_t *fault){
	if (0) { // fault detected
		ControlMode_t ctrl_mode = 0;
		fault->faultBits.Fault_gps = 1;
		update_control_mode(ctrl_mode);
	}
	else {
		faultsToClear.faultInt |= (1 << FAULT_INDEX_GPS_FAILURE);
	}
}

void inverter_check(FaultType_t *fault){
	if (0) { // fault detected
		fault->faultBits.Fault_inverter = 1;
	}
	else {
		faultsToClear.faultInt |= (1 << FAULT_INDEX_INV_FAILURE);
	}
}

void glv_check(FaultType_t *fault){
	if (0) { // fault detected
		fault->faultBits.Fault_glv = 1;
	}
	else {
		faultsToClear.faultInt |= (1 << FAULT_INDEX_GLV_FAILURE);
	}
}

void vcu_check(FaultType_t *fault){
	if (0) { // fault detected
		fault->faultBits.Fault_vcu = 1;
	}
	else {
		faultsToClear.faultInt |= (1 << FAULT_INDEX_VCU_FAILURE);
	}
}

void fault_clear_flags(){
	osThreadFlagsClear(faultsToClear.faultInt);
	faultsToClear.faultBits = FAULTS_NONE;
}

void fault_flag_callback(uint8_t fault, uint8_t value){
	FaultType_t faults = {.faultBits = FAULTS_NONE};
	if (value){
		if(fault == (FAULT_INDEX_BMS_COM_FAILURE | FAULT_INDEX_GPS_COM_FAILURE)){
			ControlMode_t ctrl_mode = 0;
			update_control_mode(ctrl_mode);
		}
		else if (fault == FAULT_INDEX_VIM_COM_FAILURE){

		}
		else if (fault == FAULT_INDEX_INV_COM_FAILURE){

		}
    	faults.faultInt |= (1 << fault);
    }
    else{
		faultsToClear.faultInt |= (1 << fault);
    }

    osThreadFlagsSet(thread_id, faults.faultInt);
}
