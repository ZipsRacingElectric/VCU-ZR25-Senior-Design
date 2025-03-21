/*
 * fault_mgmt.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

/*
 * TODO:
 * - CAN comm for inverters, bms, gps, vim
 * - apps_bps_implausibility_check: make motor torques 0 for apps/bps implausibilities
 * - vim_can_check: add status field in vehicle data, call torque handler to decide action to take
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

	if(nonCriticalFaults){
		DashboardFaultCallback(1);
	}
	else {
		DashboardFaultCallback(0);
	}
}

void fault_check(){
	FaultType_t fault = {.faultBits = FAULTS_NONE};
	VehicleData_t vehicle_data = get_vehicle_data();

	apps_bps_implausibility_check(fault, vehicle_data);
	sas_implausibility_check(fault, vehicle_data);
	gps_check(fault);
	gnss_check(fault);
	inverter_check(fault);
	inverter_can_check(fault);
	bms_can_check(fault);
	gps_can_check(fault);
	vim_can_check(fault);
	glv_check(fault);
	vcu_check(fault);

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
			torque_fault_callback(1);
			// CAN motor callback to have 0 nm
		}
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_APPS_BPS_FAILURE);
		implausibility_detected = 0;
		torque_fault_callback(0);
		osThreadFlagsClear(flagInt);
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
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_SS_FAILURE);
		implausibility_detected = 0;
		osThreadFlagsClear(flagInt);
	}
}

void gps_check(FaultType_t fault){
	if (1) { // fault detected
		ControlMode_t ctrl_mode = 0;
		fault.faultBits.Fault_gps = 1;
		update_control_mode(ctrl_mode);
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_GPS_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void gnss_check(FaultType_t fault){
	if (1) { // fault detected
		ControlMode_t ctrl_mode = 0;
		fault.faultBits.Fault_gnss = 1;
		update_control_mode(ctrl_mode);
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_GNSS_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void inverter_check(FaultType_t fault){
	if (1) { // fault detected
		fault.faultBits.Fault_inverter = 1;
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_INV_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void inverter_can_check(FaultType_t fault){
	if (1) { // fault detected
		fault.faultBits.Fault_inverter_com = 1;
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_INV_COM_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void bms_can_check(FaultType_t fault){
	if (1) { // fault detected
		ControlMode_t ctrl_mode = 0;
		fault.faultBits.Fault_bms = 1;
		update_control_mode(ctrl_mode);
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_BMS_COM_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void gps_can_check(FaultType_t fault){
	if (1) { // fault detected
		ControlMode_t ctrl_mode = 0;
		fault.faultBits.Fault_gps_com = 1;
		update_control_mode(ctrl_mode);
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_GPS_COM_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void vim_can_check(FaultType_t fault){
	if (1) { // fault detected
		fault.faultBits.Fault_vim_com = 1;
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_VIM_COM_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void glv_check(FaultType_t fault){
	if (1) { // fault detected
		fault.faultBits.Fault_glv = 1;
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_GLV_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}

void vcu_check(FaultType_t fault){
	if (1) { // fault detected
		fault.faultBits.Fault_vcu = 1;
	}
	else {
		uint32_t flagInt = 0;
		flagInt |= (1 << FAULT_INDEX_VCU_FAILURE);
		osThreadFlagsClear(flagInt);
	}
}
