/*
 * vehicle_data.c
 *
 *  Created on: Feb 12, 2025
 *      Author: bre17
 */

#include "vehicle_data.h"
#include "cmsis_os2.h"

#define VD_MUTEX_ATTRS(mutex_name) {							 \
		.name = (mutex_name), 								 \
		.attr_bits = osMutexPrioInherit | osMutexRobust, \
		.cb_mem = NULL, 								 \
		.cb_size = 0 									 \
	}

VehicleData_t VehicleData;

const osMutexAttr_t appsMutexAttrs = VD_MUTEX_ATTRS("appsVDMutex");
const osMutexAttr_t bpsFrontMutexAttrs = VD_MUTEX_ATTRS("bpsFrontVDMutex");
const osMutexAttr_t bpsRearMutexAttrs = VD_MUTEX_ATTRS("bpsRearVDMutex");
const osMutexAttr_t sasMutexAttrs = VD_MUTEX_ATTRS("sasVDMutex");
const osMutexAttr_t inverterMutexAttrs = VD_MUTEX_ATTRS("inverterVDMutex");
const osMutexAttr_t fsmStateMutexAttrs = VD_MUTEX_ATTRS("fsmStateVDMutex");
const osMutexAttr_t powSupMutexAttrs = VD_MUTEX_ATTRS("powSupVDMutex");

void initVehicleData() {
	VehicleData = (VehicleData_t){0};
	VehicleData.apps_lock = osMutexNew(&appsMutexAttrs);
	VehicleData.bps_front_lock = osMutexNew(&bpsFrontMutexAttrs);
	VehicleData.bps_rear_lock = osMutexNew(&bpsRearMutexAttrs);
	VehicleData.sas_lock = osMutexNew(&sasMutexAttrs);
	VehicleData.inverter_lock = osMutexNew(&inverterMutexAttrs);
	VehicleData.fsm_state_lock = osMutexNew(&fsmStateMutexAttrs);
	VehicleData.powsup_lock = osMutexNew(&powSupMutexAttrs);
}

VehicleData_t get_vehicle_data(){
	return VehicleData;
}
