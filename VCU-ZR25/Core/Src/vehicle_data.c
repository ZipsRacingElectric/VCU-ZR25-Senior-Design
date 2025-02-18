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

const osMutexAttr_t appsMutexAttrs = VD_MUTEX_ATTRS("appsVDMutex");
const osMutexAttr_t bpsMutexAttrs = VD_MUTEX_ATTRS("bpsVDMutex");
const osMutexAttr_t sasMutexAttrs = VD_MUTEX_ATTRS("sasVDMutex");
const osMutexAttr_t inverterMutexAttrs = VD_MUTEX_ATTRS("inverterVDMutex");
const osMutexAttr_t fsmStateMutexAttrs = VD_MUTEX_ATTRS("fsmStateVDMutex");
const osMutexAttr_t powSupMutexAttrs = VD_MUTEX_ATTRS("powSupVDMutex");

void initVehicleData() {
	VehicleData = {0};
	VehicleData.apps_lock = osMutexNew(&appsMutexAttrs);
	VehicleData.bps_lock = osMutexNew(&bpsMutexAttrs);
	VehicleData.sas_lock = osMutexNew(&sasMutexAttrs);
	VehicleData.inverter_lock = osMutexNew(&inverterMutexAttrs);
	VehicleData.fsm_state_lock = osMutexNew(&fsmStateMutexAttrs);
	VehicleData.powsup_lock = osMutexNew(&powSupMutexAttrs);
}
