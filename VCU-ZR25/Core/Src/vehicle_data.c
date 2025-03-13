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

extern const osMutexAttr_t appsMutexAttrs;
extern const osMutexAttr_t bpsFrontMutexAttrs;
extern const osMutexAttr_t bpsRearMutexAttrs;
extern const osMutexAttr_t sasMutexAttrs;
extern const osMutexAttr_t inverterMutexAttrs;
extern const osMutexAttr_t fsmStateMutexAttrs;
extern const osMutexAttr_t powSupMutexAttrs;
extern const osMutexAttr_t coolingMutexAttrs;
extern const osMutexAttr_t dashboardMutexAttrs;
extern const osMutexAttr_t torquectrlMutexAttrs;


void initVehicleData() {
	VehicleData = (VehicleData_t){0};

}

VehicleData_t get_vehicle_data(){
	return VehicleData;
}
