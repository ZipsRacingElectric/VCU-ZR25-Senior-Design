/*
 * cooling_system.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

#include "vehicle_data.h"
#include "cooling_system.h"
#include "main.h"

void update_cooling_data(CoolingData_t coolingData) {
	osMutexAcquire(VehicleData.cooling_lock, osWaitForever);
	VehicleData.cooling = coolingData;
	osMutexRelease(VehicleData.cooling_lock);
}

CoolingData_t check_cooling_data() {
	CoolingData_t coolingData = {0};
	return coolingData;
}

void StartCoolingTask(void *argument) {
	while (1) {
		CoolingData_t coolingData = check_cooling_data();
		update_cooling_data(coolingData);
		osDelay(COOLING_TASK_PERIOD);
	}
}
