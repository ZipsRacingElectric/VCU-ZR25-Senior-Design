/*
 * torque_ctrl.c
 *
 *  Created on: Feb 23, 2025
 *      Author: John
 */

#include "vehicle_data.h"
#include "power_supply.h"
#include "main.h"
#include "driver_sensors.h"

void update_torque_ctrl_data(TorqueCtrlData_t torquectrl) {
	osMutexAcquire(VehicleData.torquectrl_lock, osWaitForever);
	VehicleData.torquectrl = torquectrl;
	osMutexRelease(VehicleData.torquectrl_lock);
}

ControlMode_t check_control_mode(){
	ControlMode_t control_mode = 0;
	return control_mode;
}

TorqueCtrlData_t torquectrl = {0};

void StartTorqueCtrlTask(void *argument) {

	while (1) {
		torquectrl.controlmode = check_control_mode();
		update_torque_ctrl_data(torquectrl);
		osDelay(TORQUE_CONTROL_TASK_PERIOD);
	}
}

void increment_torque_limit(){
	if(torquectrl.torque_percent < 100){
		torquectrl.torque_percent = torquectrl.torque_percent + 10;
	}
}

void decrement_torque_limit(){
	if(torquectrl.torque_percent > 0){
		torquectrl.torque_percent = torquectrl.torque_percent - 10;
	}
}
