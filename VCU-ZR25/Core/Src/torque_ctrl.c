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
	osMutexAcquire(vdb_torquectrl_lockHandle, osWaitForever);
	VehicleData.torquectrl = torquectrl;
	osMutexRelease(vdb_torquectrl_lockHandle);
}

ControlMode_t check_control_mode(){
	ControlMode_t control_mode = 0;
	return control_mode;
}

TorqueCtrlData_t torquectrl = {.controlmode = 0, .torque_percent = 10};
uint8_t preFaultTorque = 0;

void StartTorqueCtrlTask(void *argument) {

	while (1) {
		torquectrl.controlmode = check_control_mode();
		update_torque_ctrl_data(torquectrl);
		osDelay(TORQUE_CONTROL_TASK_PERIOD);
	}
}

void update_control_mode(ControlMode_t ctrl_mode){
	torquectrl.controlmode = ctrl_mode;
}

void increment_torque_limit(){
	if(torquectrl.torque_percent <= 90 && torquectrl.torque_percent != 0){
		torquectrl.torque_percent = torquectrl.torque_percent + 10;
	}
}

void decrement_torque_limit(){
	if(torquectrl.torque_percent >= 20){
		torquectrl.torque_percent = torquectrl.torque_percent - 10;
	}
}

void torque_fault_callback(uint8_t value){
	if (value){
		if(torquectrl.torque_percent){
			preFaultTorque = torquectrl.torque_percent;
		}
		torquectrl.torque_percent = 0;
	}
	else if (preFaultTorque){
		torquectrl.torque_percent = preFaultTorque;
		preFaultTorque = 0;
	}
}
