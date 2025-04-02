/*
 * torque_ctrl.c
 *
 *  Created on: Feb 23, 2025
 *      Author: John, Tetra
 */

#include "vehicle_data.h"
#include "power_supply.h"
#include "main.h"
#include "driver_sensors.h"
#include "stm32f4xx_hal_flash.h"

__attribute__((__section__(".rodata"))) // Tells the linker that this should live in read-only flash memory.
const torqueControlParameters_t params = {
	.td_params = {},
	// Values specified here are loaded each time the VCU is reprogrammed.
	// The `program_pid_params` function can change this at runtime, but those
	// values will be overwritten during programming if they are not stored here.
	.pid_params = {}
};

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
uint8_t preFaultTorque = 10;

void StartTorqueCtrlTask(void *argument) {
	while (1) {
		torquectrl.controlmode = check_control_mode();
		update_torque_ctrl_data(torquectrl);
		osDelay(TORQUE_CONTROL_TASK_PERIOD);
	}
}

void program_pid_params(torqueControlPIDParams_t *new_pid_params) {
	size_t pid_size = sizeof(torqueControlPIDParams_t);
	uint8_t * src = (uint8_t*) new_pid_params;
	uint8_t * dst = (uint8_t*) &params.pid_params;
	for (int i = 0; i<pid_size; i++) {
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)(src+i), *(dst+i));
		HAL_FLASH_Lock();
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
	if (torquectrl.torque_percent){
		preFaultTorque = torquectrl.torque_percent;
	}
	if (value){
		torquectrl.torque_percent = 0;
	}
	else{
		torquectrl.torque_percent = preFaultTorque;
	}
}
