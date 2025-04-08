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
#include "amk_can.h"
#include "stm32f4xx_hal_flash.h"

void update_torque_ctrl_data(TorqueCtrlData_t torquectrl) {
	osMutexAcquire(vdb_torquectrl_lockHandle, osWaitForever);
	VehicleData.torquectrl = torquectrl;
	osMutexRelease(vdb_torquectrl_lockHandle);
}

ControlMode_t check_control_mode(){
	ControlMode_t control_mode = 0;
	return control_mode;
}

TorqueCtrlData_t torquectrl = {.controlmode = 0, .torque_percent = 10, .torque_value = 0};
uint8_t preFaultTorque = 10;

void StartTorqueCtrlTask(void *argument) {
	while (1) {
		torquectrl.controlmode = check_control_mode();
		update_torque_output();
		update_torque_ctrl_data(torquectrl);
		osDelay(TORQUE_CONTROL_TASK_PERIOD);
	}
}

void update_control_mode(ControlMode_t ctrl_mode){
	torquectrl.controlmode = ctrl_mode;
}

void update_torque_output(){
	if (!VehicleData.bps_front.brakes_engaged && !VehicleData.bps_rear.brakes_engaged){
		torquectrl.torque_value = ((float)(VehicleData.apps.percent_1 * torquectrl.torque_percent) / 100000.0f) * (float)MOTOR_POS_TORQUE_LIMIT;
	}
	else if (VehicleData.inverter.motor_info_rl.motorFeedbackMessage.fields.ACTUAL_SPEED_VALUE){
		float bps_level = (float)(VehicleData.bps_front.voltage - BPS_MIN_VOLTAGE) / (float)(BPS_MAX_VOLTAGE - BPS_MIN_VOLTAGE);
		torquectrl.torque_value = (bps_level * (float)torquectrl.torque_percent / 100.0f) * (float)MOTOR_NEG_TORQUE_LIMIT;
	}
	else {
		torquectrl.torque_value = 0;
	}
	amkTorqueSetpoints setpoint = {.rear_left = torquectrl.torque_value};
	AMKSetInverterTorqueSetpoints(setpoint);
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
