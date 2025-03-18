/*
 * cooling_system.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

/*
 * TODO:
 * - check_cooling_data: CAN comm for updating coolingData
 */

#include "vehicle_data.h"
#include "cooling_system.h"
#include "main.h"

void update_cooling_data(CoolingData_t coolingData) {
	osMutexAcquire(vdb_cooling_lockHandle, osWaitForever);
	VehicleData.cooling = coolingData;
	osMutexRelease(vdb_cooling_lockHandle);
}

bool isOverTemp(uint32_t temp){
	return temp >= MAX_TEMP_THRESHOLD;
}

bool isUnderTemp(uint32_t temp){
	return temp <= MIN_TEMP_THRESHOLD;
}

CoolingData_t check_cooling_data() {
	CoolingData_t coolingData = {0};

	if (isOverTemp(coolingData.fl_inverter_temp) | isOverTemp(coolingData.fl_motor_temp)
			| isOverTemp(coolingData.rl_inverter_temp) | isOverTemp(coolingData.rl_motor_temp)){
		CoolingSystemTurnOnLeft();
	}
	else if (isUnderTemp(coolingData.fl_inverter_temp) & isUnderTemp(coolingData.fl_motor_temp)
			& isUnderTemp(coolingData.rl_inverter_temp) & isUnderTemp(coolingData.rl_motor_temp)){
		CoolingSystemTurnOffLeft();
	}

	if (isOverTemp(coolingData.fr_inverter_temp) | isOverTemp(coolingData.fr_motor_temp)
			| isOverTemp(coolingData.rr_inverter_temp) | isOverTemp(coolingData.rr_motor_temp)){
		CoolingSystemTurnOnRight();
	}
	else if (isUnderTemp(coolingData.fr_inverter_temp) & isUnderTemp(coolingData.fr_motor_temp)
			& isUnderTemp(coolingData.rr_inverter_temp) & isUnderTemp(coolingData.rr_motor_temp)){
		CoolingSystemTurnOffRight();
	}
	return coolingData;
}

void StartCoolingTask(void *argument) {
	while (1) {
		CoolingData_t coolingData = check_cooling_data();
		update_cooling_data(coolingData);
		osDelay(COOLING_TASK_PERIOD);
	}
}

void CoolingSystemTurnOnLeft(){
	HAL_GPIO_WritePin(GPIOB, FAN_1_CONTROL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, PUMP_1_CONTROL_Pin, GPIO_PIN_SET);
}

void CoolingSystemTurnOnRight(){
	HAL_GPIO_WritePin(GPIOB, FAN_2_CONTROL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, PUMP_2_CONTROL_Pin, GPIO_PIN_SET);
}

void CoolingSystemTurnOffLeft(){
	HAL_GPIO_WritePin(GPIOB, FAN_1_CONTROL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, PUMP_1_CONTROL_Pin, GPIO_PIN_RESET);
}

void CoolingSystemTurnOffRight(){
	HAL_GPIO_WritePin(GPIOB, FAN_2_CONTROL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, PUMP_2_CONTROL_Pin, GPIO_PIN_RESET);
}
