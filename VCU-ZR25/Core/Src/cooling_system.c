/*
 * cooling_system.c
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

#include "vehicle_data.h"
#include "cooling_system.h"
#include "main.h"

bool isOverTemp(uint16_t temp){
	return temp >= MAX_TEMP_THRESHOLD;
}

bool isUnderTemp(uint16_t temp){
	return temp <= MIN_TEMP_THRESHOLD;
}

void check_cooling_data() {
	osMutexAcquire(vdb_inverter_lockHandle, osWaitForever);
	AMKState_t inverter = VehicleData.inverter;
	osMutexRelease(vdb_inverter_lockHandle);

	if (isOverTemp(inverter.motor_statistics_fl.temp_sensor) |
			isOverTemp(inverter.motor_statistics_rl.temp_sensor)){
		CoolingSystemTurnOnLeft();
	}
	else if (isUnderTemp(inverter.motor_statistics_fl.temp_sensor) &
			isUnderTemp(inverter.motor_statistics_rl.temp_sensor)){
		CoolingSystemTurnOffLeft();
	}

	if (isOverTemp(inverter.motor_statistics_fr.temp_sensor)
			| isOverTemp(inverter.motor_statistics_rr.temp_sensor)){
		CoolingSystemTurnOnRight();
	}
	else if (isUnderTemp(inverter.motor_statistics_fr.temp_sensor)
			| isUnderTemp(inverter.motor_statistics_rr.temp_sensor)){
		CoolingSystemTurnOffRight();
	}
}

void StartCoolingTask(void *argument) {
	while (1) {
		check_cooling_data();
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
