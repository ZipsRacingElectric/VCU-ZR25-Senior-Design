/*
 * dashboard.c
 *
 *  Created on: Feb 22, 2025
 *      Author: jmw398
 */

/*
 * TODO:
 * - DashboardDRSToggleCallback: CAN comm update DRS State
 */

#include "vehicle_data.h"
#include "dashboard.h"
#include "main.h"
#include "torque_ctrl.h"

void update_dashboard_data(DashboardData_t dashboardData) {
	osMutexAcquire(vdb_dashboard_lockHandle, osWaitForever);
	VehicleData.dashboard = dashboardData;
	osMutexRelease(vdb_dashboard_lockHandle);
}

DashboardData_t dashboardData = {0};

void StartDashboardTask(void *argument) {
	while (1) {
		update_dashboard_data(dashboardData);
		osDelay(DASHBOARD_TASK_PERIOD);
	}
}

void DashboardFaultCallback(uint8_t value){

	if (value){
		HAL_GPIO_WritePin(GPIOA, DASH_INPUT_1_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOA, DASH_INPUT_1_Pin, GPIO_PIN_RESET);
	}
}

void DashboardDRSToggleCallback(uint16_t GPIO_Pin){
	uint8_t pin_state = HAL_GPIO_ReadPin(DASH_INPUT_2_GPIO_Port, DASH_INPUT_2_Pin);
	if (pin_state){
		dashboardData.DRSState = !dashboardData.DRSState;
	}
}

void DashboardTorqueLimitCallback(uint16_t GPIO_Pin){
	if (GPIO_Pin == DASH_INPUT_3_Pin){
		uint8_t pin_state = HAL_GPIO_ReadPin(DASH_INPUT_3_GPIO_Port, DASH_INPUT_3_Pin);
		if (pin_state){
			increment_torque_limit();
		}
	}
	else {
		uint8_t pin_state = HAL_GPIO_ReadPin(DASH_INPUT_4_GPIO_Port, DASH_INPUT_4_Pin);
		if (pin_state){
			decrement_torque_limit();
		}
	}
}
