/*
 * dashboard.c
 *
 *  Created on: Feb 22, 2025
 *      Author: jmw398
 */

#include "vehicle_data.h"
#include "dashboard.h"
#include "main.h"
#include "torque_ctrl.h"

void update_dashboard_data(DashboardData_t dashboardData) {
	osMutexAcquire(VehicleData.dashboard_lock, osWaitForever);
	VehicleData.dashboard = dashboardData;
	osMutexRelease(VehicleData.dashboard_lock);
}

DashboardData_t dashboardData = {0};

void StartDashboardTask(void *argument) {
	while (1) {
		update_dashboard_data(dashboardData);
		osDelay(DASHBOARD_TASK_PERIOD);
	}
}

void DashboardCriticalFaultCallback(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(GPIOA, DASH_INPUT_2_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = DASH_INPUT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, DASH_INPUT_2_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = DASH_INPUT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT_1_GPIO_Port, &GPIO_InitStruct);
}

void DashboardFaultCallback(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(GPIOA, DASH_INPUT_1_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = DASH_INPUT_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, DASH_INPUT_1_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = DASH_INPUT_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT_1_GPIO_Port, &GPIO_InitStruct);
}

void DashboardDRSToggleCallback(uint16_t GPIO_Pin){
	uint8_t pin_state = HAL_GPIO_ReadPin(DASH_INPUT_2_GPIO_Port, DASH_INPUT_2_Pin);
	if (pin_state){
		dashboardData.DRSState = !dashboardData.DRSState;
	}
	/* TODO: CAN function to update the DRS state, put in task loop if sent in intervals*/
}

void DashboardTorqueLimitCallback(uint16_t GPIO_Pin){
	/* TODO: Complete function once torque control handler done */
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
