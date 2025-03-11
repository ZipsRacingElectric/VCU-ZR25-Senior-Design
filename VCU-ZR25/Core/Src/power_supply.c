/*
 * power_supply.c
 *
 *  Created on: Feb 12, 2025
 *      Author: John
 */

#include "vehicle_data.h"
#include "power_supply.h"
#include "main.h"
#include "driver_sensors.h"

void update_power_supply_data(PowSupData_t powsup) {
	osMutexAcquire(vdb_powsup_lockHandle, osWaitForever);
	VehicleData.powsup = powsup;
	osMutexRelease(vdb_powsup_lockHandle);
}

// Returns true if powsup changed
PowSupData_t check_power_supply(ADC_HandleTypeDef hadc1, ADC_ChannelConfTypeDef sConfig) {

	PowSupData_t powsup = {false, false};

	// Read 5V signal on PA0
	/* TODO: Figure out how to measure the 5V power */
	sConfig.Channel = ADC_CHANNEL_0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adc_value_5V = HAL_ADC_GetValue(&hadc1);
	adc_value_5V = (adc_value_5V * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
	adc_value_5V = (adc_value_5V * 5000) / 3000;

	// Check if 5V signal within range
	if (adc_value_5V >= ADC_5V_MIN_VOLTAGE && adc_value_5V <= ADC_5V_MAX_VOLTAGE) {
	    powsup.value5V = true;
	} else {
	    powsup.value3V = false;
	}

	// Change to PA4 for 3.3V signal
	sConfig = (ADC_ChannelConfTypeDef){0};
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	// Read 3.3V signal on PA4
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adc_value_3V3 = HAL_ADC_GetValue(&hadc1);
	adc_value_3V3 = (adc_value_3V3 * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;

	// Check if 3.3V signal within the range
	if (adc_value_3V3 >= ADC_3V_MIN_VOLTAGE && adc_value_3V3 <= ADC_3V_MAX_VOLTAGE) {
	  powsup.value3V = true;
	} else {
	  powsup.value3V = false;
	}
	
	return powsup;
}

void StartPwrSupTask(
	void* void_args
) {
	powSupTaskArgs_t* args = (powSupTaskArgs_t*) void_args;
	ADC_HandleTypeDef hadc1 = args->hadc1;
	ADC_ChannelConfTypeDef sConfig = args->sConfig;
	
	while (1) {
		PowSupData_t powsup = check_power_supply(hadc1, sConfig);
		update_power_supply_data(powsup);
		fsm_power_callback(powsup);
		osDelay(POWER_SUPPLY_TASK_PERIOD); 
	}
}

void fsm_power_callback(PowSupData_t powsup){
	uint8_t flag;
	uint8_t value;
		if (powsup.value3V) {
			flag = FLAG_INDEX_GLVMS_TURNED_ON;
			value = 1;
			fsm_flag_callback(flag, value);
		}
		else{
			flag = FLAG_INDEX_GLVMS_TURNED_ON;
			value = 0;
			fsm_flag_callback(flag, value);
		}
}
