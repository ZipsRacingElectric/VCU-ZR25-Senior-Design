/*
 * power_supply.c
 *
 *  Created on: Feb 12, 2025
 *      Author: John
 */

#include "power_supply.h"
#include "main.h"

// ADC thresholds for 5V and 3.3V rails
#define ADC_5V_THRESHOLD_LOW   3123   // ADC value for 4.8V
#define ADC_5V_THRESHOLD_HIGH  3383   // ADC value for 5.2V
#define ADC_3V3_THRESHOLD_LOW  2712   // ADC value for 2.8V
#define ADC_3V3_THRESHOLD_HIGH 3390   // ADC value for 3.5V

PowSupData_t powsup = {
		.value5V = true,
		.value3V = true
};

void StartPwrSupTask(
		ADC_HandleTypeDef hadc1,
		ADC_ChannelConfTypeDef sConfig
) {
	uint32_t adc_value_5V = 0;
	uint32_t adc_value_3V3 = 0;

	// Read 5V signal on PA0
	temp_channel = sConfig.Channel;
	sConfig.Channel = ADC_CHANNEL_0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_value_5V = HAL_ADC_GetValue(&hadc1);

	// Check if 5V signal within range
	if (adc_value_5V >= ADC_5V_THRESHOLD_LOW && adc_value_5V <= ADC_5V_THRESHOLD_HIGH) {
	  powsup.value5V = true;
	} else {
	  powsup.value3V = false;
	}

	// Change to PA1 for 3.3V signal

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	// Read 3.3V signal on PA1

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_value_3V3 = HAL_ADC_GetValue(&hadc1);

	// Check if 3.3V signal within the range

	if (adc_value_3V3 >= ADC_3V3_THRESHOLD_LOW && adc_value_3V3 <= ADC_3V3_THRESHOLD_HIGH) {
	  powsup.value3V = true;
	} else {
	  powsup.value3V = false;
	}

	sConfig.Channel = temp_channel;

	HAL_Delay(50);
}
