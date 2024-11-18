/*
 * driver_sensors.c
 *
 * This handles all functions relating to measuring driver input sensors.
 *
 *  Created on: Nov 13, 2024
 *      Author: bglen
 */

// Includes
#include "driver_sensors.h"

#include <math.h>

// Constants for Conversion and Validation
#define APPS1_CHANNEL	 	  1
#define APPS2_CHANNEL	      2
#define BPS_FRONT_CHANNEL	  3
#define BPS_REAR_CHANNEL	  4
#define STEER_ANGLE_CHANNEL	  5

#define ADC_MAX_VALUE         4095 // 2^12 - 1
#define ADC_REF_VOLTAGE       3300 // V * 1000

#define APPS_1_MIN_VOLTAGE    0500 // V * 1000
#define APPS_2_MIN_VOLTAGE    0250 // V * 1000
#define APPS_1_MAX_VOLTAGE    4500 // V * 1000
#define APPS_2_MAX_VOLTAGE    2250 // V * 1000
#define APPS_MIN_DELTA_V	  0030 // Minimum allowable voltage difference V * 1000
#define APPS_MAX_DELTA_P   	  0100 // Maximum allowable pedal position difference % * 10
#define APPS_DEADZONE		  0050 // Upper and lower pedal deadzone in % * 10
#define APPS_OUT_OF_RANGE     0100 // Allowable voltage beyond calibration limits before fault V * 1000

#define BPS_MAX_VOLTAGE       4500 // V * 1000
#define BPS_MIN_VOLTAGE       0500 // V * 1000
#define BPS_MAX_PRESSURE      20684 // pressure in kPA
#define BPS_MIN_PRESSURE      0000 // pressure in kPA

#define STEERING_MAX_ANGLE    4500 // V * 1000
#define STEERING_MIN_ANGLE    -4500 // V * 1000

// Static variables
APPSSensor_t s_apps = {.raw_value_1 = 0, .raw_value_2 = 0, .voltage_1 = 0, .voltage_2 = 0, .percent_1 = 0, .percent_2 = 0, .percent = 0, .plausible = false};
BPSSensor_t s_bps_front = {.raw_value = 0, .voltage = 0, .pressure = 0, .plausible = false};
BPSSensor_t s_bps_rear = {.raw_value = 0, .voltage = 0, .pressure = 0, .plausible = false};
SteeringAngleSensor_t s_steering_angle = {.raw_value = 0, .angle = 0, .plausible = false};

// Static calibration variables. These hold the 0% and 100% setpoints
static uint16_t apps_1_min = 0600; // V * 1000
static uint16_t apps_1_max = 4400; // V * 1000
static uint16_t apps_2_min = 0350; // V * 1000
static uint16_t apps_2_max = 2150; // V * 1000

// Private Function Prototypes
static uint16_t read_adc(ADC_HandleTypeDef *adc);
static uint16_t adc_to_voltage(uint16_t adc_value);
static void calc_apps_percent(APPSSensor_t *apps);
static void calc_bps_pressure(BPSSensor_t *bps);
static void calc_steering_angle(SteeringAngleSensor_t *steering_angle);
static uint16_t abs_diff(uint16_t v1, uint16_t v2);
static bool validate_apps(APPSSensor_t apps);
static bool validate_bps(BPSSensor_t bps);
static bool validate_steering_angle(SteeringAngleSensor_t steering_angle);

// Public Functions

/*
 * Initializes the peripherals used for measuring the driver input sensors
 */
void driver_input_init(void)
{
    // Initialize ADC channels, GPIOs, and other hardware interfaces
}

/*
 * Reads in the raw sensor data and updates the sensor variables
 */
void read_driver_input(ADC_HandleTypeDef *adc)
{
    // Read Raw ADC Values
	s_apps.raw_value_1 = read_adc(adc);
    s_apps.raw_value_2 = read_adc(adc);
    s_bps_front.raw_value = read_adc(adc);
    s_bps_rear.raw_value = read_adc(adc);

    // Convert Raw Values to Voltages
    s_apps.voltage_1 = adc_to_voltage(s_apps.raw_value_1);
    s_apps.voltage_2 = adc_to_voltage(s_apps.raw_value_2);
    s_bps_front.voltage = adc_to_voltage(s_bps_front.raw_value);
    s_bps_rear.voltage = adc_to_voltage(s_bps_rear.raw_value);

    // Convert Voltages to Physical Values
    (void)calc_apps_percent(&s_apps);
    (void)calc_bps_pressure(&s_bps_front);
    (void)calc_bps_pressure(&s_bps_rear);

    // Perform Plausibility Checking
    // Handling of plausibility is outside the scope of measuring driver input sensors
    s_apps.plausible = validate_apps(s_apps);
    s_bps_front.plausible = validate_bps(s_bps_front);
    s_bps_rear.plausible = validate_bps(s_bps_rear);
    s_steering_angle.plausible = validate_steering_angle(s_steering_angle);
}

/*
 * Returns an APPSSensor_t struct with the current APPS data in it
 */
APPSSensor_t get_apps_data(void)
{
	return s_apps;
}

/*
 * Returns front brake pressure in kPa
 */
uint16_t get_front_brake_pressure(void)
{
	return s_bps_front.pressure;
}

/*
 * Returns rear brake pressure in kPa
 */
uint16_t get_rear_brake_pressure(void)
{
	return s_bps_rear.pressure;
}

/*
 * Returns steering angle in radians * 1000
 */
uint16_t get_steering_angle(void)
{
	return s_steering_angle.angle;
}

/*
 * Calibrates the constants used to determine APPS position. Returns false if calibration failed.
 */
bool calibrate_apps(uint16_t apps_1_pedal_min, uint16_t apps_1_pedal_max, uint16_t apps_2_pedal_min, uint16_t apps_2_pedal_max)
{
	// Check if calibration values are in a valid range
	if (apps_1_pedal_min > apps_1_pedal_max || apps_2_pedal_min > apps_2_pedal_max)
	{
		return false;
	}

	// Apply limits that are 10% beyond what the driver recorded during calibration
	// TODO: floor these so they cannot be beyond the sensor voltage limits
	apps_1_min = apps_1_pedal_min;
	apps_2_min = apps_2_pedal_min;
	apps_1_max = apps_1_pedal_max;
	apps_2_max = apps_2_pedal_max;

	return true;
}

/*
 * Calibrates the constants used to determine BPS pressure
 */
bool calibrate_bps(void)
{
	// TODO
	return false;
}

/*
 * Calibrates the constants used to determine steering angles
 */
bool calibrate_steering(void)
{
	// TODO
	return false;
}

// Private Functions

/*
 * Reads in the raw ADC values from hardware
 */
static uint16_t read_adc(ADC_HandleTypeDef *adc)
{
	// Read ADC value
	HAL_ADC_Start(adc);
	HAL_ADC_PollForConversion(adc, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(adc);
}

/*
 * Convert raw 12-bit ADC readings to voltages in V * 1000
 */
static uint16_t adc_to_voltage(uint16_t adc_value)
{
	// Multiply adc_value by the reference voltage in millivolts
	// Then divide by the maximum ADC value to get the voltage
    uint32_t mv = ((uint32_t)adc_value * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
	return (uint16_t)mv;
}

/*
 * Calculate individual APPS channel percentage and final pedal percentage
 */
static void calc_apps_percent(APPSSensor_t *apps)
{

	// Calculate voltage setpoints considering the pedal deadzone
	uint32_t apps_1_v_min = apps_1_min + ((abs_diff(apps_1_max, apps_1_min) * APPS_DEADZONE) / 1000);
	uint32_t apps_1_v_max = apps_1_max - ((abs_diff(apps_1_max, apps_1_min) * APPS_DEADZONE) / 1000);
	uint32_t apps_2_v_min = apps_2_min + ((abs_diff(apps_2_max, apps_2_min) * APPS_DEADZONE) / 1000);
	uint32_t apps_2_v_max = apps_2_max - ((abs_diff(apps_2_max, apps_2_min) * APPS_DEADZONE) / 1000);

	// Default percent is zero, accounts for <= apps_v_min condition
	uint32_t p1 = 0;
	uint32_t p2 = 0;

	// Condition: voltage is within the linear range
	if (apps->voltage_1 > apps_1_v_min && apps->voltage_1 < apps_1_v_max)
	{
		// Multiply by 1000 to get percentage * 10, then divide by voltage range defined by calibration
		p1 = (((uint32_t)apps->voltage_1 - apps_1_v_min) * 1000) / (apps_1_v_max - apps_1_v_min);
	}

	// Condition: voltage is within the linear range
	if (apps->voltage_2 > apps_2_v_min && apps->voltage_2 < apps_2_v_max)
	{
		// Multiply by 1000 to get percentage * 10, then divide by voltage range defined by calibration
		p2 = (((uint32_t)apps->voltage_2 - apps_2_v_min) * 1000) / (apps_2_v_max - apps_2_v_min);
	}

	// Condition: voltage is above upper limit and into/past the deadzone
	if(apps->voltage_1 > apps_1_v_max)
	{
		// Limit to 100%
		p1 = 1000;
	}

	// Condition: voltage is above upper limit and into/past the deadzone
	if(apps->voltage_2 > apps_2_v_max)
	{
		// Limit to 100%
		p2 = 1000;
	}

	// Average the two channels to determine the actual pedal position
	// TODO: a better algorithm would be nice
	uint32_t pedal_percent = (p1 + p2) * 1000 / 2000;


	apps->percent_1 = (uint16_t)p1;
	apps->percent_2 = (uint16_t)p2;
	apps->percent = (uint16_t)pedal_percent;
}

/*
 * Calculate the pressure from the BPS sensor voltage
 */
static void calc_bps_pressure(BPSSensor_t *bps)
{
    // TODO
}

/*
 * Calculate the steering angle in radians
 */
static void calc_steering_angle(SteeringAngleSensor_t *steering_angle)
{
    // TODO
}

/*
 * Calculates absolute difference of unsigned integer to avoid wrap around
 */
static uint16_t abs_diff(uint16_t v1, uint16_t v2)
{
    uint16_t delta;

    if (v1 > v2) {
        delta = v1 - v2;
    } else {
        delta = v2 - v1;
    }

    return delta;
}

/*
 * APPS Plausibility Check
 * - plausibility conditions are checked here
 * - disabling of the powertrain after 100 ms is not handled here, out of scope
 *
 * TODO: it would be nice to report different plausibility faults in the future
 */
bool validate_apps(APPSSensor_t apps)
{
		// Check for short to ground
	    if (apps.voltage_1 < APPS_1_MIN_VOLTAGE || apps.voltage_2 < APPS_2_MIN_VOLTAGE)
	    {
	        return false;
	    }

	    // Check for short to power
	    if (apps.voltage_1 > APPS_1_MAX_VOLTAGE || apps.voltage_2 > APPS_2_MAX_VOLTAGE)
	    {
	        return false;
	    }

	    // Check for short to sensor output
	    if ((uint16_t)abs_diff(apps.voltage_1, apps.voltage_2) < APPS_MIN_DELTA_V)
	    {
	        return false;
	    }

	    // Check for out of range for APPS1
	    if (apps.voltage_1 < (apps_1_min - APPS_OUT_OF_RANGE) || apps.voltage_1 > (apps_1_max + APPS_OUT_OF_RANGE))
	    {
	        return false;
	    }

	    // Check for out of range for APPS2
	    if (apps.voltage_2 < (apps_1_min - APPS_OUT_OF_RANGE) || apps.voltage_2 > (apps_1_max + APPS_OUT_OF_RANGE))
	    {
	        return false;
	    }

	    // Check for a significant deviation between APPS1 and APPS2 calculated percentages
	    if ((uint16_t)abs_diff(apps.percent_1, apps.percent_2) > APPS_MAX_DELTA_P)
	    {
	    	return false;
	    }

	    // If none of the conditions are met, plausibility is valid
	    return true;
}

/*
 * BPS Plausibility Check
 */
bool validate_bps(BPSSensor_t bps)
{
    // TODO: Implement BPS plausibility logic
    return true;
}

/*
 * Steering Angle Plausibility Check
 */
bool validate_steering_angle(SteeringAngleSensor_t steering_angle)
{
	// TODO
	return true;
}

