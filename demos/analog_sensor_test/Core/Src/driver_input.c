/*
 * driver_input.c
 *
 * This handles all functions relating to measuring driver input sensors.
 *
 *  Created on: Nov 13, 2024
 *      Author: bglen
 */

// Includes
#include "driver_input.h"
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
#define APPS_MAX_POSITION     1000 // % * 10
#define APPS_MIN_POSITION     0000 // % * 10
#define APPS_MIN_DELTA_V	  0030 // Minimum allowable voltage difference V * 1000
#define APPS_MAX_DIFFERENCE   0100 // Maximum allowable pedal position difference % * 10

#define BPS_MAX_VOLTAGE       4500 // V * 1000
#define BPS_MIN_VOLTAGE       0500 // V * 1000
#define BPS_MAX_PRESSURE      1000 // pressure in
#define BPS_MIN_PRESSURE      0000 // V * 1000

#define STEERING_MAX_ANGLE    4500 // V * 1000
#define STEERING_MIN_ANGLE    -4500 // V * 1000

// Static variables
static uint16_t apps_1_min = 0600; // V * 1000
static uint16_t apps_1_max = 4400; // V * 1000
static uint16_t apps_2_min = 0350; // V * 1000
static uint16_t apps_2_max = 2150; // V * 1000

// Private Function Prototypes
static uint16_t read_adc(uint16_t channel);
static uint16_t adc_to_voltage(uint16_t adc_value);
static void get_apps_position(APPSSensor_t *apps);
static void get_bps_pressure(BPSSensor_t *bps);
static void get_steering_angle(SteeringAngleSensor_t *steering_angle);
static uint16_t abs_diff(uint16_t v1, uint16_t v2);
static bool validate_apps(APPSSensor_t *apps);
static bool validate_bps(BPSSensor_t *bps);
static bool validate_steering_angle(SteeringAngleSensor_t *steering_angle);

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
void read_driver_input(APPSSensor_t *apps, BPSSensor_t *bps_front, BPSSensor_t *bps_rear, SteeringAngleSensor_t *steering_angle)
{
    // Read Raw ADC Values
	apps->raw_value_1 = read_adc(APPS1_CHANNEL);
    apps->raw_value_2 = read_adc(APPS2_CHANNEL);
    bps_front->raw_value = read_adc(BPS_FRONT_CHANNEL);
    bps_rear->raw_value = read_adc(BPS_REAR_CHANNEL);

    // Convert Raw Values to Voltages
    apps->voltage_1 = adc_to_voltage(apps->raw_value_1);
    apps->voltage_2 = adc_to_voltage(apps->raw_value_2);
    bps_front->voltage = adc_to_voltage(bps_front->raw_value);
    bps_rear->voltage = adc_to_voltage(bps_rear->raw_value);

    // Convert Voltages to Physical Values
    (void)get_apps_position(apps);
    (void)get_bps_pressure(bps_front);
    (void)get_bps_pressure(bps_rear);

    // Perform Plausibility Checking
    // Handling of plausibility is outside the scope of measuring driver input sensors
    apps->plausible = validate_apps(apps);
    bps_front->plausible = validate_bps(bps_front);
    bps_rear->plausible = validate_bps(bps_rear);
    steering_angle->plausible = validate_steering_angle(steering_angle);
}

/*
 * Calibrates the constants used to determine APPS position
 */
bool calibrate_apps(APPSSensor_t *apps)
{
	// TODO
	return false;
}

/*
 * Calibrates the constants used to determine BPS pressure
 */
bool calibrate_bps(BPSSensor_t *bps)
{
	// TODO
	return false;
}

/*
 * Calibrates the constants used to determine steering angles
 */
bool calibrate_steering(SteeringAngleSensor_t *steering_angle)
{
	// TODO
	return false;
}

// Private Functions

/*
 * Reads in the raw ADC values from hardware
 */
static uint16_t read_adc(uint16_t channel)
{
    // TODO: Implement ADC reading based on hardware
    return 0;
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
static void get_apps_position(APPSSensor_t *apps)
{
	// TODO: sensor calibration values must be verified before calling this function

	// Multiply by 1000 to get percentage * 10, then divide by voltage range defined by calibration
	uint32_t p1 = (((uint32_t)apps->voltage_1 - apps_1_min) * 1000) / (apps_1_max - apps_1_min);
	uint32_t p2 = (((uint32_t)apps->voltage_2 - apps_2_min) * 1000) / (apps_2_max - apps_2_min);

	// Average the two channels to determine pedal position
	uint32_t pos = (p1 + p2) / 2;

	apps->percent_1 = (uint16_t)p1;
	apps->percent_2 = (uint16_t)p2;
	apps->position = (uint16_t)pos;
}

/*
 * Calculate the pressure from the BPS sensor voltage
 */
static void get_bps_pressure(BPSSensor_t *bps)
{
    // TODO
}

/*
 * Calculate the steering angle in radians
 */
static void get_steering_angle(SteeringAngleSensor_t *steering_angle)
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
bool validate_apps(APPSSensor_t *apps)
{
		// Check for short to ground
	    if (apps->voltage_1 < APPS_1_MIN_VOLTAGE || apps->voltage_2 < APPS_2_MIN_VOLTAGE)
	    {
	        return false;
	    }

	    // Check for short to power
	    if (apps->voltage_1 > APPS_1_MAX_VOLTAGE || apps->voltage_2 > APPS_2_MAX_VOLTAGE)
	    {
	        return false;
	    }

	    // Check for short to sensor output
	    if ((uint16_t)abs_diff(apps->voltage_1, apps->voltage_2) < APPS_MIN_DELTA_V)
	    {
	        return false;
	    }

	    // Check for out of range for APPS1
	    if (apps->voltage_1 < apps_1_min || apps->voltage_1 > apps_1_max)
	    {
	        return false;
	    }

	    // Check for out of range for APPS2
	    if (apps->voltage_2 < apps_2_min || apps->voltage_2 > apps_2_max)
	    {
	        return false;
	    }

	    // Check for a significant deviation between APPS1 and APPS2 calculated percentages
	    if ((uint16_t)abs_diff(apps->percent_1, apps->percent_2) > APPS_MAX_DIFFERENCE)
	    {
	    	return false;
	    }

	    // If none of the conditions are met, plausibility is valid
	    return true;
}

/*
 * BPS Plausibility Check
 */
bool validate_bps(BPSSensor_t *bps)
{
    // TODO: Implement BPS plausibility logic
    return true;
}

/*
 * Steering Angle Plausibility Check
 */
bool validate_steering_angle(SteeringAngleSensor_t *steering_angle)
{
	// TODO
	return true;
}

