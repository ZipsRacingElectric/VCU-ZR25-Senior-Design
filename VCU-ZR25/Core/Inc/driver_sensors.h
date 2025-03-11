/*
 * driver_sensors.h
 *
 * This handles all functions relating to measuring driver input sensors.
 *
 *  Created on: Nov 13, 2024
 *      Author: bglen
 */

/*
 * TODO:
 * - BPS calibration
 * - Steering angle calibration
 * - brake light set point
 * - hard braking condition
 * - Steering angle dead zone
 * - filtering
 * - unit testing
 */

#ifndef SRC_DRIVER_SENSORS_H_
#define SRC_DRIVER_SENSORS_H_

// Includes
#ifdef TEST
#include <stm32f4xx_hal.h>
#endif

#include "am4096_encoder.h"
#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmsis_os.h>

// Constants for Conversion and Validation
#define APPS_1_CHANNEL	 	  ADC_CHANNEL_0
#define APPS_2_CHANNEL	      ADC_CHANNEL_1
#define BPS_FRONT_CHANNEL	  ADC_CHANNEL_2
#define BPS_REAR_CHANNEL	  ADC_CHANNEL_3

#define ADC_MAX_VALUE         4095  // 2^12 - 1
#define ADC_REF_VOLTAGE       3300  // V * 1000

#define APPS_1_MIN_VOLTAGE    0500  // V * 1000
#define APPS_2_MIN_VOLTAGE    0250  // V * 1000
#define APPS_1_MAX_VOLTAGE    4500  // V * 1000
#define APPS_2_MAX_VOLTAGE    2250  // V * 1000
#define APPS_MIN_DELTA_V	  0030  // Minimum allowable voltage difference V * 1000
#define APPS_MAX_DELTA_P   	  0100  // Maximum allowable pedal position difference % * 10
#define APPS_DEADZONE		  0050  // Upper and lower pedal deadzone in % * 10
#define APPS_OUT_OF_RANGE     0100  // Allowable voltage beyond calibration limits before fault V * 1000

#define BPS_MIN_VOLTAGE       0500  // V * 1000
#define BPS_MAX_VOLTAGE       4500  // V * 1000
#define BPS_ENGAGED_VOLTAGE_THRESHOLD 2500  // V * 1000 Halfway between min and max
#define BPS_MAX_PRESSURE      2500 // pressure in PSI
#define BPS_DEADZONE    	  0050  // Deadzone before plausibility fault in V * 1000

#define STEERING_MAX_ANGLE    3200  // radians * 1000
#define STEERING_MIN_ANGLE    -3200 // radians * 1000

typedef struct
{
    ADC_HandleTypeDef hadc1;
    ADC_ChannelConfTypeDef sConfig;
} DriverSensorTaskArgs_t;

void StartDriverSensorTask(
    DriverSensorTaskArgs_t *args
);

// Types
typedef struct
{
    uint16_t raw_value_1; // Raw ADC value, channel 1
    uint16_t raw_value_2; // Raw ADC value, channel 2
    uint16_t voltage_1;   // Voltage of channel 1 in V * 1000
    uint16_t voltage_2;   // Voltage of channel 2 in V * 1000
    uint16_t percent_1;   // Position in percentage of channel 1
    uint16_t percent_2;   // Position in percentage of channel 2
    uint16_t percent;     // Position in percentage used for
    bool plausible;		  // Plausibility of sensor
} APPSSensor_t;

typedef struct
{
    uint16_t raw_value; // Raw ADC value
    uint16_t voltage;   // Voltage of channel 1 in V * 1000
    uint16_t pressure;  // Pressure in kPa
    bool plausible;		// Plausibility of sensor
} BPSSensor_t;

typedef struct
{
	AM4096_t i2c_device;       // AM4096 device
    uint16_t angle;            // Steering angle in radians * 1000
    uint16_t angular_velocity; // Angular velocity in rad * 1000 / s
    bool plausible;		       // Plausibility of sensor
} SteeringAngleSensor_t;

// Function Prototypes

/*
 * Initializes the driver input sensors
 */
void init_driver_input(I2C_HandleTypeDef *i2c);

/*
 * Reads in the raw sensor data and updates the sensor variables
 */
void read_driver_input(ADC_HandleTypeDef *adc);

void print_driver_input(void);

void fsm_sensor_callback(void);
/*
 * Returns an APPSSensor_t struct with the current APPS data in it
 */
APPSSensor_t get_apps_data(void);

/*
 *Returns an BPSSensor_t struct with the current front BPS data in it
 */
BPSSensor_t get_bps_front_data(void);

/*
 * Returns an BPSSensor_t struct with the current rear BPS data in it
 */
BPSSensor_t get_bps_rear_data(void);

/*
 * Returns an SteeringAngleSensor_t struct with the current steering data in it
 */
SteeringAngleSensor_t get_steering_angle_data(void);

/*
 * Calibrates the constants used to determine APPS position.  Returns false if calibration failed.
 */
bool calibrate_apps(uint16_t apps_1_pedal_min, uint16_t apps_1_pedal_max, uint16_t apps_2_pedal_min, uint16_t apps_2_pedal_max);

/*
 * Calibrates the constants used to determine BPS pressure.  Returns false if calibration failed.
 */
bool calibrate_bps(void);

/*
 * Calibrates the constants used to determine steering angles.  Returns false if calibration failed.
 */
bool calibrate_steering(void);

// Callback Prototypes

#endif /* SRC_DRIVER_SENSORS_H_ */
