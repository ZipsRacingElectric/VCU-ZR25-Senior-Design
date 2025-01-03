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
 * - Steering angle deadzones
 * - filtering
 * - unit testing
 */

#ifndef SRC_DRIVER_SENSORS_H_
#define SRC_DRIVER_SENSORS_H_

// Includes
#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

// Types
typedef struct
{
    uint16_t raw_value_1; // Raw ADC value, channel 1
    uint16_t raw_value_2; // Raw ADC value, channel 2
    uint16_t voltage_1;   // Voltage of channel 1 in V * 1000
    uint16_t voltage_2;   // Voltage of channel 2 in V * 1000
    uint16_t percent_1;   // Position in percentage of channel 1
    uint16_t percent_2;   // Position in percentage of channel 2
    uint16_t percent;    // Position in percentage used for
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
    uint16_t raw_value; // Raw value from sensor
    uint16_t angle;     // Steering angle in radians * 1000
    bool plausible;		// Plausibility of sensor
} SteeringAngleSensor_t;

// Function Prototypes

/*
 * Reads in the raw sensor data and updates the sensor variables
 */
void read_driver_input(ADC_HandleTypeDef *adc);


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
