/*
 * driver_input.h
 *
 * This handles all functions relating to measuring driver input sensors.
 *
 *  Created on: Nov 13, 2024
 *      Author: bglen
 */

/*
 * TODO:
 * - error handling
 * - filtering
 * - calibration
 * - interrupts / dma
 * - unit testing??
 */

#ifndef SRC_DRIVER_INPUT_H_
#define SRC_DRIVER_INPUT_H_

// Includes
#include <stdint.h>
#include <stdbool.h>

// Types
typedef struct {
    uint16_t raw_value_1; // Raw ADC value, channel 1
    uint16_t raw_value_2; // Raw ADC value, channel 2
    uint16_t voltage_1;   // Voltage of channel 1 in V * 1000
    uint16_t voltage_2;   // Voltage of channel 2 in V * 1000
    uint16_t percent_1;   // Position in percentage of channel 1
    uint16_t percent_2;   // Position in percentage of channel 2
    uint16_t position;    // Position in percentage used for
    bool plausible;		  // Plausibility of sensor
} APPSSensor_t;

// TODO: determine how pressure units should be represented
typedef struct {
    uint16_t raw_value; // Raw ADC value
    uint16_t voltage;   // Voltage of channel 1 in V * 1000
    uint16_t pressure;  // Pressure in PSI
    bool plausible;		// Plausibility of sensor
} BPSSensor_t;

typedef struct {
    uint16_t raw_value; // Raw value from sensor
    uint16_t angle;     // Steering angle in radians * 1000
    bool plausible;		// Plausibility of sensor
} SteeringAngleSensor_t;

// Function Prototypes

/*
 * Initializes the peripherals used for measuring the driver input sensors
 */
void driver_input_init(void);

/*
 * Reads in the raw sensor data and updates the sensor variables
 */
void read_driver_input(APPSSensor_t *apps, BPSSensor_t *bps_front, BPSSensor_t *bps_rear, SteeringAngleSensor_t *steering_angle);


/*
 * Calibrates the constants used to determine APPS position
 */
bool calibrate_apps(APPSSensor_t *apps);

/*
 * Calibrates the constants used to determine BPS pressure
 */
bool calibrate_bps(BPSSensor_t *bps);

/*
 * Calibrates the constants used to determine steering angles
 */
bool calibrate_steering(SteeringAngleSensor_t *steering_angle);

// Callback Prototypes

#endif /* SRC_DRIVER_INPUT_H_ */