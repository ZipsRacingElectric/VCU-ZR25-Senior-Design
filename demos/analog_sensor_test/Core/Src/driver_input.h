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
void read_driver_input(void);


/*
 * Returns APPS pedal position in % * 10
 */
uint16_t get_apps_position(void);

/*
 * Returns APPS voltages in V * 1000. Used for calibration.
 */
uint16_t get_apps_voltage(uint16_t channel);

/*
 * Returns front brake pressure in kPa
 */
uint16_t get_front_brake_pressure(void);

/*
 * Returns rear brake pressure in kPa
 */
uint16_t get_rear_brake_pressure(void);

/*
 * Returns steering angle in radians * 1000
 */
uint16_t get_steering_angle(void);

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

#endif /* SRC_DRIVER_INPUT_H_ */
