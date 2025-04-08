/*
 * am4096_encoder.h
 *
 * Driver functions for the AM4096 magnetic encoder.
 * The steering sensor PCB has I2C and Data lines pinned out.
 *
 *  Created on: Nov 19, 2024
 *      Author: bglen
 */

#ifndef SRC_AM4096_ENCODER_H_
#define SRC_AM4096_ENCODER_H_

// Includes
#include "stm32f4xx_hal.h"
#include "stdbool.h"

// Types
typedef enum
{
/// @brief Indicates a hardware failure has occurred, the device is not responding.
AM4096_STATE_FAILED = 0,
/// @brief Indicates the device is functioning, but the measurement is invalid.
AM4096_STATE_INVALID = 1,
/// @brief Indicates the device is functioning and the measurement is valid.
AM4096_STATE_READY = 3
} am4096State_t;

typedef struct
{
I2C_HandleTypeDef *i2c_handle;
} am4096Config_t;

typedef struct
{
am4096Config_t* config;
am4096State_t state;
uint16_t sample;
uint16_t angle;
uint16_t angular_velocity;
} am4096_t;

// Public Function Prototypes

/*
 *  Initialization, device ID check. Returns error status
 */
bool am4096Init (am4096_t* am4096, am4096Config_t* config);

/*
 * Sample the am4096
 */
bool am4096Sample (am4096_t* am4096);

/*
 * Zero Sensor Angle
 */
HAL_StatusTypeDef am4096_zero_angle(am4096_t *sensor);

/*
 * Read angle
 */
HAL_StatusTypeDef am4096_read_angle(am4096_t *sensor);

/*
 * Read angular velocity
 */
HAL_StatusTypeDef am4096_read_angular_velocity(am4096_t *sensor);

/*
 * reads the magnet status of the sensor
 */
HAL_StatusTypeDef am4096_magnet_status(am4096_t *sensor);


#endif /* SRC_AM4096_ENCODER_H_ */
