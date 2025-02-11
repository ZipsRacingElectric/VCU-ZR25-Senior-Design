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
typedef struct
{
	I2C_HandleTypeDef *i2c_handle; // peripheral handle
	/*
	 * Status Flag for device:
	 * SRCH bit, Weh bit, Wel bit, Thof bit
	 */
	uint16_t device_status;

	/*
	 * Register configuration for the device
	 */
	uint16_t config;
	uint16_t angle; // Angle in radians * 1000, relative or absolute depending on settings
	uint16_t angular_velocity;	// Angular velocity in (radians * 1000) / s

} AM4096_t;
// Public Function Prototypes

/*
 *  Initialization, device ID check. Returns error status
 */
HAL_StatusTypeDef am4096_init(AM4096_t *sensor, I2C_HandleTypeDef *i2c_handle);

/*
 * Zero Sensor Angle
 */
HAL_StatusTypeDef am4096_zero_angle(AM4096_t *sensor);

/*
 * Read angle
 */
HAL_StatusTypeDef am4096_read_angle(AM4096_t *sensor);

/*
 * Read angular velocity
 */
HAL_StatusTypeDef am4096_read_angular_velocity(AM4096_t *sensor);

/*
 * reads the magnet status of the sensor
 */
HAL_StatusTypeDef am4096_magnet_status(AM4096_t *sensor);


#endif /* SRC_AM4096_ENCODER_H_ */
