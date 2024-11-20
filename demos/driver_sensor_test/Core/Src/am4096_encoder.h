/*
 * am4096_encoder.h
 *
 * Driver functions for the AM4096 magnetic encoder.
 * The steering sensor PCB has I2C and Data lines pinned out.
 * TODO: driver is written for I2C blocking mode, should be made
 * non-block with DMA when integrating with RTOS
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
	// peripheral handle
	I2C_HandleTypeDef *i2c_handle;

	// Angle in radians * 1000, relative or absolute depending on settings
	uint16_t angle;

	// Angular velocity in (radians * 1000) / s
	uint16_t angular_velocity

} AM4096_t;
// Public Function Prototypes

/*
 *  Initialization, device ID check. Returns error status
 */
uint8_t am4096_init(AM4096_t *sensor, I2C_HandleTypeDef *i2c_handle);

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
 * Get Sensor Status
 */
uint8_t am4096_get_status(AM4096_t *sensor);


#endif /* SRC_AM4096_ENCODER_H_ */
