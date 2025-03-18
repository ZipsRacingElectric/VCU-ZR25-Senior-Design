/*
 * am4096_encoder.c
 *
 * Driver functions for the AM4096 magnetic encoder.
 * The steering sensor PCB has TWI(I2C) and Serial SSI pinned out.
 *
 *  Created on: Nov 19, 2024
 *      Author: bglen
 */

/*
 * TODO:
 * - am4096_magnet_status: insert relevant bits into status flag
 */

// Includes
#include "am4096_encoder.h"
#include "stm32f4xx_hal.h"

// Defines
#define AM4096_ADDRESS (0x00 << 1) // Left shift to ensure read/write bit is free

// Registers
#define REG_EEPROM_0       0
#define REG_EEPROM_1       1
#define REG_EEPROM_2       2
#define REG_EEPROM_3       3
#define REG_R_POS         32
#define REG_A_POS         33
#define REG_MAGNET_STATUS 34
#define REG_TACHO         35
#define REG_SETTINGS_1    48
#define REG_SETTINGS_2    49
#define REG_SETTINGS_3    50
#define REG_SETTINGS_4    51

// Configuration bits
#define STH 000 // Angular velocity 2048 Hz, 122,880 RPM

// Constants

// Static Variables

// Private Function Prototypes

/*
 * Read an individual register
 */
HAL_StatusTypeDef read_register(AM4096_t *sensor, uint8_t reg_addresss, uint16_t *data_location);

/*
 * Write to an individual register
 */
HAL_StatusTypeDef write_register(AM4096_t *sensor, uint8_t reg_address, uint16_t *write_data);

// Public Functions

/*
 *  Initialization, device ID check. Returns error status
 */
HAL_StatusTypeDef am4096_init(AM4096_t *sensor, I2C_HandleTypeDef *i2c_handle)
{
	if (!sensor || !i2c_handle) return HAL_ERROR;

	    sensor->i2c_handle = i2c_handle;

	    // Check device ID (optional: depends on AM4096 functionality)
	    uint16_t device_id;
	    if (read_register(sensor, 0x00, &device_id) != HAL_OK)
	    {
	        return HAL_ERROR;
	    }

	    // Set register settings


	    return HAL_OK;
}

/*
 * Zero Sensor Angle at the current position
 */
HAL_StatusTypeDef am4096_zero_angle(AM4096_t *sensor)
{
	uint16_t zero_data = 0x00; // Value to set zeroing
	return write_register(sensor, REG_SETTINGS_1, &zero_data);
}

/*
 * Read angle
 */
HAL_StatusTypeDef am4096_read_angle(AM4096_t *sensor)
{
	uint16_t raw_angle;
	if (read_register(sensor, REG_R_POS, &raw_angle) != HAL_OK)
	{
		sensor->device_status = 3;
		return HAL_ERROR;
	}

	// Check

	// Convert raw angle to radians (assuming a full range is 0xFFF = 2π radians)
	sensor->angle = (uint16_t)((uint32_t)raw_angle * 6283) / 4096; // Scaled to radians * 1000
	return HAL_OK;
}

/*
 * Read angular velocity
 */
HAL_StatusTypeDef am4096_read_angular_velocity(AM4096_t *sensor)
{
	uint16_t tho;
	if (read_register(sensor, REG_TACHO, &tho) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Grab speed data from the register values
	tho = tho << 5;

	// Check if speed data is overflowing


	// Convert to usable value

	sensor->angular_velocity = tho;


	return HAL_OK;
}

/*
 * Gets the magnet status of the sensor
 */
HAL_StatusTypeDef am4096_magnet_status(AM4096_t *sensor)
{
	uint16_t magnet_status;
	if (read_register(sensor, REG_MAGNET_STATUS, &magnet_status) != HAL_OK)
	{
		return HAL_ERROR;
	}

	sensor->device_status = 0;
	return HAL_OK;
}

// Private Functions

/*
 * Read an individual register
 */
HAL_StatusTypeDef read_register(AM4096_t *sensor, uint8_t reg_address, uint16_t *data_location)
{
	// Handle null pointers for sensor or write_data
	if (!sensor || !data_location)
	{
		return HAL_ERROR;
	}

	uint8_t command = reg_address;
	uint8_t data[2] = {0};

	// Send write command with just register address
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(sensor->i2c_handle, AM4096_ADDRESS, &command, 1, HAL_MAX_DELAY);

	if (status != HAL_OK)
	{
		return status;
	}

	// Send read command and receive back data
	status = HAL_I2C_Master_Receive(sensor->i2c_handle, AM4096_ADDRESS, data, 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		*data_location = (data[0] << 8) | data[1];
	}

	return status;
}

/*
 * Write to an individual register, sending 1 byte
 */
HAL_StatusTypeDef write_register(AM4096_t *sensor, uint8_t reg_address, uint16_t *write_data)
{
	// Handle null pointers for sensor or write_data
	if (!sensor || !write_data)
	{
		return HAL_ERROR;
	}

	// create data buffer
	uint8_t data[3];
	data[0] = reg_address; // register address
	data[1] = (*write_data >> 8) & 0xFF; // MSB of write data
	data[2] = (*write_data & 0xFF); // LSB of write data


	// Send out device address and write bit
	return HAL_I2C_Master_Transmit(sensor->i2c_handle, AM4096_ADDRESS, data, 3, HAL_MAX_DELAY);

	// Note: device cannot be addressed for 20 ms after write command
}
