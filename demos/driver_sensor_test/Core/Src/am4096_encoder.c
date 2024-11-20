/*
 * am4096_encoder.c
 *
 * Driver functions for the AM4096 magnetic encoder.
 * The steering sensor PCB has TWI(I2C) and Serial SSI pinned out.
 * TODO: driver is written for I2C blocking mode, should be made
 * non-block with DMA when integrating with RTOS
 *
 *  Created on: Nov 19, 2024
 *      Author: bglen
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
uint8_t am4096_init(AM4096_t *sensor, I2C_HandleTypeDef *i2c_handle)
{

}

/*
 * Zero Sensor Angle
 */
HAL_StatusTypeDef am4096_zero_angle(AM4096_t *sensor)
{

}

/*
 * Read angle
 */
HAL_StatusTypeDef am4096_read_angle(AM4096_t *sensor)
{

}

/*
 * Read angular velocity
 */
HAL_StatusTypeDef am4096_read_angular_velocity(AM4096_t *sensor)
{

}

/*
 * Get Sensor Status
 */
uint8_t am4096_get_status(AM4096_t *sensor)
{

}

// Private Functions

/*
 * Read an individual register
 */
HAL_StatusTypeDef read_register(AM4096_t *sensor, uint8_t reg_address, uint16_t *data_location)
{
	HAL_StatusTypeDef status;
	uint8_t buffer[12];
	buffer[0] = reg_address;

	// Send out device address and r/w bit

	// wait for acknowledge

	// recieve data

	// acknowledge recieve & stop
}

/*
 * Write to an individual register, sending 1 byte
 */
HAL_StatusTypeDef write_register(AM4096_t *sensor, uint8_t reg_address, uint16_t *write_data)
{
	if (write_data == NULL)
	{
		// TODO: handle null pointer
	}

	uint8_t buffer[12];

	buffer[0] = reg_address; // Register address byte
	buffer[1] = (*write_data >> 8); // 8 MSB of data
	buffer[2] = (*write_data & 0xFF); // 8 LSB of data

	// Send out device address and write bit
	return HAL_I2C_Master_Transmit(sensor->i2c_handle, AM4096_ADDRESS, buffer, 3, HAL_MAX_DELAY);

	// Note: device cannot be addressed for 20 ms after write command
}
