/*
 * can_sensors.h
 *
 *  Created on: Apr 8, 2025
 *      Author: bre17
 */

#ifndef INC_CAN_SENSORS_H_
#define INC_CAN_SENSORS_H_
typedef union {
	uint32_t flagInt;
	struct {
		uint8_t updateGps : 1;
	} flagBits;
} CanSensorsFlags_t;

typedef enum {
	GPS_UNINITIALIZED = 0,
	GPS_NOFIX = 1,
	GPS_2D = 3,
	GPS_3D = 4,
} GpsStatus_t;

typedef struct {
	int32_t latitude; // Measured in 10^-7 degrees
	int32_t longitude; // Measured in 10^-7 degrees
	int16_t speed; // Measured in (10^-3 / 36) km/h
	int16_t height; // Measured in meters
	uint8_t num_satellites;
	uint8_t gps_status;
	uint16_t heading_motion; // Measured in degrees
	uint16_t heading_vehicle; // Measured in degrees
	int16_t x_angle_rate; // Measured in 10^-2 degrees/s
	int16_t y_angle_rate; // Measured in 10^-2 degrees/s
	int16_t z_angle_rate; // Measured in 10^-2 degrees/s
	int16_t x_acceleration; // Measured in 10^-2 Gs
	int16_t y_acceleration; // Measured in 10^-2 Gs
	int16_t z_acceleration; // Measured in 10^-2 Gs

	uint8_t utc_year; // Years after 2000
	uint8_t utc_month;
	uint8_t utc_day;
	uint8_t utc_hour;
	uint8_t utc_minute;
	uint8_t utc_second;
	uint16_t utc_millisecond; // Measured in (1000/65536) milliseconds
} GPSState_t;

void StartCanSensorsTask(void *argument);

#endif /* INC_CAN_SENSORS_H_ */
