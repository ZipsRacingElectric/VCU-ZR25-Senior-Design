/*
 * can_sensors.c
 *
 *  Created on: Apr 8, 2025
 *      Author: bre17
 */

#include "can_db.h"
#include "can_messages.h"
#include "can_sensors.h"
#include "vehicle_data.h"

GPSState_t gps_state;
osThreadId_t thread_id;

void gps_position_callback(uint32_t can_id, uint64_t messageContents, void* custom) {
	CANMessage_ECUMASTER_GPS_POSITION msg;
	msg.as_u64 = messageContents;
	CanSensorsFlags_t flags = {.flagBits = {.updateGps=1}};

	gps_state.latitude = msg.fields.LATITUDE;
	gps_state.longitude = msg.fields.LONGITUDE;

	osThreadFlagsSet(thread_id, flags.flagInt);
}
void gps_velocity_callback(uint32_t can_id, uint64_t messageContents, void* custom) {
	CANMessage_ECUMASTER_GPS_VELOCITY msg;
	msg.as_u64 = messageContents;
	CanSensorsFlags_t flags = {.flagBits = {.updateGps=1}};

	gps_state.speed = msg.fields.SPEED;
	gps_state.height = msg.fields.HEIGHT;
	gps_state.num_satellites = msg.fields.SATELLITES_NUMBER;
	gps_state.gps_status = msg.fields.GPS_STATUS;

	osThreadFlagsSet(thread_id, flags.flagInt);
}
void gps_imu_0_callback(uint32_t can_id, uint64_t messageContents, void* custom) {
	CANMessage_ECUMASTER_GPS_HEADING_IMU0 msg;
	msg.as_u64 = messageContents;
	CanSensorsFlags_t flags = {.flagBits = {.updateGps=1}};

	gps_state.heading_motion = msg.fields.HEADING_MOTION;
	gps_state.heading_vehicle = msg.fields.HEADING_VEHICLE;
	gps_state.x_angle_rate = msg.fields.X_ANGLE_RATE;
	gps_state.y_angle_rate = msg.fields.Y_ANGLE_RATE;

	osThreadFlagsSet(thread_id, flags.flagInt);
}
void gps_imu_1_callback(uint32_t can_id, uint64_t messageContents, void* custom) {
	CANMessage_ECUMASTER_GPS_IMU1 msg;
	msg.as_u64 = messageContents;
	CanSensorsFlags_t flags = {.flagBits = {.updateGps=1}};

	gps_state.z_angle_rate = msg.fields.Z_ANGLE_RATE;
	gps_state.x_acceleration = msg.fields.X_ACCELERATION;
	gps_state.y_acceleration = msg.fields.Y_ACCELERATION;
	gps_state.z_acceleration = msg.fields.Z_ACCELERATION;

	osThreadFlagsSet(thread_id, flags.flagInt);
}
void gps_utc_callback(uint32_t can_id, uint64_t messageContents, void* custom) {
	CANMessage_ECUMASTER_GPS_UTC msg;
	msg.as_u64 = messageContents;
	CanSensorsFlags_t flags = {.flagBits = {.updateGps=1}};

	gps_state.utc_year = msg.fields.UTC_YEAR;
	gps_state.utc_month = msg.fields.UTC_MONTH;
	gps_state.utc_day = msg.fields.UTC_DAY;
	gps_state.utc_hour = msg.fields.UTC_HOUR;
	gps_state.utc_minute = msg.fields.UTC_MINUTE;
	gps_state.utc_second = msg.fields.UTC_SECOND;
	gps_state.utc_millisecond = msg.fields.UTC_MILLISECOND;

	osThreadFlagsSet(thread_id, flags.flagInt);
}

void StartCanSensorsTask(void *argument) {
	gps_state = (GPSState_t){0};
	thread_id = osThreadGetId();
	CANRegisterCallback(CANGetDbEntry(CAN_DB_ECUMASTER_GPS_POSITION_ID), gps_position_callback, NULL);
	CANRegisterCallback(CANGetDbEntry(CAN_DB_ECUMASTER_GPS_VELOCITY_ID), gps_velocity_callback, NULL);
	CANRegisterCallback(CANGetDbEntry(CAN_DB_ECUMASTER_GPS_HEADING_IMU0_ID), gps_imu_0_callback, NULL);
	CANRegisterCallback(CANGetDbEntry(CAN_DB_ECUMASTER_GPS_IMU1_ID), gps_imu_1_callback, NULL);
	CANRegisterCallback(CANGetDbEntry(CAN_DB_ECUMASTER_GPS_UTC_ID), gps_utc_callback, NULL);

	const CanSensorsFlags_t ALL_FLAGS = {.flagBits = {.updateGps=1}};
	CanSensorsFlags_t flags;
	while (1) {
		flags.flagInt = osThreadFlagsWait(ALL_FLAGS.flagInt, osFlagsWaitAny, 10);
		if (flags.flagBits.updateGps) {
			osMutexAcquire(vdb_gps_lockHandle, osWaitForever);
			VehicleData.gps = gps_state;
			osMutexRelease(vdb_gps_lockHandle);
		}
	}
}
