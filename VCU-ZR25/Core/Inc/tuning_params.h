/*
 * tuning_params.h
 *
 *  Created on: Apr 6, 2025
 *      Author: tetra
 */

#ifndef INC_TUNING_PARAMS_H_
#define INC_TUNING_PARAMS_H_

#include <stdint.h>
#include "lut_utils.h"

// LUT Dimensions
#define SL_AXIS_LENGTH 31
#define SA_AXIS_LENGTH 53
#define FZ_AXIS_LENGTH 33
// TODO: do not know dimensions/contents of motor efficiency LUT
#define MOTOR_SPEED_AXIS_LENGTH 0
#define MOTOR_CURRENT_AXIS_LENGTH 0
#define MOTOR_TEMP_AXIS_LENGTH 0
// TODO: i don't know what names are being used for reference velocity and steering angle - feel free to rename
// TODO: i also don't know the dimensions or contents of these tables.
// TODO: need vref and steering angle breakpoints if we're doing those
#define PID_LENGTH_VREF_AXIS 26
#define PID_LENGTH_SW_ANGLE_AXIS 10

typedef struct {
	breakpoints_t sl_breakpoints; // Slip angle (degrees)
	breakpoints_t sa_breakpoints; // Slip ratio
	breakpoints_t fz_breakpoints; // Normal force (newtons)
	breakpoints_t motor_speed_breakpoints; // Motor speed (rpm)
	breakpoints_t motor_current_breakpoints; // Motor current (amps)
	breakpoints_t motor_temp_breakpoints; // Motor temperature (celcius)
	int16_t fx[SL_AXIS_LENGTH][SA_AXIS_LENGTH][FZ_AXIS_LENGTH];
	int16_t fy[SL_AXIS_LENGTH][SA_AXIS_LENGTH][FZ_AXIS_LENGTH];
	int16_t motor_efficiency[MOTOR_SPEED_AXIS_LENGTH][MOTOR_CURRENT_AXIS_LENGTH][MOTOR_TEMP_AXIS_LENGTH];
} torqueControlTorqueDistParams_t;

typedef struct {
	breakpoints_t vref_breakpoints; // Reference speed (m/s)
	breakpoints_t sw_angle_breakpoints; // Absolute value of steering wheel angle (degrees)
	float p[PID_LENGTH_VREF_AXIS][PID_LENGTH_SW_ANGLE_AXIS];
	float i[PID_LENGTH_VREF_AXIS][PID_LENGTH_SW_ANGLE_AXIS];
} torqueControlPIDParams_t;

typedef struct {
	torqueControlTorqueDistParams_t td_params;
	torqueControlPIDParams_t pid_params;
} torqueControlParameters_t;

extern const torqueControlParameters_t torque_control_params;

// Parameter->index conversion functions
// Given the real value of a parameter, find its index for use in LUTs
// Return -1 if out of bounds
// If `interp` non-NULL, returns an interpolation factor from 0-1
int get_index(const breakpoints_t *breakpoints, float value, float* interp);
int sl_index(float sl_value, float* interp);
int sa_index(float sa_value_degrees, float* interp);
int fz_index(float fz_value_newtons, float* interp);
int motor_speed_index(float speed_value_rpm, float* interp);
int motor_current_index(float current_value_amps, float* interp);
int motor_temp_index(float temp_value_celcius, float* interp);
int vref_index(float vref_value_meters_per_second, float* interp);
int sw_angle_index(float sw_value_degrees, float* interp);


// Non-interpolating lookup functions
// May have discontinuous jumps between points.
float lookup_fx_nointerp(float slip_ratio, float slip_angle_degrees, float fz_newtons);
float lookup_fy_nointerp(float slip_ratio, float slip_angle_degrees, float fz_newtons);
float lookup_motor_efficiency_nointerp(float motor_speed_rpm, float motor_current_amps, float motor_temp_celcius);
float lookup_p_gain_nointerp(float vref_meters_per_second, float steering_wheel_angle);
float lookup_i_gain_nointerp(float vref_meters_per_second, float steering_wheel_angle);

// Interpolating lookup functions
// Performs n-linear interpolation between LUT points.
// Also returns partial derivatives if passed non-NULL pointer arguments.
float lookup_fx(float slip_ratio, float slip_angle_degrees, float fz_newtons, float* ddsl, float* ddsa, float* ddfz);
float lookup_fy(float slip_ratio, float slip_angle_degrees, float fz_newtons, float* ddsl, float* ddsa, float* ddfz);
float lookup_motor_efficiency(float motor_speed_rpm, float motor_current_amps, float motor_temp_celcius, float* ddms, float* ddmc, float* ddmt);
// No gradients for PI gains.
float lookup_p_gain(float vref_meters_per_second, float steering_wheel_angle);
float lookup_i_gain(float vref_meters_per_second, float steering_wheel_angle);

// Programs new PID parameters to the flash memory.
// These new values will persist across reboots, but will be overwritten when
// the VCU is reprogrammed. Permanent changes should be committed in
// tuning_params.c.
void program_pid_params(torqueControlPIDParams_t *);

#endif /* INC_TUNING_PARAMS_H_ */
