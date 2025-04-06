/*
 * tuning_params.h
 *
 *  Created on: Apr 6, 2025
 *      Author: tetra
 */

#ifndef INC_TUNING_PARAMS_H_
#define INC_TUNING_PARAMS_H_

#include <stdint.h>

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
#define PID_LENGTH_VREF_AXIS 0
#define PID_LENGTH_THETA_AXIS 0

// Used to convert from raw measurements to LUT indexes.
typedef struct {
	float min_point; // Value corresponding to lowest index
	float max_point; // Value corresponding to highest index
	int num_points; // Length of (axis of) lookup table
	// Should be equal to (max_point-min_point)/(num_points-1).
	// i.e. min_point + (num_points-1)*point_spacing == max_point
	// Pre-computed to save CPU cycles.
	// Also useful for gradient calculation.
	float point_spacing;
} breakpoints_t;

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
	float p[PID_LENGTH_VREF_AXIS][PID_LENGTH_THETA_AXIS];
	float i[PID_LENGTH_VREF_AXIS][PID_LENGTH_THETA_AXIS];
	float d[PID_LENGTH_VREF_AXIS][PID_LENGTH_THETA_AXIS];
} torqueControlPIDParams_t;

typedef struct {
	torqueControlTorqueDistParams_t td_params;
	volatile torqueControlPIDParams_t pid_params;
} torqueControlParameters_t;

extern const torqueControlParameters_t torque_control_params;

// Parameter->index conversion functions
// Given the real value of a parameter, find its index for use in LUTs
// Return -1 if out of bounds
// If `interp` non-NULL, returns an interpolation factor from 0-1
int sl_index(float sl_value, float* interp);
int sa_index(float sa_value_degrees, float* interp);
int fz_index(float fz_value_newtons, float* interp);
int motor_speed_index(float speed_value_rpm, float* interp);
int motor_current_index(float current_value_amps, float* interp);
int motor_temp_index(float temp_value_celcius, float* interp);

// Linear interpolation.
static inline float lerp(float a, float b, float interp) {
	return a*(1-interp) + b*interp;
}

// Linear interpolation partial derivative w.r.t. interp.
static inline float lerp_ddx(float a, float b, float interp) {
	return b-a;
}

// Bilinear interpolation.
// interp_x is the interpolation value along the first axis, interp_y the second axis.
static inline float bilerp(float points[2][2], float interp_x, float interp_y) {
	float c0 = lerp(points[0][0], points[0][1], interp_y);
	float c1 = lerp(points[1][0], points[1][1], interp_y);
	return lerp(c0, c1, interp_x);
}

// Bilinear interpolation partial derivative w.r.t. interp_x.
static inline float bilerp_ddx(float points[2][2], float interp_x, float interp_y) {
	return lerp(points[1][0],points[1][1], interp_y) - lerp(points[0][0], points[0][1], interp_y);
}
// Bilinear interpolation partial derivative w.r.t. interp_y.
static inline float bilerp_ddy(float points[2][2], float interp_x, float interp_y) {
	return lerp(points[0][1],points[1][1], interp_x) - lerp(points[0][0], points[1][0], interp_x);
}

// Trilinear interpolation.
static inline float trilerp(float points[2][2][2], float interp_x, float interp_y, float interp_z) {
	return lerp(
		bilerp(points[0], interp_y, interp_z),
		bilerp(points[1], interp_y, interp_z),
		interp_x
	);
}

static inline float trilerp_ddx(float points[2][2][2], float interp_x, float interp_y, float interp_z) {
	return bilerp(points[1],interp_y,interp_z) - bilerp(points[0],interp_y,interp_z);
}
static inline float trilerp_ddy(float points[2][2][2], float interp_x, float interp_y, float interp_z) {
	float c1 = lerp(
			lerp(points[0][1][0], points[0][1][1], interp_z),
			lerp(points[1][1][0], points[1][1][1], interp_z),
			interp_x);
	float c0 = lerp(
			lerp(points[0][0][0], points[0][0][1], interp_z),
			lerp(points[1][0][0], points[1][0][1], interp_z),
			interp_x);
	return c1-c0;
}
static inline float trilerp_ddz(float points[2][2][2], float interp_x, float interp_y, float interp_z) {
	float c1 = lerp(
			lerp(points[0][0][1], points[0][1][1], interp_y),
			lerp(points[1][0][1], points[1][1][1], interp_y),
			interp_x);
	float c0 = lerp(
			lerp(points[0][0][0], points[0][1][0], interp_y),
			lerp(points[1][0][0], points[1][1][0], interp_y),
			interp_x);
	return c1-c0;
}

// Non-interpolating lookup functions
// May have discontinuous jumps between points.
float lookup_fx_nointerp(float slip_ratio, float slip_angle_degrees, float fz_newtons);
float lookup_fy_nointerp(float slip_ratio, float slip_angle_degrees, float fz_newtons);
float lookup_motor_efficiency_nointerp(float motor_speed_rpm, float motor_current_amps, float motor_temp_celcius);

// Interpolating lookup functions
// Performs n-linear interpolation between LUT points.
// Also returns partial derivatives if passed non-NULL pointer arguments.
float lookup_fx(float slip_ratio, float slip_angle_degrees, float fz_newtons, float* ddsl, float* ddsa, float* ddfz);
float lookup_fy(float slip_ratio, float slip_angle_degrees, float fz_newtons, float* ddsl, float* ddsa, float* ddfz);
float lookup_motor_efficiency(float motor_speed_rpm, float motor_current_amps, float motor_temp_celcius, float* ddms, float* ddmc, float* ddmt);

// Programs new PID parameters to the flash memory.
// These new values will persist across reboots, but will be overwritten when
// the VCU is reprogrammed. Permanent changes should be committed in
// tuning_params.c.
void program_pid_params(torqueControlPIDParams_t *);

#endif /* INC_TUNING_PARAMS_H_ */
