/*
 * lut.h
 * Simplified version of tuning_params.h with just LUT functions
 *
 *  Created on: Apr 6, 2025
 *      Author: tetra, bglen
 */

#ifndef INC_LUT_H_
#define INC_LUT_H_

#include <stdint.h>

// Types

// Used to convert from raw measurements to LUT indexes.
typedef struct {
	float min_point; // Value corresponding to lowest index
	float max_point; // Value corresponding to highest index
	int num_points; // Length of (axis of) lookup table
	float point_spacing; // must be = (max_point - min_point) / (num_points - 1)
} breakpoints_t;

// Public Function Prototypes

/*
General 1-D lookup table function, no interpolation
	const breakpoints_t* bp_x - Structure containing breakpoint information
	int length_x - Length of lookup table
	const int16_t lut[length_x] - Lookup table data
	float x - Value of breakpoint to lookup data at
*/
float __attribute__((unused)) lookup_1d_nointerp(
	const breakpoints_t* bp_x,
	int length_x,
	const int16_t lut[length_x],
	float x
);

/*
 General 1-D lookup table function with interpolation and optional gradient
	const breakpoints_t* bp_x - Structure containing breakpoint data
	int length_x - Length of lookup table
	const int16_t lut[length_x] - Lookup table data
	float x - Value of breakpoint to lookup data at
	float* ddx - (optional) Pass a non-NULL pointer which gets populated with the gradient value
 */
float __attribute__((unused)) lookup_1d(
	const breakpoints_t* bp_x,
	int length_x,
	const int16_t lut[length_x],
	float x,
	float* ddx
);

/*
General 2-D lookup table function, no interpolation
	const breakpoints_t* bp_x - Structure containing breakpoint data 1st dimension
	const breakpoints_t* bp_y - Structure containing breakpoint data 2nd dimension
	int length_x - Length of lookup table 1st dimension
	int length_y - Length of lookup table 2nd dimension
	const int16_t lut[length_x][length_y] - Lookup table data
	float x - Value to lookup data at
*/
float __attribute__((unused)) lookup_2d_nointerp(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const int16_t lut[length_x][length_y],
	float x, float y
);

/*
 General 2-D lookup table function with interpolation and optional gradient
	const breakpoints_t* bp_x - Structure containing breakpoint data 1st dimension
	const breakpoints_t* bp_y - Structure containing breakpoint data 2nd dimension
	int length_x - Length of lookup table 1st dimension
	int length_y - Length of lookup table 2nd dimension
	const int16_t lut[length_x][length_y] - Lookup table data
	float x - Value of 1st dimension to lookup data at
	float y - Value of 2nd dimension to lookup data at
	float* ddx - (optional) Pass a non-NULL pointer which gets populated with the partial derivative value
	float* ddy - (optional) Pass a non-NULL pointer which gets populated with the partial derivative value
 */
float __attribute__((unused)) lookup_2d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const int16_t lut[length_x][length_y],
	float x, float y,
	float* ddx, float* ddy
);

/*
General 3-D lookup table function, no interpolation
	const breakpoints_t* bp_x - Structure containing breakpoint data 1st dimension
	const breakpoints_t* bp_y - Structure containing breakpoint data 2nd dimension
	const breakpoints_t* bp_z - Structure containing breakpoint data 2nd dimension
	int length_x - Length of lookup table 1st dimension
	int length_y - Length of lookup table 2nd dimension
	int length_z - Length of lookup table 2nd dimension
	const int16_t lut[length_x][length_y][length_z] - Lookup table data
	float x - Value of 1st dimension to lookup data at
	float y - Value of 2nd dimension to lookup data at
	float z - Value of 3rd dimension to lookup data at
*/
float lookup_3d_nointerp(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y, const breakpoints_t* bp_z,
	int length_x, int length_y, int length_z,
	const int16_t lut[length_x][length_y][length_z],
	float x, float y, float z
);

/*
General 3-D lookup table function with interpolation and optional gradient
	const breakpoints_t* bp_x - Structure containing breakpoint data 1st dimension
	const breakpoints_t* bp_y - Structure containing breakpoint data 2nd dimension
	const breakpoints_t* bp_z - Structure containing breakpoint data 2nd dimension
	int length_x - Length of lookup table 1st dimension
	int length_y - Length of lookup table 2nd dimension
	int length_z - Length of lookup table 2nd dimension
	const int16_t lut[length_x][length_y][length_z] - Lookup table data
	float x - Value of 1st dimension to lookup data at
	float y - Value of 2nd dimension to lookup data at
	float z - Value of 3rd dimension to lookup data at
	float* ddx - (optional) Pass a non-NULL pointer which gets populated with the partial derivative value
	float* ddy - (optional) Pass a non-NULL pointer which gets populated with the partial derivative value
	float* ddz - (optional) Pass a non-NULL pointer which gets populated with the partial derivative value
*/
float lookup_3d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y, const breakpoints_t* bp_z,
	int length_x, int length_y, int length_z,
	const int16_t lut[length_x][length_y][length_z],
	float x, float y, float z,
	float* ddx, float* ddy, float* ddz
);

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

// Inline Functions

// Initializes breakpoints and calculates point spacing for you
static inline breakpoints_t init_breakpoints(float min, float max, int n) {
    breakpoints_t bp = {
        .min_point = min,
        .max_point = max,
        .num_points = n,
        .point_spacing = (max - min) / (n - 1),
    };
    return bp;
}

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

// Trilinear interpolation of partial derivatives
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

#endif /* INC_LUT_H_ */