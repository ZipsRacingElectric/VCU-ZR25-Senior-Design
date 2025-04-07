/*
 * lut_utils.h
 *
 *  Created on: Apr 7, 2025
 *      Author: tetra, bglen
 */

#ifndef INC_LUT_UTILS_H_
#define INC_LUT_UTILS_H_

#include "math.h"

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

// helpers to get all points around a segment/square/cube cell of an LUT
#define BOUNDS_SEGMENT(lut_1d,segment,x) \
	(segment)[0] = (lut_1d)[x]; \
	(segment)[1] = (lut_1d)[x+1]
#define BOUNDS_SQUARE(lut_2d,square,x,y) \
	BOUNDS_SEGMENT((lut_2d)[x],square[0],y); \
	BOUNDS_SEGMENT((lut_2d)[x+1],square[1],y)
#define BOUNDS_CUBE(lut_3d,cube,x,y,z) \
	BOUNDS_SQUARE((lut_3d)[x], cube[0], y,z); \
	BOUNDS_SQUARE((lut_3d)[x+1], cube[1], y,z)

// Use a breakpoint array to convert from raw measurement -> LUT index
// Clamps high/low if out of bounds.
// Low clamping edge will be index zero, with interp=0.
// High clamping edge will be second-to-last index, with interp=1.
// Never returns index of last point (num_points-1).
// If `interp` is non-NULL, it will be written with an interpolation value between 0-1.
static inline int get_index(const breakpoints_t *breakpoints, float value, float* interp) {
	if (value < breakpoints->min_point) {
		// Clamp low.
		if (interp) {
			*interp = 0.0;
		}
		return 0;
	}
	if (value > breakpoints->max_point) {
		// Clamp high.
		if (interp) {
			*interp = 1.0;
		}
		return breakpoints->num_points-2;
	}
	float idx_f = floor((value - breakpoints->min_point)/breakpoints->point_spacing);
	int idx = (int)idx_f;
	if (idx == breakpoints->num_points-1) {
		// Never return index of last point,
		// rather return index of 2nd-to-last point with interp set to 1.
		idx--;
	}
	if (interp) {
		*interp = (value - (breakpoints->min_point + idx_f*breakpoints->point_spacing))/breakpoints->point_spacing;
	}
	return idx;
}

// Derivative calculations:
// 1. Get derivative w.r.t. interpolation factors from trilerp_ddx and friends.
// 2. Get derivative of interpolation factors w.r.t. measurements.
//    Since interpolation factor varies from 0-1 across one cell,
//    this rate is actually just 1/(distance between breakpoints)!
// 3. Multiply the above derivatives to get the derivative via chain rule:
//    d(lut)/dx = d(lut)/d(interp_x) * d(interp_x)/dx
//              = (bi-)(tri-)lerp_ddx(lut) / point_spacing_x

/*
General 1-D lookup table function, no interpolation
	const breakpoints_t* bp_x - Structure containing breakpoint information
	int length_x - Length of lookup table
	const int16_t lut[length_x] - Lookup table data
	float x - Value of breakpoint to lookup data at
*/
static float __attribute__((unused)) lookup_1d_nointerp(
	const breakpoints_t* bp_x,
	int length_x,
	const float lut[length_x],
	float x
) {
	int x_idx = get_index(bp_x, x, NULL);
	return lut[x_idx];
}

/*
 General 1-D lookup table function with interpolation and optional gradient
	const breakpoints_t* bp_x - Structure containing breakpoint data
	int length_x - Length of lookup table
	const int16_t lut[length_x] - Lookup table data
	float x - Value of breakpoint to lookup data at
	float* ddx - (optional) Pass a non-NULL pointer which gets populated with the gradient value
 */
static float __attribute__((unused)) lookup_1d(
	const breakpoints_t* bp_x,
	int length_x,
	const float lut[length_x],
	float x,
	float* ddx
) {
	float interp_x;
	int x_idx = get_index(bp_x, x, &interp_x);

	float point_segment[2];
	BOUNDS_SEGMENT(lut, point_segment, x_idx);

	float r = lerp(point_segment[0], point_segment[1], interp_x);
	if (ddx)
		*ddx = lerp_ddx(point_segment[0], point_segment[1], interp_x) / bp_x->point_spacing;
	return r;
}

/*
General 2-D lookup table function, no interpolation
	const breakpoints_t* bp_x - Structure containing breakpoint data 1st dimension
	const breakpoints_t* bp_y - Structure containing breakpoint data 2nd dimension
	int length_x - Length of lookup table 1st dimension
	int length_y - Length of lookup table 2nd dimension
	const int16_t lut[length_x][length_y] - Lookup table data
	float x - Value to lookup data at
*/
static float __attribute__((unused)) lookup_2d_nointerp(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const float lut[length_x][length_y],
	float x, float y
) {
	int x_idx = get_index(bp_x, x, NULL);
	int y_idx = get_index(bp_y, y, NULL);
	return lut[x_idx][y_idx];
}

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
static float __attribute__((unused)) lookup_2d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const float lut[length_x][length_y],
	float x, float y,
	float* ddx, float* ddy
) {
	float interp_x, interp_y;
	int x_idx = get_index(bp_x, x, &interp_x);
	int y_idx = get_index(bp_y, y, &interp_y);

	float point_square[2][2];
	BOUNDS_SQUARE(lut, point_square, x_idx, y_idx);

	float r = bilerp(point_square, interp_x, interp_y);
	if (ddx)
		*ddx = bilerp_ddx(point_square, interp_x, interp_y) / bp_x->point_spacing;
	if (ddy)
		*ddy = bilerp_ddy(point_square, interp_x, interp_y) / bp_y->point_spacing;
	return r;
}

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
static float __attribute__((unused)) lookup_3d_nointerp(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y, const breakpoints_t* bp_z,
	int length_x, int length_y, int length_z,
	const float lut[length_x][length_y][length_z],
	float x, float y, float z
) {
	int x_idx = get_index(bp_x, x, NULL);
	int y_idx = get_index(bp_y, y, NULL);
	int z_idx = get_index(bp_z, z, NULL);
	return lut[x_idx][y_idx][z_idx];
}

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
static float __attribute__((unused)) lookup_3d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y, const breakpoints_t* bp_z,
	int length_x, int length_y, int length_z,
	const float lut[length_x][length_y][length_z],
	float x, float y, float z,
	float* ddx, float* ddy, float* ddz
) {
	float interp_x, interp_y, interp_z;
	int x_idx = get_index(bp_x, x, &interp_x);
	int y_idx = get_index(bp_y, y, &interp_y);
	int z_idx = get_index(bp_z, z, &interp_z);

	float point_cube[2][2][2];
	BOUNDS_CUBE(lut, point_cube, x_idx, y_idx, z_idx);

	float r = trilerp(point_cube, interp_x, interp_y, interp_z);
	if (ddx)
		*ddx = trilerp_ddx(point_cube, interp_x, interp_y, interp_z) / bp_x->point_spacing;
	if (ddy)
		*ddy = trilerp_ddy(point_cube, interp_x, interp_y, interp_z) / bp_y->point_spacing;
	if (ddz)
		*ddz = trilerp_ddz(point_cube, interp_x, interp_y, interp_z) / bp_z->point_spacing;
	return r;
}

#endif /* INC_LUT_UTILS_H_ */
