/*
 * lut_utils.h
 *
 *  Created on: Apr 7, 2025
 *      Author: tetra
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
// Returns -1 if out of bounds
// If `interp` is non-NULL, it will be written with an interpolation value between 0-1.
static inline int get_index(const breakpoints_t *breakpoints, float value, float* interp) {
	if (value < breakpoints->min_point || value > breakpoints->max_point) {
		return -1;
	}
	float idx_f = floor((value - breakpoints->min_point)/breakpoints->point_spacing);
	int idx = (int)idx_f;
	if (interp) {
		*interp = (value - idx_f*breakpoints->point_spacing)/breakpoints->point_spacing;
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

static float __attribute__((unused)) lookup_1d_nointerp(
	const breakpoints_t* bp_x,
	int length_x,
	const int16_t lut[length_x],
	float x
) {
	int x_idx = get_index(bp_x, x, NULL);
	// error check
	if (x_idx == -1)
		{/*TODO: Raise a fault*/}
	return lut[x_idx];
}

// Generalized lookup-and-gradient for 1d LUTs.
static float __attribute__((unused)) lookup_1d(
	const breakpoints_t* bp_x,
	int length_x,
	const int16_t lut[length_x],
	float x,
	float* ddx
) {
	float interp_x;
	int x_idx = get_index(bp_x, x, &interp_x);
	// error check
	if (x_idx == -1)
		{/*TODO: Raise a fault*/}

	float point_segment[2];
	BOUNDS_SEGMENT(lut, point_segment, x_idx);

	float r = lerp(point_segment[0], point_segment[1], interp_x);
	if (ddx)
		*ddx = lerp_ddx(point_segment[0], point_segment[1], interp_x) / bp_x->point_spacing;
	return r;
}

static float __attribute__((unused)) lookup_2d_nointerp(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const int16_t lut[length_x][length_y],
	float x, float y
) {
	int x_idx = get_index(bp_x, x, NULL);
	int y_idx = get_index(bp_y, y, NULL);
	// error check
	if (x_idx == -1 || y_idx == -1)
		{/*TODO: Raise a fault*/}
	return lut[x_idx][y_idx];
}

// Generalized lookup-and-gradient for 2d LUTs.
static float __attribute__((unused)) lookup_2d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const int16_t lut[length_x][length_y],
	float x, float y,
	float* ddx, float* ddy
) {
	float interp_x, interp_y;
	int x_idx = get_index(bp_x, x, &interp_x);
	int y_idx = get_index(bp_y, y, &interp_y);
	// error check
	if (x_idx == -1 || y_idx == -1)
		{/*TODO: Raise a fault*/}

	float point_square[2][2];
	BOUNDS_SQUARE(lut, point_square, x_idx, y_idx);

	float r = bilerp(point_square, interp_x, interp_y);
	if (ddx)
		*ddx = bilerp_ddx(point_square, interp_x, interp_y) / bp_x->point_spacing;
	if (ddy)
		*ddy = bilerp_ddy(point_square, interp_x, interp_y) / bp_y->point_spacing;
	return r;
}

static float lookup_3d_nointerp(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y, const breakpoints_t* bp_z,
	int length_x, int length_y, int length_z,
	const int16_t lut[length_x][length_y][length_z],
	float x, float y, float z
) {
	int x_idx = get_index(bp_x, x, NULL);
	int y_idx = get_index(bp_y, y, NULL);
	int z_idx = get_index(bp_z, z, NULL);
	// error check
	if (x_idx == -1 || y_idx == -1 || z_idx == -1)
		{/*TODO: Raise a fault*/}
	return lut[x_idx][y_idx][z_idx];
}

// Generalized lookup-and-gradient for 3d LUTs.
static float lookup_3d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y, const breakpoints_t* bp_z,
	int length_x, int length_y, int length_z,
	const int16_t lut[length_x][length_y][length_z],
	float x, float y, float z,
	float* ddx, float* ddy, float* ddz
) {
	float interp_x, interp_y, interp_z;
	int x_idx = get_index(bp_x, x, &interp_x);
	int y_idx = get_index(bp_y, y, &interp_y);
	int z_idx = get_index(bp_z, z, &interp_z);
	// error check
	if (x_idx == -1 || y_idx == -1 || z_idx == -1)
		{/*TODO: Raise a fault*/}

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
