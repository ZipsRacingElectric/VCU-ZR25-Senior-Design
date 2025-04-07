/*
 * lut.c
 * Simplified version of tuning_params.h with just LUT functions
 *
 *  Created on: Apr 6, 2025
 *      Author: tetra, bglen
 */

#include "lut.h"
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

// Function Definitions

// Use a breakpoint array to convert from raw measurement -> LUT index
// Returns -1 if out of bounds
// If `interp` is non-NULL, it will be written with an interpolation value between 0-1.
int get_index(const breakpoints_t *breakpoints, float value, float* interp) {
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
	const float lut[length_x],
	float x
) {
	int x_idx = get_index(bp_x, x, NULL);
	// error check
	if (x_idx == -1)
		{/*TODO: Raise a fault*/}
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
float __attribute__((unused)) lookup_1d(
	const breakpoints_t* bp_x,
	int length_x,
	const float lut[length_x],
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
	const float lut[length_x][length_y],
	float x, float y
) {
	int x_idx = get_index(bp_x, x, NULL);
	int y_idx = get_index(bp_y, y, NULL);
	// error check
	if (x_idx == -1 || y_idx == -1)
		{/*TODO: Raise a fault*/}
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
float __attribute__((unused)) lookup_2d(
	const breakpoints_t* bp_x, const breakpoints_t* bp_y,
	int length_x, int length_y,
	const float lut[length_x][length_y],
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
	const float lut[length_x][length_y][length_z],
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
	const float lut[length_x][length_y][length_z],
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
