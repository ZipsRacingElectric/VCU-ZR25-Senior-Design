/*
 * pid.c
 *
 * This handles all PID functionality a torque vectoring strategy might use.
 *
 *  Created on: Mar 7, 2025
 *      Author: bglen
 */

// Includes
#include <pid.h>

// Defines
#define PID_LUT_V_MIN 2			// Minimum breakpoint value for velocity, m/s
#define PID_LUT_V_MAX 50		// Maximum breakpoint value for velocity, m/s
#define PID_LUT_V_STEP	9.6		// Step size for velocity breakpoint
#define PID_LUT_DELTA_MIN 10	// Minimum breakpoint value for steering angle, degrees
#define PID_LUT_DELTA_MAX 90	// Maximum breakpoint value for steering angle, degrees
#define PID_LUT_DELTA_STEP 20	// Step size for steering breakpoint

// Constants
// PID gain Lut has breakpoints for velocity and steering angle
// TODO: replace with evenly spaced breakpoint data (tetras got this)
static const gain_t pid_gains[30] = {
	{25320, 11396, 0}, { 8344, 11663, 0}, { 2484,  5680, 0}, { 1000,  1000, 0}, {34132, 29683, 0}, {21997, 66594, 0},
	{25320, 11396, 0}, { 8300, 12000, 0}, { 2484,  5680, 0}, { 6993,  8782, 0}, {34132, 29683, 0}, {34452, 70850, 0},
	{25320, 11396, 0}, { 8300, 12000, 0}, { 2484,  5680, 0}, { 8642,  2194, 0}, {35054, 34602, 0}, {46685, 83942, 0},
	{25320, 11396, 0}, { 8300, 12000, 0}, { 2484,  5680, 0}, {71300, 13884, 0}, {44181, 42895, 0}, {50763, 50763, 0},
	{25320, 11396, 0}, { 8300, 12000, 0}, { 2484,  5680, 0}, {71300, 13884, 0}, {48881, 49547, 0}, {50763, 99999, 0}
};

// Static variables

// Private Function Prototypes

// Public Function Definitions

/*
 * Initializes PID controller with gains and sampling time.
 *
 * Inputs:
 * pid_t pid_data - pid_t struct storing PID data
 * double T - sampling period in seconds
 * double tau - time constant of optional D filter, can be 0
 */
void init_pid(pid_t* pid_data, double T, double tau) {
	pid_data->gain.kp = 0;
	pid_data->gain.ki = 0;
	pid_data->gain.kd = 0;
	pid_data->T = T;
	pid_data->tau = tau;
	pid_data->e_1 = 0;
	pid_data->ui_1 = 0;
	pid_data->ud_1 = 0;
	pid_data->u = 0;
}

/*
 * Updates PID gains used by the controller
 */
void update_gains(pid_t* pid_data, gain_t new_gain) {
	pid_data->gain.kp = new_gain.kp;
	pid_data->gain.ki = new_gain.ki;
	pid_data->gain.kd = new_gain.kd;
}

/*
 * Schedules gains by interpolating the gain lookup table
 */
gain_t schedule_gains(float ref_velocity, float sw_angle) {
	// Interpolate gain LUT at sample points
	// TODO: replace with tetra functions
	gain_t new_gains = {1, 1, 0};
	return new_gains;
}

/*
 * Updates the PID controller
 *
 * Inputs:
 * double e - holds discrete input value e[k]
 * pid_t pid_data - pid_t struct storing PID data
 *
 * Output:
 * double containing u[k]
 *
 * Description:
 * PID controller with D term filtering is implemented as:
 *
 * u[k] = up[k] + ui[k] + ud[k], where
 *
 * up[k] = kp * e[k]
 * ui[k] = (ki*T/2) * (e[k] - e[k-1]) + ui[k-1]
 * ud[k] = (2*kd)/(2*tau+T) * (e[k] - e[k-1]) + (2*tau-T)/(2*tau+T) * ud[k-1]
 */
void update_pid(double e, pid_t* pid_data) {
	double u = 0;
	double kp = pid_data->gain.kp;
	double ki = pid_data->gain.ki;
	double kd = pid_data->gain.kd;
	double T = pid_data->T;
	double tau = pid_data->tau;
	double e_1 = pid_data->e_1;
	double ui_1 = pid_data->ui_1;
	double ud_1 = pid_data->ud_1;

	// Calculate u[k] output
	double up = kp * e;
	double ui = (ki * T / 2) * (e - e_1) + ui_1;
	double ud = (2 * kd) / (2 * tau + T) * (e - e_1) + (2 * tau - T) / (2 * tau + T) * ud_1;
	u = up + ui + ud;

	// Update the stored prior samples
	pid_data->e_1 = e; // Save the input as the prior input
	pid_data->ui_1 = ui;
	pid_data->ud_1 = ud;

	// Update the controller output
	pid_data->u = u;
}

