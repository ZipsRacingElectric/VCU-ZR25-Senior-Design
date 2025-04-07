/*
 * pid.c
 *
 * This handles all PID functionality a torque vectoring strategy might use.
 *
 *  Created on: Mar 7, 2025
 *      Author: bglen
 */

// Includes
#include <stdlib.h>
#include "pid.h"
#include "lut.h"

// Defines

// Constants

// PI gain tables has breakpoints for steering angle and velocity
// TODO: We need to fix these, they got messed up when they got extrapolated
// TODO: Need to switch dimensions so it is kp_gains[ref_velocity][sw_angle]
static const float kp_gains[26][10] = {
	{25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f},
	{25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f},
	{25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f},
	{25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f, 25320.0f},
	{8344.4f, 8344.4f, 8344.4f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f},
	{8344.4f, 8344.4f, 8344.4f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f},
	{8344.4f, 8344.4f, 8344.4f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f, 8300.0f},
	{2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 6836.2f, 6836.2f, 6836.2f},
	{2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 6836.2f, 6836.2f, 6836.2f},
	{2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 6836.2f, 6836.2f, 6836.2f},
	{2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 2483.7f, 6836.2f, 6836.2f, 6836.2f},
	{1000.0f, 1000.0f, 1000.0f, 6993.3f, 6993.3f, 8642.0f, 8642.0f, 71300.0f, 71300.0f, 71300.0f},
	{1000.0f, 1000.0f, 1000.0f, 6993.3f, 6993.3f, 8642.0f, 8642.0f, 71300.0f, 71300.0f, 71300.0f},
	{1000.0f, 1000.0f, 1000.0f, 6993.3f, 6993.3f, 8642.0f, 8642.0f, 71300.0f, 71300.0f, 71300.0f},
	{1000.0f, 1000.0f, 1000.0f, 6993.3f, 6993.3f, 8642.0f, 8642.0f, 71300.0f, 71300.0f, 71300.0f},
	{1000.0f, 1000.0f, 1000.0f, 6993.3f, 6993.3f, 8642.0f, 8642.0f, 71300.0f, 71300.0f, 71300.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f},
	{34132.0f, 34132.0f, 34132.0f, 34132.0f, 34132.0f, 35054.0f, 35054.0f, 44181.0f, 44181.0f, 44181.0f}
};

static const float ki_gains[26][10] = {
	{11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f},
	{11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f},
	{11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f},
	{11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f, 11396.0f},
	{11663.0f, 11663.0f, 11663.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f},
	{11663.0f, 11663.0f, 11663.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f},
	{11663.0f, 11663.0f, 11663.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f, 12000.0f},
	{5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 46016.0f, 46016.0f, 46016.0f},
	{5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 46016.0f, 46016.0f, 46016.0f},
	{5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 46016.0f, 46016.0f, 46016.0f},
	{5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 5679.9f, 46016.0f, 46016.0f, 46016.0f},
	{1000.0f, 1000.0f, 1000.0f, 8782.2f, 8782.2f, 2193.8f, 2193.8f, 13884.0f, 13884.0f, 13884.0f},
	{1000.0f, 1000.0f, 1000.0f, 8782.2f, 8782.2f, 2193.8f, 2193.8f, 13884.0f, 13884.0f, 13884.0f},
	{1000.0f, 1000.0f, 1000.0f, 8782.2f, 8782.2f, 2193.8f, 2193.8f, 13884.0f, 13884.0f, 13884.0f},
	{1000.0f, 1000.0f, 1000.0f, 8782.2f, 8782.2f, 2193.8f, 2193.8f, 13884.0f, 13884.0f, 13884.0f},
	{1000.0f, 1000.0f, 1000.0f, 8782.2f, 8782.2f, 2193.8f, 2193.8f, 13884.0f, 13884.0f, 13884.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f},
	{29683.0f, 29683.0f, 29683.0f, 29683.0f, 29683.0f, 34602.0f, 34602.0f, 42895.0f, 42895.0f, 42895.0f}
};

static const breakpoints_t pid_dim_1_breakpoints = {0.0f, 90.0f, 10, 10.0f};	// Steering Angle
static const breakpoints_t pid_dim_2_breakpoints = {0.0f, 50.0f, 26, 2.0f};		// Reference Velocity

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
void init_pid(pid_t* pid_data, float T, float tau) {
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
	float new_kp = lookup_2d(&pid_dim_1_breakpoints, &pid_dim_2_breakpoints, 10, 26, kp_gains, sw_angle, ref_velocity, NULL, NULL);
	float new_ki = lookup_2d(&pid_dim_1_breakpoints, &pid_dim_2_breakpoints, 10, 26, ki_gains, sw_angle, ref_velocity, NULL, NULL);
	gain_t new_gains = {new_kp, new_ki, 0.0f};
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
void update_pid(pid_t* pid_data, float e) {
	float u = 0;
	float kp = pid_data->gain.kp;
	float ki = pid_data->gain.ki;
	float kd = pid_data->gain.kd;
	float T = pid_data->T;
	float tau = pid_data->tau;
	float e_1 = pid_data->e_1;
	float ui_1 = pid_data->ui_1;
	float ud_1 = pid_data->ud_1;

	// Calculate u[k] output
	float up = kp * e;
	float ui = (ki * T / 2) * (e - e_1) + ui_1;
	float ud = (2 * kd) / (2 * tau + T) * (e - e_1) + (2 * tau - T) / (2 * tau + T) * ud_1;
	u = up + ui + ud;

	// Update the stored prior samples
	pid_data->e_1 = e; // Save the input as the prior input
	pid_data->ui_1 = ui;
	pid_data->ud_1 = ud;

	// Update the controller output
	pid_data->u = u;
}

