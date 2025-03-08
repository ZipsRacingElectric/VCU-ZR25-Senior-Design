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
	pid_data->kp = 0;
	pid_data->ki = 0;
	pid_data->kd = 0;
	pid_data->T = T;
	pid_data->tau = tau;
	pid_data->e_1 = 0;
	pid_data->ui_1 = 0;
	pid_data->ud_1 = 0;
	pid_data->u = 0;
}

/*
 * Updates PID gains
 *
 */
void update_gains(pid_t* pid_data, double kp, double ki, double kd) {
	pid_data->kp = kp;
	pid_data->ki = ki;
	pid_data->kd = kd;
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
	double kp = pid_data->kp;
	double ki = pid_data->ki;
	double kd = pid_data->kd;
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

