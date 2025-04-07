/*
 * pid.h
 *
 * This handles all PID functionality a torque vectoring strategy might use.
 *
 *  Created on: Mar 7, 2025
 *      Author: bglen
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// Includes

// Constants

// Types
typedef struct {
	double kp;
	double ki;
	double kd;
} gain_t;

typedef struct {
	gain_t gain;	// Struct of currently used gains

	double T; 		// Sampling period in seconds
	double tau;		// Time constant for kd 1st order filter

	double e_1; 	// Value storing E[k-1]
	double ui_1; 	// Value storing Ui[k-1]
	double ud_1; 	// Value storing Ud[k-1]

	double u;		// Value of controller output E[k]
} pid_t;

// Function Prototypes

/*
 * Initializes PID controller with gains and sampling time.
 *
 * Inputs:
 * pid_t pid_data - pid_t struct storing PID data
 * double T - sampling period in seconds
 * double tau - time constant of optional D filter, can be 0
 */
void init_pid(pid_t* pid_data, double T, double tau);

/*
 * Updates PID gains used by the controller
 */
void update_gains(pid_t* pid_data, gain_t new_gain);

/*
 * Schedules gains by interpolating the gain lookup table
 */
gain_t schedule_gains(float ref_velocity, float sw_angle);

/*
 * Updates the PID controller
 *
 * Inputs:
 * double e - holds discrete input value e[k]
 * pid_t pid_data - pid_t struct storing PID data
 *
 * Output:
 * double containing u[k]
 */
void update_pid(double e, pid_t* pid_data);

#endif /* INC_PID_H_ */
