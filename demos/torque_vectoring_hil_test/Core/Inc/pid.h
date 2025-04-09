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
	float kp;
	float ki;
	float kd;
} gain_t;

typedef struct {
	gain_t gain;	// Struct of currently used gains

	float T; 		// Sampling period in seconds
	float tau;		// Time constant for kd 1st order filter

	float e_1; 	// Value storing E[k-1]
	float ui_1; 	// Value storing Ui[k-1]
	float ud_1; 	// Value storing Ud[k-1]

	float u;		// Value of controller output E[k]
} pid_t;

// Function Prototypes

/*
 * Initializes PID controller with gains and sampling time.
 *	pid_t pid_data - pid_t struct storing PID data
 *	float T - sampling period in seconds
 *	float tau - time constant of optional D filter, can be 0
 */
void init_pid(pid_t* pid_data, float T, float tau);

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
 *	float e - holds discrete input value e[k]
 *	pid_t pid_data - pid_t struct storing PID data
 *	float containing u[k]
 */
void update_pid(pid_t* pid_data, float e);

#endif /* INC_PID_H_ */
