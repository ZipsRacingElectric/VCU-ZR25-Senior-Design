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
	double kp; 		// Kp gain
	double ki; 		// Ki gain
	double kd; 		// Kd gain

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
 *
 */
void init_pid(pid_t* pid_data, double T, double tau);

/*
 * Updates PID gains
 *
 */
void update_gains(pid_t* pid_data, double kp, double ki, double kd);

/*
 * Updates the PID controller data
 *
 * Inputs:
 * double e - containts discrete input value e[k]
 * pid_t pid_data - pid_t struct storing PID data
 */
void update_pid(double e, pid_t* pid_data);



#endif /* INC_PID_H_ */
