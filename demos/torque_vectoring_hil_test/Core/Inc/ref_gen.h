/*
 * ref_gen.h
 * This module handles everything related to generating the yaw moment
 * and longitudinal acceleration reference values for the torque
 * vectoring algorith.
 *
 *  Created on: Apr 6, 2025
 *      Author: bglen
 */

#ifndef INC_REF_GEN_H_
#define INC_REF_GEN_H_

// Includes

// Constants

// Types
typedef struct {
    float delta_lf;
    float delta_rf;
} DeltaPair_t;

// Function Prototypes

/* 
Returns FL and FR steer angles given steering wheel angle
    float sw_angle - Steering Wheel Angle in radians
    DeltaPair_t -  delta_fl and delta_fr in radians
*/
DeltaPair_t get_steering_angle(float steer_angle);

/*
Calculates the Yaw Reference Signal for the Yaw Controller
    float sw_angle - Steering wheel angle, radians
    float ref_velocity - Reference velocity, m/s
    float yaw_ref - Yaw rate reference, rad/s
*/
float calculate_yaw_reference(float sw_angle, float ref_velocity);

/*
Calculates the Yaw Reference Signal for the Yaw Controller
    float apps_percent - Accelerator pedal percentage, percent
    float ax_ref - Longitudinal acceleration reference, m/s^2
*/
float __attribute__((unused)) calculate_acceleration_reference(float apps_percent, float ref_velocity);

#endif /* INC_REF_GEN_H_ */
