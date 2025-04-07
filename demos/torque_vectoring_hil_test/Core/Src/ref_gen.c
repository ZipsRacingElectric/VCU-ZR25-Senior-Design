/*
* ref_gen.c
* This module handles everything related to generating the yaw moment
* and longitudinal acceleration reference values for the torque
* vectoring algorith.
*
*  Created on: Apr 6, 2025
*      Author: bglen
*/

// Includes
#include <ref_gen.h>
#include <stdlib.h>
#include "lut.h"

// Defines
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define MIN_REF_VELOCITY 0.01f  // Minimum velocity for yaw reference generation

// Constants

// Steering Angle Lookup Table, degrees
// TODO: store these in radians
static const float steering_lf_table[11] = {
    -18.2240f,
    -14.5580f,
    -10.9180f,
     -7.2890f,
     -3.6540f,
      0.0000f,
      3.6890f,
      7.4280f,
     11.2340f,
     15.1260f,
     19.1270f,
};

static const float steering_rf_table[11] = {
    -19.1260f,
    -15.1260f,
    -11.2340f,
     -7.4280f,
     -3.6890f,
      0.0000f,
      3.6540f,
      7.2890f,
     10.9180f,
     14.5580f,
     18.2250f,
};

static const breakpoints_t steering_breakpoints = {-90.0f, 90.0f, 11, 18.0f};
static const float g = 9.81f;     // graviationa acceleration, m/s

// Static Variables
static float sigma = 1.0f;          // Tuning parameter for maximum yaw rate
static float mu = 2.0f;             // Maximum tire friction coefficient
static float ku = 0.0f;             // Target understeer gradient, d_delta / d_ay. Tune to adjust reference yaw rate.
static float wheelbase = 1.528f;    // TODO: replace with vehicle_data.h

// Private Function Prototypes
static inline float deg_to_rad(float deg);
static inline float rad_to_deg(float rad);

// Function Definitions

/* 
Returns FL and FR steer angles given steering wheel angle
    float sw_angle - Steering Wheel Angle in radians
    DeltaPair_t -  delta_fl and delta_fr in radians
*/
// TODO: handle faults from interpolation
DeltaPair_t get_steering_angle(float sw_angle) {
    sw_angle = rad_to_deg(sw_angle);

    // 1-D Interpolation of lookup table
    double delta_lf = lookup_1d(&steering_breakpoints, 11, steering_lf_table, sw_angle, NULL);
    double delta_rf = lookup_1d(&steering_breakpoints, 11, steering_rf_table, sw_angle, NULL);
    delta_lf = deg_to_rad(delta_lf);
    delta_rf = deg_to_rad(delta_rf);

    DeltaPair_t angles = {delta_lf, delta_rf};

    return angles;
}

/*
Calculates the Yaw Reference Signal for the Yaw Controller
    float sw_angle - Steering wheel angle, radians
    float ref_velocity - Reference velocity, m/s
    float yaw_ref - Yaw rate reference, rad/s
*/
float calculate_yaw_reference(float sw_angle, float ref_velocity) {
    // Calculate average front tire steer angle
    DeltaPair_t angles = get_steering_angle(sw_angle);
    double delta = (angles.delta_lf + angles.delta_rf) / 2;

    double yaw_ref = 0.0f; // Default value for zero or negative speed condition

    // Calculate maximum physical yaw rate
    if (ref_velocity > MIN_REF_VELOCITY) {
        double yaw_rate_max = sigma * mu * g / ref_velocity;

        // Calculate yaw rate reference
        yaw_ref = ref_velocity / (wheelbase + ku * ref_velocity * ref_velocity) * delta;

        if(yaw_ref > yaw_rate_max) {
            yaw_ref = yaw_rate_max;
        }
    }
    return yaw_ref;
}

/*
Calculates the Yaw Reference Signal for the Yaw Controller
    float apps_percent - Accelerator pedal percentage, percent
    float ax_ref - Longitudinal acceleration reference, m/s^2
*/
float __attribute__((unused)) calculate_acceleration_reference(float apps_percent, float ref_velocity) {
    // TODO
    return 0.0f;
}

// Private Function Declarations

// Converts degrees to radians
// TODO: remove these in the future
static inline float deg_to_rad(float deg) {
    return deg * (PI / 180.0f);
}

// Converts radians to degrees
static inline float rad_to_deg(float rad) {
    return rad * (180.0f / PI);
}
