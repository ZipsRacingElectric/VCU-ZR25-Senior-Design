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

// Defines
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define SW_MIN_ANGLE -90        // min angle for LUT, degrees
#define SW_MAX_ANGLE 90         // max angle for LUT, degrees
#define SW_STEP 18              // breakpoint step for LUT, degrees
#define MIN_REF_VELOCITY 0.01f  // Minimum velocity for yaw reference generation

// Constants

// Steering Angle Lookup Table, degrees
// TODO: store these in radians
static const DeltaPair_t stering_angle_table[11] = {
    {-18.2240f, -19.1260f},
    {-14.5580f, -15.1260f},
    {-10.9180f, -11.2340f},
    { -7.2890f,  -7.4280f},
    { -3.6540f,  -3.6890f},
    {  0.0000f,   0.0000f},
    {  3.6890f,   3.6540f},
    {  7.4280f,   7.2890f},
    { 11.2340f,  10.9180f},
    { 15.1260f,  14.5580f},
    { 19.1270f,  18.2250f},
};

static const float g = 9.81f;     // graviationa acceleration, m/s

// Static Variables
static float sigma = 1.0f;  // Tuning parameter for maximum yaw rate
static float mu = 2.0f;     // Maximum tire friction coefficient
static float ku = 0.0f;     // Target understeer gradient, d_delta / d_ay. Tune to adjust reference yaw rate.
static float wheelbase = 1.528f;    // TODO: replace with vehicle_data.h


// Private Function Prototypes
static inline float deg_to_rad(float deg);
static inline float rad_to_deg(float rad);

// Function Definitions

/*
Calculates FL and FR steer angles given steering wheel angle
Inputs:
    float sw_angle - Steering Wheel Angle in radians
Outputs:
    DeltaPair_t -  delta_fl and delta_fr in radians
*/
DeltaPair_t get_steering_angle(float sw_angle) {
    sw_angle = rad_to_deg(sw_angle);

    // Interpolate LUT
    // TODO: use tetra's LUT functions for interpolation
    DeltaPair_t angles = {1, 1};

    angles.delta_lf = deg_to_rad(angles.delta_lf);
    angles.delta_rf = deg_to_rad(angles.delta_rf);

    return angles;
}

/*
Calculates the Yaw Reference Signal for the Yaw Controller
Inputs:
    float sw_angle - Steering wheel angle, radians
    float ref_velocity - Reference velocity, m/s
Output:
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
Inputs:
    float apps_percent - Accelerator pedal percentage, percent
Output:
    float ax_ref - Longitudinal acceleration reference, m/s^2
*/
float calculate_acceleration_reference(float apps_percent, float ref_velocity) {
    // TODO
    return 0.0f;
}

// Private Function Declarations

// Converts degrees to radians
static inline float deg_to_rad(float deg) {
    return deg * (PI / 180.0f);
}

// Converts radians to degrees
static inline float rad_to_deg(float rad) {
    return rad * (180.0f / PI);
}
