/*
 * vd.h
 * This module contains all functions related to caclulating vehicle dynamics.
 *
 *  Created on: Apr 6, 2025
 *      Author: bglen
 */

#ifndef INC_VD_H_
#define INC_VD_H_

// Includes

// Vehicle Constant Parameters
#define G 9.81

#define TOTAL_MASS 298.645 // kg
#define FRONT_MASS_DISTRIBUTION 0.48 // percentage
#define CG_HEIGHT 0.25756 // m

#define TRACK_WIDTH_FRONT 1.23 // m
#define TRACK_WIDTH_REAR 1.21 // m
#define WHEELBASE 1.53 // m
#define WHEELBASE_FRONT (WHEELBASE * (1 - FRONT_MASS_DISTRIBUTION)) // m
#define WHEELBASE_REAR (WHEELBASE * FRONT_MASS_DISTRIBUTION)  // m

#define TLLTD 0.51165335080032859 // percent

#define EFFECTIVE_RADIUS 0.25 // m

// Constants

// Types

// Tire Normal Force - N
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} normal_force_t;

// Downforce at axle - N
typedef struct {
    float front_axle;
    float rear_axle;
} downforce_t;

// Steering Angles Struct - rad
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} steering_angles_t;

// Slip Angles Struct - rad
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} slip_angles_t;

// Slip Ratios Struct - unitless
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} slip_ratios_t;

// Wheel Speeds Struct - rad/s
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} wheel_speeds_t;

// Tire Surface Velocities Struct - m/s
typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} surf_vel_t;

/*
 * Vehicle Body Struct
 * This holds data relating to vehicle body motion
 * ISO 8855 coordinate system (same as VD models and diagrams)
 */
typedef struct {
    // TODO: should this be referred to as reference velocity or velocity_x?
    float velocity_x;              // Vehicle-frame velocity forward direction (Reference Velocity), m/s
    float velocity_y;              // Vehicle-frame velocity lateral direction, m/s
    float yaw_rate;                // Angular velocity around yaw (vertical) axis, rad/s
    float yaw_acceleration;        // Angular acceleration around yaw (vertical) axis, rad/s^2
    float acceleration_x;          // Vehicle-frame acceleration forward direction, m/s^2
    float acceleration_y;          // Vehicle-frame acceleration lateral direction, m/s^2
    float side_slip_angle;         // Angle between vehicle heading and velocity heading, rad
    wheel_speeds_t wheel_speeds;   // Wheel speeds, positive is forward motion, rad/s
} vehicle_body_t;

// Function Prototypes

/*
 * Calculates tire normal forces from accelerations. Compare to measured values.
 *   vehicle_body_t *vehicle
 *   downforce_t *downforce
 *   normal_force_t *normal_force
 */
void __attribute__((unused)) calculate_normal_force(vehicle_body_t *vehicle, downforce_t *downforce, normal_force_t *normal_force);

/*
 * Calculates tire normal forces from strain gauges.
 */
// TODO
void __attribute__((unused)) calculate_normal_force_strain_gauge(float normal_force[4]);

/*
 * Calculates slip angles on each tire.
 *   vehicle_body_t *vehicle
 *   steering_angles_t *delta
 *   slip_angles_t *alpha
 */
void __attribute__((unused)) calculate_slip_angles(vehicle_body_t *vehicle, steering_angles_t *delta, slip_angles_t *alpha);

/*
 * Calculates tire surface velocites. Used for slip angle determination.
 *   vehicle_body_t *vehicle
 *   steering_angles_t *delta
 *   slip_angles_t *alpha
 *   surf_vel_t *surface_velocities
 */
void __attribute__((unused)) calculate_tire_surface_velocities(vehicle_body_t *vehicle, steering_angles_t *delta, slip_angles_t *alpha, surf_vel_t *surf_vel);

/*
 * Calculate tire slip ratios
 *   vehicle_body_t *vehicle
 *   surf_vel_t *surf_vel
 *   slip_ratios_t *slip_ratios
 */
void __attribute__((unused)) calculate_slip_ratios(vehicle_body_t *vehicle,  surf_vel_t *surf_vel, slip_ratios_t *slip_ratios);

#endif /* INC_VD_H_ */
