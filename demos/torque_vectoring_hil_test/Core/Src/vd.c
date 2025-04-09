/*
 * vd.c
 * This module contains all functions related to caclulating vehicle dynamics.
 *
 *  Created on: Apr 6, 2025
 *      Author: bglen
 */

// Includes
#include <math.h>
#include "vd.h"


// Constants

// Types

// Private Function Prototypes

// Function Definitions

/*
 * Calculates tire normal forces from accelerations. Compare to measured values.
 *   vehicle_body_t *vehicle
 *   downforce_t *downforce
 *   normal_force_t *normal_force
 */
void __attribute__((unused)) calculate_normal_force(vehicle_body_t *vehicle, downforce_t *downforce, normal_force_t *normal_force) {
    // TODO: remove constants from function
    float total_normal_load = TOTAL_MASS * G;
    float static_load_front = FRONT_MASS_DISTRIBUTION * total_normal_load;
    float static_load_rear = (1.0f - FRONT_MASS_DISTRIBUTION) * total_normal_load;
    float track_width = (TRACK_WIDTH_FRONT + TRACK_WIDTH_REAR) / 2.0f;

    float fx_vehicle = vehicle->acceleration_x * TOTAL_MASS;
    float fy_vehicle = vehicle->acceleration_y * TOTAL_MASS;
    float lateral_transfer = (fy_vehicle * CG_HEIGHT) / track_width;
    float longitudinal_transfer = (fx_vehicle * CG_HEIGHT) / WHEELBASE;

    float total_downforce = -(downforce->front_axle + downforce->rear_axle);

    // Combine static and transfer loads
    // TODO: downforce inputs are negative from LUT, everything else is calculated as positive and later inverted. Should be simplified
    // fz = Fz_static + Fz_transfer_lat + Fz_transfer_lat + Fz_downforce;
    float fz_fl = (static_load_front / 2.0f) + (lateral_transfer * -TLLTD) + (-longitudinal_transfer / 2.0f * (1.0f - FRONT_MASS_DISTRIBUTION)) + (downforce->front_axle * -0.5f);
    float fz_fr = (static_load_front / 2.0f) + (lateral_transfer * TLLTD) + (-longitudinal_transfer / 2.0f * (1.0f - FRONT_MASS_DISTRIBUTION)) + (downforce->front_axle * -0.5f);
    float fz_rl = (static_load_rear / 2.0f) + (lateral_transfer * -(1.0f - TLLTD)) + (longitudinal_transfer / 2.0f * FRONT_MASS_DISTRIBUTION) + (downforce->rear_axle * -0.5f);
    float fz_rr = (static_load_rear / 2.0f) + (lateral_transfer * (1.0f - TLLTD)) + (longitudinal_transfer / 2.0f * FRONT_MASS_DISTRIBUTION) + (downforce->rear_axle * -0.5f);

    // Clamp negative loads to zero
    //if a tire is off the ground, make it zero so tire model doesnt go crazy
    fz_fl = fmaxf(fz_fl, 0.0f);
    fz_fr = fmaxf(fz_fr, 0.0f);
    fz_rl = fmaxf(fz_rl, 0.0f);
    fz_rr = fmaxf(fz_rr, 0.0f);

    // Check if redistribution is necessary
    // When a tire lifts off the ground, the lateral load transfer distribution no longer applies
    // Same for longitudinal load transfer distribution
    // This is where this function is not that accurate and normal loads fromt strain gauges are better

    // find the tires that are on the ground
    int grounded_fl = (fz_fl > 0.0f);
    int grounded_fr = (fz_fr > 0.0f);
    int grounded_rl = (fz_rl > 0.0f);
    int grounded_rr = (fz_rr > 0.0f);

    int num_grounded = grounded_fl + grounded_fr + grounded_rl + grounded_rr;

    // Redistribution is needed only if a tire is off the ground
    if (num_grounded < 4) {

        // Correct the other tire normal loads for the contact adjustment
        // The excess load which can be distributed cannot be the static normal
        // load or the downforce - only from load transfer reactions
        float total_fz = fz_fl + fz_fr + fz_rl + fz_rr;
        float excess_load = total_fz - total_normal_load - total_downforce;

        // Sum of grounded tire loads
        float grounded_sum = grounded_fl * fz_fl + grounded_fr * fz_fr + grounded_rl * fz_rl + grounded_rr * fz_rr;

        if (grounded_sum > 0.0f) {
            // We will subtract load from tires that are contacting the ground based
            // on the ratios of their normal loads
            fz_fl = fz_fl - grounded_fl * excess_load * (fz_fl / grounded_sum);
            fz_fr = fz_fr - grounded_fr * excess_load * (fz_fr / grounded_sum);
            fz_rl = fz_rl - grounded_rl * excess_load * (fz_rl / grounded_sum);
            fz_rr = fz_rr - grounded_rr * excess_load * (fz_rr / grounded_sum);
        }
    }

    // Invert for the tire model:
    // TODO: clean up the logic
    normal_force->fl = -fz_fl;
    normal_force->fr = -fz_fr;
    normal_force->rl = -fz_rl;
    normal_force->rr = -fz_rr;
}

/*
 * Calculates slip angles on each tire.
 *   vehicle_body_t *vehicle
 *   steering_angles_t *delta
 *   slip_angles_t *alpha
 */
void __attribute__((unused)) calculate_slip_angles(vehicle_body_t *vehicle, steering_angles_t *delta, slip_angles_t *alpha) {
    alpha->fl = delta->fl - atan((vehicle->velocity_y + (WHEELBASE_FRONT * vehicle->yaw_rate))/(vehicle->velocity_x - (vehicle->yaw_rate * TRACK_WIDTH_FRONT/2)));
    alpha->fr = delta->fr - atan((vehicle->velocity_y + (WHEELBASE_FRONT * vehicle->yaw_rate))/(vehicle->velocity_x + (vehicle->yaw_rate * TRACK_WIDTH_FRONT/2)));
    alpha->rl = delta->rl - atan((vehicle->velocity_y - (WHEELBASE_REAR * vehicle->yaw_rate))/(vehicle->velocity_x - (vehicle->yaw_rate * TRACK_WIDTH_REAR/2)));
    alpha->rr = delta->rr - atan((vehicle->velocity_y - (WHEELBASE_REAR * vehicle->yaw_rate))/(vehicle->velocity_x + (vehicle->yaw_rate * TRACK_WIDTH_REAR/2)));
}

/*
* Calculates tire surface velocites. Used for slip angle determination.
*   vehicle_body_t *vehicle
*   steering_angles_t *delta
*   slip_angles_t *alpha
*   surf_vel_t *surface_velocities
*/
void __attribute__((unused)) calculate_tire_surface_velocities(vehicle_body_t *vehicle, steering_angles_t *delta, slip_angles_t *alpha, surf_vel_t *surf_vel) {
    surf_vel->fl = (vehicle->velocity_x - vehicle->yaw_rate * TRACK_WIDTH_FRONT / 2) * cos(delta->fl - alpha->fl) + (vehicle->velocity_y + vehicle->yaw_rate * WHEELBASE_FRONT) * sin(delta->fl - alpha->fl);
    surf_vel->fr = (vehicle->velocity_x + vehicle->yaw_rate * TRACK_WIDTH_FRONT / 2) * cos(delta->fr - alpha->fr) + (vehicle->velocity_y + vehicle->yaw_rate * WHEELBASE_FRONT) * sin(delta->fr - alpha->fr);
    surf_vel->rl = (vehicle->velocity_x - vehicle->yaw_rate * TRACK_WIDTH_REAR / 2) * cos(delta->rl - alpha->rl) + (vehicle->velocity_y - vehicle->yaw_rate * WHEELBASE_REAR) * sin(delta->rl - alpha->rl);
    surf_vel->rr = (vehicle->velocity_x + vehicle->yaw_rate * TRACK_WIDTH_REAR / 2) * cos(delta->rr - alpha->rr) + (vehicle->velocity_y - vehicle->yaw_rate * WHEELBASE_REAR) * sin(delta->rr - alpha->rr);
}

/*
* Calculate tire slip ratios
*   vehicle_body_t *vehicle
*   surf_vel_t *surf_vel
*   slip_ratios_t *slip_ratios
*/
void __attribute__((unused)) calculate_slip_ratios(vehicle_body_t *vehicle,  surf_vel_t *surf_vel, slip_ratios_t *slip_ratios) {
    // Note: a better approach is to calculate RE as a function of V, Fz ->
    // RE(V, Fz) for each wheel individually, and use a lookup table for RE
    // derived from the tire test data
    slip_ratios->fl = (vehicle->wheel_speeds.fl * EFFECTIVE_RADIUS / surf_vel->fl) - 1.0f;
    slip_ratios->fr = (vehicle->wheel_speeds.fr * EFFECTIVE_RADIUS / surf_vel->fr) - 1.0f;
    slip_ratios->rl = (vehicle->wheel_speeds.rl * EFFECTIVE_RADIUS / surf_vel->rl) - 1.0f;
    slip_ratios->rr = (vehicle->wheel_speeds.rr * EFFECTIVE_RADIUS / surf_vel->rr) - 1.0f;
}
