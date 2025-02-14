/*
 * vehicle_state.h
 *
 * The Big Struct (tm) that tracks vehicle state
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#ifndef INC_VEHICLE_DATA_H_
#define INC_VEHICLE_DATA_H_

#include "driver_sensors.h"
#include "amk_can.h"
#include "vehicle_fsm.h"
#include "power_supply.h"

typedef struct {
  APPSSensor_t apps;
  BPSSensor_t bps;
  SteeringAngleSensor_t sas;
  AMKState_t inverter;
  VCU_State_t fsm_state;
  PowSupData_t powsup;
} VehicleData_t;

#endif /* INC_VEHICLE_DATA_H_ */
