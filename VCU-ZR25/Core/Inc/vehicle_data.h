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
  osMutexId_t apps_lock;

  BPSSensor_t bps;
  osMutexId_t bps_lock;

  SteeringAngleSensor_t sas;
  osMutexId_t sas_lock;

  AMKState_t inverter;
  osMutexId_t inverter_lock;

  VCU_State_t fsm_state;
  osMutexId_t fsm_state_lock;

  PowSupData_t powsup;
  osMutexId_t powsup_lock;
} VehicleData_t;

void initVehicleData();



#endif /* INC_VEHICLE_DATA_H_ */
