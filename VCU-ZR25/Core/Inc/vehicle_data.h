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
#include "cooling_system.h"
#include "dashboard.h"
#include "torque_ctrl.h"

typedef struct {
  APPSSensor_t apps;
  osMutexId_t apps_lock;

  BPSSensor_t bps_front;
  osMutexId_t bps_front_lock;

  BPSSensor_t bps_rear;
  osMutexId_t bps_rear_lock;

  SteeringAngleSensor_t sas;
  osMutexId_t sas_lock;

  AMKState_t inverter;
  osMutexId_t inverter_lock;

  VCU_State_t fsm_state;
  osMutexId_t fsm_state_lock;

  PowSupData_t powsup;
  osMutexId_t powsup_lock;

  CoolingData_t cooling;
  osMutexId_t cooling_lock;

  DashboardData_t dashboard;
  osMutexId_t dashboard_lock;

  TorqueCtrlData_t torquectrl;
  osMutexId_t torquectrl_lock;
} VehicleData_t;

extern VehicleData_t VehicleData;

void initVehicleData();
VehicleData_t get_vehicle_data();


#endif /* INC_VEHICLE_DATA_H_ */
