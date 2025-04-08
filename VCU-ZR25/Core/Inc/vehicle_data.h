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

#include "fault_mgmt.h"
#include "driver_sensors.h"
#include "amk_can.h"
#include "vehicle_fsm.h"
#include "power_supply.h"
#include "cooling_system.h"
#include "dashboard.h"
#include "torque_ctrl.h"
#include "can_sensors.h"

extern osMutexId_t vdb_apps_lockHandle;
extern osMutexId_t vdb_bps_front_lockHandle;
extern osMutexId_t vdb_bps_rear_lockHandle;
extern osMutexId_t vdb_sas_lockHandle;
extern osMutexId_t vdb_inverter_lockHandle;
extern osMutexId_t vdb_fsm_state_lockHandle;
extern osMutexId_t vdb_powsup_lockHandle;
extern osMutexId_t vdb_cooling_lockHandle;
extern osMutexId_t vdb_dashboard_lockHandle;
extern osMutexId_t vdb_torquectrl_lockHandle;
extern osMutexId_t vdb_faulttask_lockHandle;
extern osMutexId_t vdb_gps_lockHandle;

typedef struct {
  APPSSensor_t apps;
  BPSSensor_t bps_front;
  BPSSensor_t bps_rear;
  SteeringAngleSensor_t sas;
  AMKState_t inverter;
  VCU_State_t fsm_state;
  PowSupData_t powsup;
  DashboardData_t dashboard;
  TorqueCtrlData_t torquectrl;
  GPSState_t gps;
  FaultType_t faultmgmt;
} VehicleData_t;

extern VehicleData_t VehicleData;

void initVehicleData();


#endif /* INC_VEHICLE_DATA_H_ */
