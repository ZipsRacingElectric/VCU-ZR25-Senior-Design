/*
 * fault_mgmt.h
 *
 *  Created on: Feb 21, 2025
 *      Author: John
 */

#ifndef INC_FAULT_MGMT_H_
#define INC_FAULT_MGMT_H_

#include "cmsis_os.h"
#include "gpio.h"
#include "vehicle_data.h"

#define NUM_FAULTS 11
/* Critical Fault Indexes */
#define FAULT_INDEX_INV_FAILURE 1 // unsure, may need shutdown loop reset
#define FAULT_INDEX_INV_COM_FAILURE 2 // unsure, may need shutdown loop reset
#define FAULT_INDEX_VCU_FAILURE 3

/* Non-Critical Fault Indexes */
#define FAULT_INDEX_APPS_BPS_FAILURE 4 // motor commands 0, after 100ms
#define FAULT_INDEX_SS_FAILURE 5 // limp mode, until manual press
#define FAULT_INDEX_GPS_FAILURE 6 // limp mode, until manual press
#define FAULT_INDEX_GNSS_FAILURE 7 // limp mode, until manual press
#define FAULT_INDEX_BMS_COM_FAILURE 8 // limp mode, until manual press
#define FAULT_INDEX_GPS_COM_FAILURE 9 // limp mode, until manual press
#define FAULT_INDEX_VIM_COM_FAILURE 10 // unsure if LED should be on, needs to update vehicle data, torque control determine calculating tire normal loads
#define FAULT_INDEX_GLV_FAILURE 11

typedef union {
	struct FaultTypeBits{
		uint8_t Fault_inverter : 1;
		uint8_t Fault_inverter_com : 1;
		uint8_t Fault_vcu : 1;
		uint8_t Fault_apps_bps : 1;
		uint8_t Fault_ss : 1;
		uint8_t Fault_gps : 1;
		uint8_t Fault_gnss : 1;
		uint8_t Fault_bms : 1;
		uint8_t Fault_gps_com : 1;
		uint8_t Fault_vim_com : 1;
		uint8_t Fault_glv : 1;
	} faultBits;
	uint32_t faultInt;
} FaultType_t;

const static struct FaultTypeBits FAULTS_ALL = {1};
const static struct FaultTypeBits FAULTS_NONE = {0};

void StartFaultTask(void *argument);
void fault_callback();
void fault_check();
void apps_bps_implausibility_check(FaultType_t fault, VehicleData_t vehicle_data);
void sas_implausibility_check(FaultType_t fault, VehicleData_t vehicle_data);

#endif /* INC_FAULT_MGMT_H_ */
