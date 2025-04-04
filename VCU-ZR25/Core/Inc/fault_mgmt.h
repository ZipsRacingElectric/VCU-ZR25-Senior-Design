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

#define NUM_FAULTS 11
#define FAULT_MGMT_TASK_PERIOD 50
/* Critical Fault Indexes */

#define FAULT_INDEX_VCU_FAILURE 0

/* Non-Critical Fault Indexes */
#define FAULT_INDEX_INV_FAILURE 1 // unsure, may need shutdown loop reset
#define FAULT_INDEX_INV_COM_FAILURE 2 // unsure, may need shutdown loop reset
#define FAULT_INDEX_APPS_BPS_FAILURE 3 // motor commands 0, after 100ms
#define FAULT_INDEX_SS_FAILURE 4 // limp mode, until manual press
#define FAULT_INDEX_GPS_FAILURE 5 // limp mode, until manual press
#define FAULT_INDEX_GNSS_FAILURE 6 // limp mode, until manual press
#define FAULT_INDEX_BMS_COM_FAILURE 7 // limp mode, until manual press
#define FAULT_INDEX_GPS_COM_FAILURE 8 // limp mode, until manual press
#define FAULT_INDEX_VIM_COM_FAILURE 9 // unsure if LED should be on, needs to update vehicle data, torque control determine calculating tire normal loads
#define FAULT_INDEX_GLV_FAILURE 10

typedef union {
	struct FaultTypeBits{
		uint8_t Fault_vcu : 1;
		uint8_t Fault_inverter : 1;
		uint8_t Fault_inverter_com : 1;
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
void apps_bps_implausibility_check(FaultType_t *fault);
void sas_implausibility_check(FaultType_t *fault);
void gps_check(FaultType_t *fault);
void gnss_check(FaultType_t *fault);
void inverter_check(FaultType_t *fault);
void glv_check(FaultType_t *fault);
void vcu_check(FaultType_t *fault);
void fault_clear_flags();
void fault_flag_callback(uint8_t fault, uint8_t value);

#endif /* INC_FAULT_MGMT_H_ */
