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

#define NUM_FAULTS 10
#define FAULT_MGMT_TASK_PERIOD 50

/* Critical Fault Indexes */
#define FAULT_INDEX_VCU_FAILURE 0

/* Non-Critical Fault Indexes */
#define FAULT_INDEX_INV_FAILURE 1 // unsure, may need shutdown loop reset
#define FAULT_INDEX_INV_COM_FAILURE 2 // unsure, may need shutdown loop reset
#define FAULT_INDEX_APPS_BPS_FAILURE 3 // motor commands 0, after 100ms
#define FAULT_INDEX_SS_FAILURE 4 // limp mode, until manual press
#define FAULT_INDEX_GPS_FAILURE 5 // limp mode, until manual press
#define FAULT_INDEX_BMS_COM_FAILURE 6 // limp mode, until manual press
#define FAULT_INDEX_GPS_COM_FAILURE 7 // limp mode, until manual press
#define FAULT_INDEX_BMS_FAILURE 8
#define FAULT_INDEX_STRAIN_GAUGE_FAILURE 9

// Dummy variables for GPS
#define GPS_HEADING_MOTION_MAX       3600     // degrees * 10
#define GPS_HEADING_MOTION_MIN       0        // degrees * 10
#define GPS_HEADING_VEHICLE_MAX      3600     // degrees * 10
#define GPS_HEADING_VEHICLE_MIN      0        // degrees * 10

#define GPS_YAW_RATE_MAX             314      // rad/s * 100
#define GPS_YAW_RATE_MIN            -314      // rad/s * 100

#define GPS_ACCEL_X_MAX              2000     // m/s^2 * 100
#define GPS_ACCEL_X_MIN             -2000     // m/s^2 * 100

#define GPS_ACCEL_Y_MAX              2000     // m/s^2 * 100
#define GPS_ACCEL_Y_MIN             -2000     // m/s^2 * 100
#define GPS_ACCEL_Y_INVERT          -1        // inversion constant

// Dummy variables for strain gauge
#define STRAIN_FL_LOAD_MAX           500      // N * 10
#define STRAIN_FL_LOAD_MIN           100      // N * 10

#define STRAIN_FR_LOAD_MAX           500      // N * 10
#define STRAIN_FR_LOAD_MIN           100      // N * 10

#define STRAIN_RL_LOAD_MAX           500      // N * 10
#define STRAIN_RL_LOAD_MIN           100      // N * 10

#define STRAIN_RR_LOAD_MAX           500      // N * 10
#define STRAIN_RR_LOAD_MIN           100      // N * 10

// Dummy variables for BMS
#define BMS_TEMP_MAX                 600      // C * 10
#define BMS_TEMP_MIN                 0        // C * 10

#define BMS_VOLTAGE_MAX              42000    // V * 1000
#define BMS_VOLTAGE_MIN              30000	  // V * 1000

#define BMS_POWER_MAX                100000	  // W
#define BMS_POWER_MIN                40000	  // W

#define BMS_SOC_MAX                  1000     // % * 10
#define BMS_SOC_MIN                  50       // % * 10

typedef union {
	struct FaultTypeBits{
		uint8_t Fault_vcu : 1;
		uint8_t Fault_inverter : 1;
		uint8_t Fault_inverter_com : 1;
		uint8_t Fault_apps_bps : 1;
		uint8_t Fault_ss : 1;
		uint8_t Fault_gps : 1;
		uint8_t Fault_bms_com : 1;
		uint8_t Fault_gps_com : 1;
		uint8_t Fault_bms : 1;
		uint8_t Fault_strain_gauge: 1;
	} faultBits;
	uint32_t faultInt;
} FaultType_t;

typedef struct {
	int16_t fl_tire_load;
	int16_t fr_tire_load;
	int16_t rl_tire_load;
	int16_t rr_tire_load;
	uint8_t plausible;
} StrainGaugeData_t;

typedef struct {
	int16_t battery_temp;
	int16_t dc_bus_voltage;
	int32_t instantaneous_power;
	uint16_t soc;
	uint8_t plausible;
} BMSData_t;

const static struct FaultTypeBits FAULTS_ALL = {1};
const static struct FaultTypeBits FAULTS_NONE = {0};

void StartFaultTask(void *argument);
void fault_callback();
void fault_check();
void apps_bps_implausibility_check(FaultType_t *fault);
void sas_implausibility_check(FaultType_t *fault);
void gps_check(FaultType_t *fault);
void inverter_check(FaultType_t *fault);
void bms_check(FaultType_t *fault);
void vcu_check(FaultType_t *fault);
void strain_gauge_check(FaultType_t *fault);
void fault_clear_flags();
void fault_flag_callback(uint8_t fault, uint8_t value);

#endif /* INC_FAULT_MGMT_H_ */
