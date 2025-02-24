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

/* Non-Critical Fault Indexes */
#define FAULT_INDEX_IMPLAUSIBILITY 0
#define FAULT_INDEX_APPS_FAILURE 1
#define FAULT_INDEX_BPS_FAILURE 2
#define FAULT_INDEX_SSA_FAILURE 3
#define FAULT_INDEX_GNSS_FAILURE 4
#define FAULT_INDEX_INV_FAILURE 5
#define FAULT_INDEX_INV_COM_FAILURE 6
#define FAULT_INDEX_DASH_COM_FAILURE 7
#define FAULT_INDEX_BMS_FAILURE 8
#define FAULT_INDEX_BMS_COM_FAILURE 9
#define FAULT_INDEX_GLV_LOW_FAILURE 10

static const osThreadAttr_t faultTask_attributes = {
  .name = "faultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal
};

void StartFaultTask(void *argument);
void fault_callback();
void fault_check();

#endif /* INC_FAULT_MGMT_H_ */
