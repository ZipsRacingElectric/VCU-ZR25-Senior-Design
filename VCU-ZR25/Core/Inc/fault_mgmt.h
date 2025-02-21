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

#define FAULT_INDEX_IMPLAUSIBILITY 0

typedef union {
	struct FaultTypeBits{
		uint8_t Fault_implausibility : 1;
	} faultBits;
	uint32_t faultInt;
} FaultType_t;

const static struct FaultTypeBits FAULTS_ALL = {1};
const static struct FaultTypeBits FAULTS_NONE = {0};

static const osThreadAttr_t faultTask_attributes = {
  .name = "faultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal
};

void StartFaultTask(void *argument);
void fsm_fault_callback();
void fault_check();

#endif /* INC_FAULT_MGMT_H_ */
