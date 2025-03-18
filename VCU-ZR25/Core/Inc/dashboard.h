/*
 * dashboard.h
 *
 *  Created on: Feb 22, 2025
 *      Author: jmw398
 */

#ifndef INC_DASHBOARD_H_
#define INC_DASHBOARD_H_

#include "cmsis_os.h"

#define DASHBOARD_TASK_PERIOD 50

typedef struct {
  bool DRSState;
} DashboardData_t;

void StartDashboardTask(void *argument);
void DashboardFaultCallback(uint8_t value);
void DashboardDRSToggleCallback(uint16_t GPIO_Pin);
void DashboardTorqueLimitCallback(uint16_t GPIO_Pin);

#endif /* INC_DASHBOARD_H_ */
