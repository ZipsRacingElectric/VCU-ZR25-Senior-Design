/*
 * vehicle_fsm.c
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#include "vehicle_fsm.h"
#include "driver_sensors.h"
#include "vehicle_data.h"
#include "cooling_system.h"
#include "dashboard.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "fault_mgmt.h"

VCU_State_t currentState = VEHICLE_OFF;

static osThreadId_t thread_id;

void update_fsm_data(VCU_State_t fsm_state) {
	osMutexAcquire(vdb_fsm_state_lockHandle, osWaitForever);
	VehicleData.fsm_state = fsm_state;
	osMutexRelease(vdb_fsm_state_lockHandle);
}

void StartFsmTask(void *argument)
{
  thread_id = osThreadGetId();
  FSMInterruptFlags_t flags = {.flagBits = FSM_FLAGS_NONE};
  osThreadFlagsSet(thread_id, flags.flagInt);

  for(;;)
  {
	flags.flagInt = osThreadFlagsGet();

    switch(currentState)
    {
      case VEHICLE_OFF:
        if (flags.flagBits.GLVMS_Turned_On)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      case LOW_VOLTAGE_STATE:
		if (!flags.flagBits.GLVMS_Turned_On)
		{
		  TransitionState(VEHICLE_OFF);
		}
		else if (!flags.flagBits.Shutdown_Loop_Open && flags.flagBits.External_Button_Pressed)
        {
          TransitionState(TRACTIVE_SYSTEM_ACTIVE_STATE);
        }
        break;

      case TRACTIVE_SYSTEM_ACTIVE_STATE:
    	if (!flags.flagBits.GLVMS_Turned_On)
    	{
    	  TransitionState(VEHICLE_OFF);
    	}
        else if (flags.flagBits.Shutdown_Loop_Open | flags.flagBits.Fault_Detected)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
    	else if (flags.flagBits.Brake_Pressed && flags.flagBits.Start_Button_Pressed)
        {
          TransitionState(READY_TO_DRIVE_STATE);
        }
        break;

      case READY_TO_DRIVE_STATE:
      	if (!flags.flagBits.GLVMS_Turned_On)
      	{
      	  TransitionState(VEHICLE_OFF);
      	}
        else if (flags.flagBits.Shutdown_Loop_Open | flags.flagBits.Fault_Detected)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      default:
        break;
    }

    update_fsm_data(currentState);
    osDelay(VEHICLE_FSM_TASK_PERIOD);
  }
}

void TransitionState(VCU_State_t newState)
{
  currentState = newState;
  FSMInterruptFlags_t flags = {.flagBits = FSM_FLAGS_NONE};
  flags.flagInt = osThreadFlagsGet();

  switch(newState)
  {
    case VEHICLE_OFF:
      if (!flags.flagBits.Fault_Detected){
    	  HAL_GPIO_WritePin(GPIOB, VCU_FAULT_Pin, GPIO_PIN_RESET);
      }
      else {
    	  HAL_GPIO_WritePin(GPIOB, VCU_FAULT_Pin, GPIO_PIN_SET);
      }
      HAL_GPIO_WritePin(GPIOC, RAIL_POWER_ENABLE_5V_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_2_Pin, GPIO_PIN_RESET);
      break;

    case LOW_VOLTAGE_STATE:
	  if (flags.flagBits.Fault_Detected){
		  HAL_GPIO_WritePin(GPIOB, VCU_FAULT_Pin, GPIO_PIN_SET);
	  }
	  else {
		  HAL_GPIO_WritePin(GPIOB, VCU_FAULT_Pin, GPIO_PIN_SET);
	  }
	  HAL_GPIO_WritePin(GPIOC, RAIL_POWER_ENABLE_5V_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_2_Pin, GPIO_PIN_RESET);
      break;

    case TRACTIVE_SYSTEM_ACTIVE_STATE:
      HAL_GPIO_WritePin(GPIOB, VCU_FAULT_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, RAIL_POWER_ENABLE_5V_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_2_Pin, GPIO_PIN_SET);
      break;

    case READY_TO_DRIVE_STATE:
      HAL_GPIO_WritePin(GPIOB, VCU_FAULT_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, RAIL_POWER_ENABLE_5V_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, DEBUG_LED_2_Pin, GPIO_PIN_SET);
      CoolingSystemTurnOnLeft();
      CoolingSystemTurnOnRight();
      break;

    default:
      break;
  }
}

void FSM_GPIO_Callback(uint16_t GPIO_Pin) {
  FSMInterruptFlags_t flags = {.flagBits = FSM_FLAGS_NONE};

  if (GPIO_Pin == VCU_SHUTDOWN_LOOP_IN_Pin)
  {
	flags.flagBits.Shutdown_Loop_Open = !HAL_GPIO_ReadPin(
			VCU_SHUTDOWN_LOOP_IN_GPIO_Port,
			VCU_SHUTDOWN_LOOP_IN_Pin
			);
	if(flags.flagBits.Shutdown_Loop_Open == 0)
		{
			osThreadFlagsClear(1 << FLAG_INDEX_SHUTDOWN_LOOP_OPEN);
		}
	else {
		DashboardFaultCallback(1);
	}
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_RESET_Pin)
  {
	flags.flagBits.External_Button_Pressed = HAL_GPIO_ReadPin(
			VCU_SHUTDOWN_LOOP_RESET_GPIO_Port,
			VCU_SHUTDOWN_LOOP_RESET_Pin
			);
	if(flags.flagBits.External_Button_Pressed == 0)
			{
				osThreadFlagsClear(1 << FLAG_INDEX_EXTERNAL_BUTTON_PRESSED);
			}
  }
  else if (GPIO_Pin == START_BUTTON_Pin)
  {
	flags.flagBits.Start_Button_Pressed = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
	if(flags.flagBits.Start_Button_Pressed == 0)
				{
					osThreadFlagsClear(1 << FLAG_INDEX_START_BUTTON_PRESSED);
				}
  }
  else if (GPIO_Pin == DASH_INPUT_2_Pin){
	  DashboardDRSToggleCallback(GPIO_Pin);
  }
  else if ((GPIO_Pin == DASH_INPUT_3_Pin) | (GPIO_Pin == DASH_INPUT_4_Pin)){
	  DashboardTorqueLimitCallback(GPIO_Pin);
  }
  osThreadFlagsSet(thread_id, flags.flagInt);
}

void fsm_flag_callback(uint8_t flag, uint8_t value){
	FSMInterruptFlags_t flags = {.flagBits = FSM_FLAGS_NONE};

    if (value){
    	flags.flagInt |= (1 << flag);
    	if(flag == FLAG_INDEX_FAULT_DETECTED){
    		DashboardFaultCallback(value);
    	}
    }
    else{
    	osThreadFlagsClear(1 << flag);
    }

    osThreadFlagsSet(thread_id, flags.flagInt);
}


