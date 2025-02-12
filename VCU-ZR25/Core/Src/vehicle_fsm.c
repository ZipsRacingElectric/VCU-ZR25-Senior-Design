/*
 * vehicle_fsm.c
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#include "vehicle_fsm.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

VCU_State_t currentState = VEHICLE_OFF;
/* Interrupt flags */
typedef struct {
	uint8_t GLVMS_Turned_On : 1;
	uint8_t Shutdown_Loop_Open : 1;
	uint8_t Shutdown_Loop_Open_Critical : 1;
	uint8_t External_Button_Pressed : 1;
	uint8_t Brake_Pressed : 1;
	uint8_t Start_Button_Pressed : 1;
	uint8_t Fault_Cleared : 1;
	uint8_t External_Reset_Pressed : 1;
} FSMInterruptFlags_t;

const FSMInterruptFlags_t FSM_FLAGS_ALL = {1,1,1,1,1,1,1,1};
const FSMInterruptFlags_t FSM_FLAGS_NONE = {0,0,0,0,0,0,0,0};

static osThreadId_t thread_id;

void StartFSMTask(void *argument)
{
  thread_id = osThreadGetId();
  FSMInterruptFlags_t flags = FSM_FLAGS_NONE;
  osThreadFlagsSet(thread_id, (uint32_t)flags);

  for(;;)
  {
	flags = osThreadFlagsGet();

    switch(currentState)
    {
      case VEHICLE_OFF:
        if (flags->GLVMS_Turned_On)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      case LOW_VOLTAGE_STATE:
		if (!flags->GLVMS_Turned_On)
		{
		  TransitionState(VEHICLE_OFF);
		}
		else if (!flags->Shutdown_Loop_Open && flags->External_Button_Pressed)
        {
          TransitionState(TRACTIVE_SYSTEM_ACTIVE_STATE);
        }
        break;

      case TRACTIVE_SYSTEM_ACTIVE_STATE:
    	if (!flags->GLVMS_Turned_On)
    	{
    	  TransitionState(VEHICLE_OFF);
    	}
        else if (flags->Shutdown_Loop_Open_Critical)
		{
		  TransitionState(LOCKOUT_STATE);
		}
        else if (flags->Shutdown_Loop_Open)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
    	else if (flags->Brake_Pressed && flags->Start_Button_Pressed)
        {
          TransitionState(READY_TO_DRIVE_STATE);
        }
        break;

      case READY_TO_DRIVE_STATE:
      	if (!flags->GLVMS_Turned_On)
      	{
      	  TransitionState(VEHICLE_OFF);
      	}
      	if (flags->Shutdown_Loop_Open_Critical)
		{
		  TransitionState(LOCKOUT_STATE);
		}
        else if (flags->Shutdown_Loop_Open)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      case LOCKOUT_STATE:
		if (!flags->GLVMS_Turned_On)
		{
		  TransitionState(VEHICLE_OFF);
		}
        if (flags->Fault_Cleared && flags->External_Reset_Pressed)
        {
          TransitionState(TRACTIVE_SYSTEM_ACTIVE_STATE);
        }
        break;

      default:
        break;
    }

    flags = osThreadFlagsWait(FSM_FLAGS_ALL, osFlagsWaitAny, 10);
  }
}

/* TODO: Determine Pin writes and reads for transitions and exceptions */
void TransitionState(VCU_State_t newState)
{
  currentState = newState;

  switch(newState)
  {
    case VEHICLE_OFF:
      HAL_GPIO_WritePin(GPIOA, STATUS_LED_1_Pin, GPIO_PIN_RESET);
      break;

    case LOW_VOLTAGE_STATE:
      HAL_GPIO_WritePin(GPIOA, STATUS_LED_1_Pin, GPIO_PIN_SET);
      break;

    case TRACTIVE_SYSTEM_ACTIVE_STATE:
      HAL_GPIO_WritePin(GPIOB, STATUS_LED_1_Pin, GPIO_PIN_SET);
      break;

    case READY_TO_DRIVE_STATE:
      HAL_GPIO_WritePin(GPIOA, STATUS_LED_1_Pin, GPIO_PIN_SET);
      break;

    case LOCKOUT_STATE:
      HAL_GPIO_WritePin(GPIOB, STATUS_LED_1_Pin, GPIO_PIN_RESET);
      break;

    default:
      break;
  }
}

void FSM_GPIO_Callback(uint16_t GPIO_Pin) {
  FSMInterruptFlags_t flags = FSM_FLAGS_NONE;
  if (GPIO_Pin == GLV_BATTERY_Pin)
  {
	flags->GLVMS_Turned_On = 1;
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_Pin)
  {
	flags->Shutdown_Loop_Open = 1;
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_Pin)
  {
	flags->Shutdown_Loop_Open_Critical = 1;
  }
  else if (GPIO_Pin == STATUS_LED_1_Pin)
  {
	flags->External_Button_Pressed = 1;
  }
  else if (GPIO_Pin == (BPS_FRONT_Pin | BPS_REAR_Pin))
  {
	flags->Brake_Pressed = 1;
  }
  else if (GPIO_Pin == STATUS_LED_1_Pin)
  {
	flags->Start_Button_Pressed = 1;
  }
  osThreadFlagsSet(thread_id, flags);
}

