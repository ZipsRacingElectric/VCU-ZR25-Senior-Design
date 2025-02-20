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
typedef union {
	struct FSMInterruptFlagBits{
		uint8_t GLVMS_Turned_On : 1;
		uint8_t Shutdown_Loop_Open : 1;
		uint8_t External_Button_Pressed : 1;
		uint8_t Brake_Pressed : 1;
		uint8_t Start_Button_Pressed : 1;
		uint8_t Fault_Cleared : 1;
		uint8_t External_Reset_Pressed : 1;
	} flagBits;
	uint32_t flagInt;
} FSMInterruptFlags_t;

const struct FSMInterruptFlagBits FSM_FLAGS_ALL = {1,1,1,1,1,1,1};
const struct FSMInterruptFlagBits FSM_FLAGS_NONE = {0,0,0,0,0,0,0};

static osThreadId_t thread_id;

void StartFSMTask(void *argument)
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
        else if (flags.flagBits.Shutdown_Loop_Open)
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
        else if (flags.flagBits.Shutdown_Loop_Open)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      default:
        break;
    }

    const FSMInterruptFlags_t mask = {.flagBits = FSM_FLAGS_ALL};
    flags.flagInt = osThreadFlagsWait(mask.flagInt, osFlagsWaitAny, 10);
  }
}

/* TODO: Determine Pin writes and reads for transitions and exceptions */
void TransitionState(VCU_State_t newState)
{
  currentState = newState;

  switch(newState)
  {
    case VEHICLE_OFF:
      HAL_GPIO_WritePin(GPIOA, DEBUG_LED_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, DEBUG_LED_2_Pin, GPIO_PIN_RESET);
      break;

    case LOW_VOLTAGE_STATE:
      HAL_GPIO_WritePin(GPIOA, DEBUG_LED_1_Pin, GPIO_PIN_SET);
      break;

    case TRACTIVE_SYSTEM_ACTIVE_STATE:
      HAL_GPIO_WritePin(GPIOB, DEBUG_LED_2_Pin, GPIO_PIN_SET);
      break;

    case READY_TO_DRIVE_STATE:
      HAL_GPIO_WritePin(GPIOA, DEBUG_LED_1_Pin, GPIO_PIN_RESET);
      break;

    default:
      break;
  }
}

/* TODO: Take out BPS as they're not GPIO, tell fsm when reading them */
/* there's HAL_ADC_ConvCpltCallback in hal_adc.c line 1580 that should be defined here */
void FSM_GPIO_Callback(uint16_t GPIO_Pin) {
  FSMInterruptFlags_t flags = {.flagBits = osThreadFlagsGet()};
  if (GPIO_Pin == GLV_BATTERY_Pin)
  {
	flags.flagBits.GLVMS_Turned_On = 1;
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_IN_Pin)
  {
	flags.flagBits.Shutdown_Loop_Open = 1;
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_RESET_Pin)
  {
	flags.flagBits.External_Button_Pressed = 1;
  }
  else if (GPIO_Pin == START_BUTTON_Pin)
  {
	flags.flagBits.Start_Button_Pressed = 1;
  }
  osThreadFlagsSet(thread_id, flags.flagInt);
}

/* TODO: this is a fleshout, need to make functional */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    FSMInterruptFlags_t flags = {.flagInt osThreadFlagsGet()};

    if (hadc == &BPS_FRONT) {
        uint32_t bps_front_value = HAL_ADC_GetValue(&BPS_FRONT);
        if (bps_front_value > BRAKE_PRESSURE_THRESHOLD) {
            flags.flagBits.Brake_Pressed = 1;
        }
    }

    if (hadc == &BPS_REAR) {
        uint32_t bps_rear_value = HAL_ADC_GetValue(&BPS_REAR);
        if (bps_rear_value > BRAKE_PRESSURE_THRESHOLD) {
            flags.flagBits.Brake_Pressed = 1;
        }
    }

    osThreadFlagsSet(thread_id, flags.flagInt);
}


