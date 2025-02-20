/*
 * vehicle_fsm.c
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */

#include "vehicle_fsm.h"
#include "driver_sensors.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#define FLAG_INDEX_GLVMS_TURNED_ON 0
#define FLAG_INDEX_SHUTDOWN_LOOP_OPEN 1
#define FLAG_INDEX_EXTERNAL_BUTTON_PRESSED 2
#define FLAG_INDEX_BRAKE_PRESSED 3
#define FLAG_INDEX_START_BUTTON_PRESSED 4
#define FLAG_INDEX_FAULT_DETECTED 5
#define FLAG_INDEX_EXTERNAL_RESET_PRESSED 6

VCU_State_t currentState = VEHICLE_OFF;
/* Interrupt flags */
/* TODO: Handle fault_detected logic and writing vcu_fault low */
typedef union {
	struct FSMInterruptFlagBits{
		uint8_t GLVMS_Turned_On : 1;
		uint8_t Shutdown_Loop_Open : 1;
		uint8_t External_Button_Pressed : 1;
		uint8_t Brake_Pressed : 1;
		uint8_t Start_Button_Pressed : 1;
		uint8_t Fault_Detected : 1;
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

    const FSMInterruptFlags_t mask = {.flagBits = FSM_FLAGS_ALL};
    flags.flagInt = osThreadFlagsWait(mask.flagInt, osFlagsWaitAny, 10);
  }
}

/* TODO: Determine Pin writes */
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

void FSM_GPIO_Callback(uint16_t GPIO_Pin) {
  FSMInterruptFlags_t flags = {.flagBits = FSM_FLAGS_NONE};
  if (GPIO_Pin == GLV_BATTERY_Pin)
  {
	flags.flagBits.GLVMS_Turned_On = HAL_GPIO_ReadPin(GLV_BATTERY_GPIO_Port, GLV_BATTERY_Pin);
	if(flags.flagBits.GLVMS_Turned_On == 0)
	{
		osThreadFlagsClear(1 << FLAG_INDEX_GLVMS_TURNED_ON);
	}
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_IN_Pin)
  {
	flags.flagBits.Shutdown_Loop_Open = !HAL_GPIO_ReadPin(
			VCU_SHUTDOWN_LOOP_IN_GPIO_Port,
			VCU_SHUTDOWN_LOOP_IN_Pin
			);
	if(flags.flagBits.Shutdown_Loop_Open == 0)
		{
			osThreadFlagsClear(1 << FLAG_INDEX_SHUTDOWN_LOOP_OPEN);
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
  osThreadFlagsSet(thread_id, flags.flagInt);
}

/* TODO: Just made it as 50% for now, should be changed based on motor torque issue, maybe removed */
/* IDK how to do this yet so I'm just commenting it out and will address it later
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	FSMInterruptFlags_t flags = {.flagBits = FSM_FLAGS_NONE};

    if (hadc->Channel == ADC_CHANNEL_2) {
        if (HAL_ADC_GetValue(BPS_FRONT_Pin) > BPS_MAX_VOLTAGE / 2) {
            flags.flagBits.Brake_Pressed = 1;
        }
    }

    if (hadc->Channel == ADC_CHANNEL_3) {
        if (HAL_ADC_GetValue(BPS_REAR_Pin) > BPS_MAX_VOLTAGE / 2) {
            flags.flagBits.Brake_Pressed = 1;
        }
    }

    osThreadFlagsSet(thread_id, flags.flagInt);
} */


