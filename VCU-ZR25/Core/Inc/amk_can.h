/*
 * amk_e1291.h
 *
 * Driver headers for the AMKmotion RACING KIT 4WD (E1291)
 *
 * Check to make sure this is up-to-date with the CAN configuration in ZipsRacingElectric/AMK-2025.
 * Refer to AMKmotion RACING KIT 4WD technical document.
 *
 *  Created on: Feb 4, 2025
 *      Author: bre17
 */

#ifndef INC_AMK_CAN_H_
#define INC_AMK_CAN_H_

#include "cmsis_os.h"
#include <stdint.h>

enum AMKCanIdBase {
	AMKCanIdBase_RearLeft = 0x200,
	AMKCanIdBase_RearRight = 0x201,
	AMKCanIdBase_FrontLeft = 0x202,
	AMKCanIdBase_FrontRight = 0x203
};

enum AMKCanIdOffset {
	AMKCanIdOffset_SendMessage1 = 0x004, // AMKSendMessage1
	AMKCanIdOffset_SendMessage2 = 0x008, // AMKSendMessage2
	AMKCanIdOffset_SendMessage3 = 0x300, // AMKSendMessage3
	AMKCanIdOffset_SendMessage4 = 0x304, // AMKSendMessage4
	AMKCanIdOffset_SendMessage5 = 0x308, // AMKSendMessage5
	AMKCanIdOffset_RecvMessage1 = 0x000, // AMKRecvMessage1
};


// Update frequencies (Hz)
#define AMKSendMessage1_Freq 200
#define AMKSendMessage2_Freq 200
#define AMKSendMessage3_Freq 10
#define AMKSendMessage4_Freq 10
#define AMKSendMessage5_Freq 10
#define AMKRecvMessage1_Freq 200


// AMK Task interface
static const osThreadAttr_t amkTask_attributes = {
  .name = "amkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime
};

typedef struct {
	int16_t front_left;
	int16_t front_right;
	int16_t rear_left;
	int16_t rear_right;
} amkTorqueSetpoints;

typedef union {
	struct AMKControllerEventFlagBits {
		uint8_t Start_Motors;
	} flagBits;
	uint32_t flagInt;
} AMKControllerEventFlags_t;

typedef enum {
	WAITING_FOR_SYSTEM_READY,
	WAITING_FOR_QUIT_DC_ON,
	WAITING_FOR_QUIT_INVERTER_ON,
	MOTOR_READY,
} AMKMotorState_t;

typedef enum {
	MOTORS_DISABLED,
	STARTING_MOTORS,
	MOTORS_READY,
	STOPPING_MOTORS
} AMKSequenceState_t;

typedef struct {
	int16_t torqueLimitPos;
	int16_t torqueLimitNeg;

	amkTorqueSetpoints torqueSetpoints;

	AMKMotorState_t motor_state_fl;
	AMKMotorState_t motor_state_fr;
	AMKMotorState_t motor_state_rl;
	AMKMotorState_t motor_state_rr;
	AMKSequenceState_t controller_state;
} AMKState_t;

const static struct AMKControllerEventFlagBits AMK_FLAGS_ALL = {1};
const static struct AMKControllerEventFlagBits AMK_FLAGS_NONE = {0};

void StartAMKTask(void *argument);

void AMKSetInverterTorqueSetpoints(amkTorqueSetpoints setpoints);
void AMKCANInterruptCallback();

#endif /* INC_AMK_CAN_H_ */
