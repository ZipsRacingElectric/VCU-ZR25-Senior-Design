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
#include "can_db.h"
#include "can_messages.h"
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

typedef struct {
	int16_t front_left;
	int16_t front_right;
	int16_t rear_left;
	int16_t rear_right;
} amkTorqueSetpoints;

typedef union {
	struct AMKControllerEventFlagBits {
		uint8_t start_motors : 1;
		uint8_t change_setpoints : 1;
		uint8_t stop_motors : 1;
		uint8_t motor_feedback_fl_received : 1;
		uint8_t motor_feedback_fr_received : 1;
		uint8_t motor_feedback_rl_received : 1;
		uint8_t motor_feedback_rr_received : 1;
	} flagBits;
	uint32_t flagInt;
} AMKControllerEventFlags_t;

typedef enum {
	MOTOR_DISABLED,

	// power on sequence
	WAITING_FOR_SYSTEM_READY,
	READY_TO_SET_DC_ON,
	WAITING_FOR_QUIT_DC_ON,
	READY_TO_ENABLE,
	WAITING_FOR_QUIT_INVERTER_ON,

	MOTOR_READY,

	// power off sequence
	READY_TO_DISABLE,
	WAITING_FOR_QUIT_INVERTER_OFF,
	READY_TO_SET_DC_OFF,
	WAITING_FOR_QUIT_DC_OFF,
} AMKMotorState_t;

// This should be identical to the CANMessage_AMK_**_MOTOR_REQUEST structs
typedef union {
    uint64_t as_u64;
    struct PACKED {
        uint64_t _reserved0 : 8;
        uint64_t INVERTER_ENABLE : 1;
        uint64_t DC_ENABLE : 1;
        uint64_t DRIVER_ENABLE : 1;
        uint64_t ERROR_RESET : 1;
        uint64_t _reserved12 : 4;
        uint64_t TORQUE_SETPOINT : 16;
        uint64_t POSITIVE_TORQUE_LIMIT : 16;
        uint64_t NEGATIVE_TORQUE_LIMIT : 16;
    } fields;
} AMKMotorRequestMessage_t;

// This should be identical to the CANMessage_AMK_**_MOTOR_FEEDBACK structs
typedef union {
    uint64_t as_u64;
    struct PACKED {
        uint64_t _reserved0 : 8;
        uint64_t SYSTEM_READY : 1;
        uint64_t ERROR : 1;
        uint64_t WARNING : 1;
        uint64_t QUIT_DC_ON : 1;
        uint64_t DC_ON : 1;
        uint64_t QUIT_INVERTER : 1;
        uint64_t INVERTER_ON : 1;
        uint64_t DERATING : 1;
        uint64_t ACTUAL_TORQUE_VALUE : 16;
        uint64_t ACTUAL_SPEED_VALUE : 32;
    } fields;
} AMKMotorFeedbackMessage_t;

typedef struct {
	bool isConfigured;
	CANDatabaseEntryId motorRequestMessageEntry;
	CANDatabaseEntryId motorFeedbackMessageEntry;
	AMKMotorRequestMessage_t motorRequestMessage;
	AMKMotorFeedbackMessage_t motorFeedbackMessage;
} AMKMotorInfo_t;

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

	AMKMotorInfo_t motor_info_fl;
	AMKMotorInfo_t motor_info_fr;
	AMKMotorInfo_t motor_info_rl;
	AMKMotorInfo_t motor_info_rr;

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
