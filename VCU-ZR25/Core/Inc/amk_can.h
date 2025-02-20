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

// Send message 1: Motor feedback
struct AMKSendMessage1 {
	uint16_t status_word;             // Index 3, see AMKStatusWord
	int16_t actual_torque;            // Index 19
	int32_t actual_speed;             // Index 20
};

// Send message 2: Power consumption
struct AMKSendMessage2 {
	uint16_t dc_bus_voltage;          // Index 32836
	int16_t torque_current_feedback;  // Index 32834
	uint32_t actual_power;            // Index 33100
};

// Send message 3: Temperatures
struct AMKSendMessage3 {
	int16_t internal_temp;            // Index 33116
	int16_t external_temp;            // Index 33117
	uint16_t motor_temp;              // Index 34166
	int16_t igbt_temp;                // Index 27
};

// Send message 4: Errors 1
struct AMKSendMessage4 {
	uint32_t diagnostic;              // Index 21
	uint32_t error_info_1;            // Index 22
};

// Send message 5: Errors 2
struct AMKSendMessage5 {
	uint32_t error_info_2;            // Index 23
	uint32_t error_info_3;            // Index 24
};

// Receive message 1: Motor request
struct AMKRecvMessage1 {
	uint16_t control_word;            // Index 4, see AMKControlWord
	int16_t torque_setpoint;          // Index 17
	int16_t torque_limit_pos;         // Index 13
	int16_t torque_limit_neg;         // Index 14
};

// Update frequencies (Hz)
#define AMKSendMessage1_Freq 200
#define AMKSendMessage2_Freq 200
#define AMKSendMessage3_Freq 10
#define AMKSendMessage4_Freq 10
#define AMKSendMessage5_Freq 10
#define AMKRecvMessage1_Freq 200


struct AMKControlWord {
	uint8_t reserve1 : 8;
	uint8_t inverter_on : 1;
	uint8_t dc_on : 1;
	uint8_t enable : 1;
	uint8_t error_reset : 1;
	uint8_t reserve2 : 4;
};

struct AMKStatusWord {
	uint8_t reserve : 8;
	uint8_t system_ready : 1;
	uint8_t error : 1;
	uint8_t warn : 1;
	uint8_t quit_dc_on : 1;
	uint8_t dc_on : 1;
	uint8_t quit_inverter_on : 1;
	uint8_t inverter_on : 1;
	uint8_t derating : 1;
};

// Vehicle state module
typedef struct {
	// SM1
	uint16_t status_word;             // Index 3, see AMKStatusWord
	int16_t actual_torque;            // Index 19
	int32_t actual_speed;             // Index 20
	// SM2
	uint16_t dc_bus_voltage;          // Index 32836
	int16_t torque_current_feedback;  // Index 32834
	uint32_t actual_power;            // Index 33100
	// SM3
	int16_t internal_temp;            // Index 33116
	int16_t external_temp;            // Index 33117
	uint16_t motor_temp;              // Index 34166
	int16_t igbt_temp;                // Index 27
	// SM4
	uint32_t diagnostic;              // Index 21
	uint32_t error_info_1;            // Index 22
	// SM5
	uint32_t error_info_2;            // Index 23
	uint32_t error_info_3;            // Index 24
} AMKState_t;

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

void StartAMKTask(void *argument);

void AMKSetInverterTorqueSetpoints(amkTorqueSetpoints setpoints);
void AMKCANInterruptCallback();

#endif /* INC_AMK_CAN_H_ */
