/*
 * amk_can.c
 *
 * Defines the tasks in charge of CAN bus communication
 *
 *  Created on: Feb 11, 2025
 *      Author: bre17
 */
#include "amk_can.h"
#include "vehicle_data.h"
#include "cmsis_os.h"
#include "can_db.h"

static AMKState_t state;
extern osEventFlagsId_t amkEventFlagsHandle;

#define FORALL_MOTORS(mid) for (enum MotorId mid = 0; mid < 4; mid++)

AMKMotorState_t * MotorState(enum MotorId mid) {
	switch (mid) {
	case MOTOR_FL: return &state.motor_state_fl;
	case MOTOR_FR: return &state.motor_state_fr;
	case MOTOR_RL: return &state.motor_state_rl;
	case MOTOR_RR: return &state.motor_state_rr;
	default: return NULL;
	}
}

int16_t * MotorTorqueSetpoint(enum MotorId mid) {
	switch (mid) {
	case MOTOR_FL: return &state.torqueSetpoints.front_left;
	case MOTOR_FR: return &state.torqueSetpoints.front_right;
	case MOTOR_RL: return &state.torqueSetpoints.rear_left;
	case MOTOR_RR: return &state.torqueSetpoints.rear_right;
	default: return NULL;
	}
}

AMKMotorInfo_t * MotorInfo(enum MotorId mid) {
	switch (mid) {
	case MOTOR_FL: return &state.motor_info_fl;
	case MOTOR_FR: return &state.motor_info_fr;
	case MOTOR_RL: return &state.motor_info_rl;
	case MOTOR_RR: return &state.motor_info_rr;
	default: return NULL;
	}
}

void update_vehicle_state(uint32_t timeout) {
	osMutexAcquire(vdb_inverter_lockHandle, timeout);
	VehicleData.inverter = state;
	osMutexRelease(vdb_inverter_lockHandle);
}

void motor_feedback_callback(uint32_t can_id, uint64_t message_int, void* void_motor_id) {
	enum MotorId mid = (enum MotorId) void_motor_id;
	AMKMotorInfo_t * motor_info = MotorInfo(mid);
	motor_info->motorFeedbackMessage.as_u64 = message_int;

	AMKControllerEventFlags_t flags = {0};
	switch (mid) {
	case MOTOR_FL: flags.flagBits.motor_feedback_fl_received = 1; break;
	case MOTOR_FR: flags.flagBits.motor_feedback_fr_received = 1; break;
	case MOTOR_RL: flags.flagBits.motor_feedback_rl_received = 1; break;
	case MOTOR_RR: flags.flagBits.motor_feedback_rr_received = 1; break;
	default: break;
	}
	osEventFlagsSet(amkEventFlagsHandle, flags.flagInt);
}

void initialize_motor_info(enum MotorId mid) {
	AMKMotorInfo_t * motor_info = MotorInfo(mid);
	uint32_t request_can_id, feedback_can_id;
	CAN_HandleTypeDef* canInterface;
	switch (mid) {
	case MOTOR_RL:
		request_can_id = CAN_DB_AMK_RL_MOTOR_REQUEST_ID;
		feedback_can_id = CAN_DB_AMK_RL_MOTOR_FEEDBACK_ID;
		canInterface = &hcan2;
		break;
	default:
		motor_info->isConfigured = false;
		return;
	}
	motor_info->isConfigured = true;
	motor_info->motorRequestMessageEntry = CANGetDbEntry(request_can_id);
	motor_info->motorFeedbackMessageEntry = CANGetDbEntry(feedback_can_id);
	motor_info->motorRequestMessage = (AMKMotorRequestMessage_t){0};
	motor_info->canInterface = canInterface;

	CANRegisterCallback(motor_info->motorFeedbackMessageEntry, motor_feedback_callback, (void*) mid);
}

void update_motor(enum MotorId mid);

void StartAMKTask(void *argument) {
	// Initialize state machines
	FORALL_MOTORS(mid)
		*MotorState(mid) = MOTOR_DISABLED;
	FORALL_MOTORS(mid)
		initialize_motor_info(mid);
	state.controller_state = MOTORS_DISABLED;
	update_vehicle_state(osWaitForever);


	while (1) {
		AMKControllerEventFlags_t flags;
		flags.flagInt = osEventFlagsWait(amkEventFlagsHandle, (AMKControllerEventFlags_t){.flagBits=AMK_FLAGS_ALL}.flagInt, osFlagsWaitAny, osWaitForever);

		if (state.controller_state == MOTORS_DISABLED && flags.flagBits.start_motors) {
			// Begin startup sequence
			state.controller_state = STARTING_MOTORS;
			FORALL_MOTORS(id)
				*MotorState(id) = WAITING_FOR_SYSTEM_READY;
		}

		if (flags.flagBits.motor_feedback_fl_received) {
			update_motor(MOTOR_FL);
		}
		if (flags.flagBits.motor_feedback_fr_received) {
			update_motor(MOTOR_FR);
		}
		if (flags.flagBits.motor_feedback_rl_received) {
			update_motor(MOTOR_RL);
		}
		if (flags.flagBits.motor_feedback_rr_received) {
			update_motor(MOTOR_RR);
		}

		update_vehicle_state(osWaitForever);
	}
}

void AMKSetInverterTorqueSetpoints(amkTorqueSetpoints setpoints) {
	state.torqueSetpoints = setpoints;
	update_vehicle_state(0);
}

void update_motor(enum MotorId mid) {
	AMKMotorState_t * motor_state = MotorState(mid);
	AMKMotorInfo_t * motor_info = MotorInfo(mid);
	uint16_t torque_setpoint = *MotorTorqueSetpoint(mid);
	if (!motor_info->isConfigured)
		return;
	AMKMotorFeedbackMessage_t feedback = motor_info->motorFeedbackMessage;

	switch (*motor_state) {
	case MOTOR_DISABLED: break; // Wait for start_motors event flag
	case WAITING_FOR_SYSTEM_READY:
		if (feedback.fields.SYSTEM_READY)
			*motor_state = READY_TO_SET_DC_ON;
		break;
	case READY_TO_SET_DC_ON:
		motor_info->motorRequestMessage.fields.DC_ENABLE = 1;
		*motor_state = WAITING_FOR_QUIT_DC_ON;
		break;
	case WAITING_FOR_QUIT_DC_ON:
		if (feedback.fields.QUIT_DC_ON)
			*motor_state = READY_TO_ENABLE;
		break;
	case READY_TO_ENABLE:
		motor_info->motorRequestMessage.fields.TORQUE_SETPOINT = 0;
		motor_info->motorRequestMessage.fields.NEGATIVE_TORQUE_LIMIT = 0;
		motor_info->motorRequestMessage.fields.POSITIVE_TORQUE_LIMIT = 0;
		motor_info->motorRequestMessage.fields.DRIVER_ENABLE = 1;
		motor_info->motorRequestMessage.fields.INVERTER_ENABLE = 1;
		*motor_state = WAITING_FOR_QUIT_INVERTER_ON;
		break;
	case WAITING_FOR_QUIT_INVERTER_ON:
		if (feedback.fields.QUIT_INVERTER)
			*motor_state = MOTOR_READY;
		break;

	case MOTOR_READY:
		motor_info->motorRequestMessage.fields.TORQUE_SETPOINT = torque_setpoint;
		motor_info->motorRequestMessage.fields.POSITIVE_TORQUE_LIMIT = MOTOR_POS_TORQUE_LIMIT;
		motor_info->motorRequestMessage.fields.NEGATIVE_TORQUE_LIMIT = MOTOR_NEG_TORQUE_LIMIT;
		break;

	case READY_TO_DISABLE:
		motor_info->motorRequestMessage.fields.TORQUE_SETPOINT = 0;
		motor_info->motorRequestMessage.fields.NEGATIVE_TORQUE_LIMIT = 0;
		motor_info->motorRequestMessage.fields.POSITIVE_TORQUE_LIMIT = 0;
		motor_info->motorRequestMessage.fields.DRIVER_ENABLE = 0;
		motor_info->motorRequestMessage.fields.INVERTER_ENABLE = 0;
		*motor_state = WAITING_FOR_QUIT_INVERTER_OFF;
		break;
	case WAITING_FOR_QUIT_INVERTER_OFF:
		if (!feedback.fields.QUIT_INVERTER)
			*motor_state = READY_TO_SET_DC_OFF;
		break;
	case READY_TO_SET_DC_OFF:
		motor_info->motorRequestMessage.fields.DC_ENABLE = 0;
		*motor_state = WAITING_FOR_QUIT_DC_OFF;
		break;
	case WAITING_FOR_QUIT_DC_OFF:
		if (!feedback.fields.QUIT_DC_ON)
			*motor_state = MOTOR_DISABLED;
		break;

	default: break;
	};

	// Send the updated motor settings to the inverter
	CANQueueMessageToSend(motor_info->motorRequestMessageEntry, motor_info->motorRequestMessage.as_u64, motor_info->canInterface);
}
