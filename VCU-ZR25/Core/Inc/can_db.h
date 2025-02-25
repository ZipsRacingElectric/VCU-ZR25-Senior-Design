/*
 * can_db.h
 *
 *  Created on: Feb 21, 2025
 *      Author: bre17
 */

#ifndef INC_CAN_DB_H_
#define INC_CAN_DB_H_

#include "stdint.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

// Each CANDatabaseMessage_t represents the contents of a message
// from/to a certain can_id. The signals are a list of bit fields
// which the message may be split into.

typedef struct {
	const char *name;
	int bitpos;
	uint64_t bitmask;
} CANDatabaseSignal_t;

typedef struct {
	const char *name;
	uint32_t can_id;
	int num_signals;
	CANDatabaseSignal_t *signals;
} CANDatabaseMessage_t;

typedef void (*CANCallback_t)(uint32_t can_id, uint64_t messageContents, void* custom);

typedef struct {
	int message_count;
	CANDatabaseMessage_t *messages; // sorted by can_id
	uint64_t *message_contents;
	bool *message_contents_valid;
	osMutexId_t message_contents_lock;
	CANCallback_t *callbacks;
	void **callback_payloads;
} CANDatabase_t;

extern CANDatabase_t can_db;

void initCANDatabase();

typedef int CANDatabaseEntryId;

// Returns -1 if entry does not exist
CANDatabaseEntryId CANGetDbEntry(uint32_t can_id);

// Must acquire can_db.message_contents_lock first.
// Returns false if message not valid.
bool CANGetMessageContents(CANDatabaseEntryId entry_id, uint64_t *out);

// Must acquire can_db.message_contents_lock first.
void CANSetMessageContents(CANDatabaseEntryId entry_id, uint64_t contents);

// Returns true once message is successfully queued.
// Does not need the can_db lock to be acquired.
// Does not set the contents of the message in the db.
bool CANQueueMessageToSend(CANDatabaseEntryId entry_id, uint64_t contents);

// Must acquire can_db.message_contents_lock first.
// Have a function be called whenever a message with a given id is received.
// Returns false if another callback has already been registered and was overwritten.
bool CANRegisterCallback(CANDatabaseEntryId entry_id, CANCallback_t callback, void *custom_argument);

// Returns true if message recognized by CAN DB
bool CANIRQRxHandler(CAN_RxHeaderTypeDef *header, uint8_t rx_data[8]);

const static osThreadAttr_t can_task_attrs = {
	.name = "CAN_DB_Task",
	.stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

void StartCANDatabaseTask(void* _argument);

#endif /* INC_CAN_DB_H_ */
