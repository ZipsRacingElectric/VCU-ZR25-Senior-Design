/*
 * can_db.c
 *
 *  Created on: Feb 22, 2025
 *      Author: tetra
 */

#include "can_db.h"
#include "stdbool.h"
#include "can_messages.h"
#include "stm32f4xx_hal_can.h"

CANDatabaseMessage_t messages[CAN_DB_MESSAGE_COUNT];
uint64_t message_contents[CAN_DB_MESSAGE_COUNT];
bool message_contents_valid[CAN_DB_MESSAGE_COUNT];
CANCallback_t callbacks[CAN_DB_MESSAGE_COUNT];
void* callback_payloads[CAN_DB_MESSAGE_COUNT];
CANDatabase_t can_db = {
	.message_count = CAN_DB_MESSAGE_COUNT,
	.messages = messages,
	.message_contents = message_contents,
	.message_contents_valid = message_contents_valid,
	.callbacks = callbacks,
	.callback_payloads = callback_payloads
};
osThreadId_t can_task_id;

typedef union {
	uint32_t flagInt;
	struct {
		bool PENDING_RX : 1;
		bool PENDING_TX : 1;
	} flagBits;
} CanDbFlags_t;

static const CanDbFlags_t CAN_DB_FLAGS_ALL = {.flagInt = -1};
static const CanDbFlags_t CAN_DB_FLAGS_NONE = {.flagInt = 0};
static const CanDbFlags_t CAN_DB_FLAGS_PENDING_RX = {.flagBits = {.PENDING_RX=1}};
static const CanDbFlags_t CAN_DB_FLAGS_PENDING_TX = {.flagBits = {.PENDING_TX=1}};

struct queue {
	int size;
	struct queued_message {
		int message_idx;
		uint64_t message_contents;
	} *messages;
	int start;
	int end;
};

#define CAN_SENDING_QUEUE_SIZE 16
struct queued_message sending_queue_array[CAN_SENDING_QUEUE_SIZE];
struct queue sending_queue = {
	.size = CAN_SENDING_QUEUE_SIZE,
	.messages = sending_queue_array,
	.start = 0,
	.end = 0
};


#define CAN_RECEIVING_QUEUE_SIZE 256
struct queued_message receiving_queue_array[CAN_RECEIVING_QUEUE_SIZE];
struct queue receiving_queue = {
	.size = CAN_RECEIVING_QUEUE_SIZE,
	.messages = receiving_queue_array,
	.start = 0,
	.end = 0
};

bool queueMessagePush(struct queue *queue, struct queued_message message) {
	bool interrupts_enabled = __get_PRIMASK() == 0;
	__disable_irq(); // Critical section -- do not interrupt

	int nextEnd = queue->end + 1;
	// wrap around
	while (nextEnd >= queue->size)
		nextEnd -= queue->size;

	if (nextEnd == queue->start) {
		if (interrupts_enabled) __enable_irq();
		return false; // queue full
	}

	queue->messages[queue->end] = message;
	queue->end = nextEnd;
	if (interrupts_enabled) __enable_irq();
	return true;
}

bool queueMessagePop(struct queue *queue, struct queued_message *message_out) {
	bool interrupts_enabled = __get_PRIMASK() == 0;
	__disable_irq(); // Critical section -- do not interrupt

	if (queue->start == queue->end) {
		if (interrupts_enabled) __enable_irq();
		return false; // queue empty
	}

	int nextStart = queue->start + 1;
	// wrap around
	while (nextStart >= queue->size)
		nextStart -= queue->size;

	*message_out = queue->messages[queue->start];
	queue->start = nextStart;
	if (interrupts_enabled) __enable_irq();
	return true;
}


osMutexAttr_t canDbMutexAttrs = {
	.name = "CANDatabaseContentsLock",
	.attr_bits = osMutexRobust,
};

void swapMessageEntries(int idx_a, int idx_b) {
	if (idx_a < 0 || idx_a >= CAN_DB_MESSAGE_COUNT) return;
	if (idx_b < 0 || idx_b >= CAN_DB_MESSAGE_COUNT) return;
	if (idx_a == idx_b) return;
	CANDatabaseMessage_t temp = messages[idx_b];
	messages[idx_b] = messages[idx_a];
	messages[idx_a] = temp;
}

void quicksortMessages(int start, int end) {
	int length = end-start;
	if (length <= 1) return; // arrays of length <=1 are already sorted
	int pivot = start + length/2; // pivot in middle
	uint32_t pivot_id = messages[pivot].can_id;

	// Bring pivot to start
	swapMessageEntries(0, pivot);
	pivot = 0;

	int i = 1;
	uint32_t current_id;
	while (i<end) {
		current_id = messages[i].can_id;
		if (current_id < pivot_id) {
			// Advance the pivot by one, place the current message behind the pivot
			if (i == pivot+1) {
				// Edge case
				swapMessageEntries(pivot, i);
				pivot = i;
			} else {
				// General case
				swapMessageEntries(pivot, pivot+1);
				swapMessageEntries(pivot, i);
				pivot = pivot+1;
			}
		}
		i++;
	}

	quicksortMessages(start, pivot);
	quicksortMessages(pivot+1, end);
}

void initCANDatabase() {
	// zero-init message contents & callbacks
	for (int i=0; i<CAN_DB_MESSAGE_COUNT; i++) {
		can_db.message_contents[i] = 0;
		can_db.message_contents_valid[i] = false;
		can_db.messages[i] = can_db_messages[i];
		can_db.callbacks[i] = NULL;
		can_db.callback_payloads[i] = NULL;
	}
	// sort message entries by can_id to enable binary searching
	quicksortMessages(0, CAN_DB_MESSAGE_COUNT);
	// initialize mutexes
	can_db.message_contents_lock = osMutexNew(&canDbMutexAttrs);
}

// Returns -1 if message is not in database
int searchForMessageId(uint32_t target_id) {
	int start = 0;
	int end = can_db.message_count;
	while (start<end) {
		int length = end-start;
		int midpoint = start + length/2;
		uint32_t midpoint_id = can_db.messages[midpoint].can_id;
		if (midpoint_id == target_id) {
			return midpoint;
		} else if (midpoint_id < target_id) {
			start = midpoint+1;
		} else { // midpoint_id > target_id
			end = midpoint;
		}
	}
	return -1;
}

// Returns -1 if entry does not exist
CANDatabaseEntryId CANGetDbEntry(uint32_t can_id) {
	return searchForMessageId(can_id);
}

// Must acquire can_db.message_contents_lock first.
// Returns false if message not valid or is not in database.
bool CANGetMessageContents(CANDatabaseEntryId index, uint64_t *out) {
	if (index<0) return false;
	*out = can_db.message_contents[index];
	return can_db.message_contents_valid[index];
}

// Must acquire can_db.message_contents_lock first.
void CANSetMessageContents(CANDatabaseEntryId index, uint64_t contents) {
	if (index<0) return;
	can_db.message_contents[index] = contents;
	can_db.message_contents_valid[index] = true;
}

// Returns true if has been successfully queued to send
bool CANQueueMessageToSend(CANDatabaseEntryId index, uint64_t contents) {
	struct queued_message message = {
		.message_contents = contents,
		.message_idx = index
	};
	bool success = queueMessagePush(&sending_queue, message);
	osThreadFlagsSet(can_task_id, CAN_DB_FLAGS_PENDING_TX.flagInt);
	return success;
}


// Must acquire can_db.message_contents_lock first.
// Have a function be called whenever a message with a given id is received.
// Returns false if another callback has already been registered and was overwritten.
bool CANRegisterCallback(CANDatabaseEntryId index, CANCallback_t callback, void* payload) {
	bool callback_exists = can_db.callbacks[index] != NULL;
	can_db.callbacks[index] = callback;
	can_db.callback_payloads[index] = payload;
	return !callback_exists;
}

bool CANIRQRxHandler(CAN_RxHeaderTypeDef *header, uint8_t rx_data[8]) {
	uint32_t can_id;
	if (header->IDE == CAN_ID_EXT) {
		can_id = header->ExtId;
	} else {
		can_id = header->StdId;
	}
	int index = searchForMessageId(can_id);
	if (index<0)
		return false;
	struct queued_message message = {
		.message_idx = index,
		.message_contents = *(uint64_t*)rx_data
	};
	queueMessagePush(&receiving_queue, message);

	osThreadFlagsSet(can_task_id, CAN_DB_FLAGS_PENDING_RX.flagInt);
	return true;
}

void StartCANDatabaseTask(void* hcan_void) {
	CAN_HandleTypeDef hcan = *(CAN_HandleTypeDef*)hcan_void;
	can_task_id = osThreadGetId();
	osThreadFlagsSet(can_task_id, CAN_DB_FLAGS_NONE.flagInt);
	while (1) {
		CanDbFlags_t flags = {.flagInt = osThreadFlagsGet()};

		// If there are outstanding messages in the queue, send them to the mailbox.
		if (flags.flagBits.PENDING_TX) {
			if (osMutexAcquire(can_db.message_contents_lock, 1) == osOK) {
				struct queued_message message_tx;
				CANSetMessageContents(message_tx.message_idx, message_tx.message_contents);
				while (queueMessagePop(&sending_queue, &message_tx)) {
					CANDatabaseMessage_t message_info = can_db.messages[message_tx.message_idx];
					CAN_TxHeaderTypeDef pHeader = {
						.DLC = 8,
					};
					if (message_info.can_id >= 0x800) {
						pHeader.IDE = true;
						pHeader.ExtId = message_info.can_id;
					} else {
						pHeader.IDE = false;
						pHeader.StdId = message_info.can_id;
					}
					uint32_t mailbox;
					HAL_CAN_AddTxMessage(&hcan, &pHeader, (uint8_t*)&message_tx.message_contents, &mailbox);
				}
				osMutexRelease(can_db.message_contents_lock);
			}
		}

		// If there are pending received messages, commit them to the DB, and call their callbacks.
		if (flags.flagBits.PENDING_RX) {
			if (osMutexAcquire(can_db.message_contents_lock, 1) == osOK) {
				struct queued_message message_rx;
				while (queueMessagePop(&receiving_queue, &message_rx)) {
					CANDatabaseMessage_t message_info = can_db.messages[message_rx.message_idx];
					CANSetMessageContents(message_rx.message_idx, message_rx.message_contents);
					CANCallback_t callback = can_db.callbacks[message_rx.message_idx];
					if (callback) {
						void *payload = can_db.callback_payloads[message_rx.message_idx];
						callback(message_info.can_id, message_rx.message_contents, payload);
					}
				}
				osMutexRelease(can_db.message_contents_lock);
			}
		}

		osThreadFlagsWait(CAN_DB_FLAGS_ALL.flagInt, osFlagsWaitAny, 10);
	}
}

