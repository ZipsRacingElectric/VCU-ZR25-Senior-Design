/*
 * can_db.c
 *
 *  Created on: Feb 22, 2025
 *      Author: tetra
 */

#include "can_db.h"
#include "stdbool.h"
#include "can_messages.h"

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
	int nextEnd = queue->end + 1;
	// wrap around
	while (nextEnd >= queue->size)
		nextEnd -= queue->size;

	if (nextEnd == queue->start)
		return false; // queue full

	queue->messages[queue->end] = message;
	queue->end = nextEnd;
	return true;
}

bool queueMessagePop(struct queue *queue, struct queued_message *message_out) {
	if (queue->start == queue->end)
		return false; // queue empty

	int nextStart = queue->start + 1;
	// wrap around
	while (nextStart >= queue->size)
		nextStart -= queue->size;

	*message_out = queue->messages[queue->start];
	queue->start = nextStart;
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
bool CANQueueSendingMessage(CANDatabaseEntryId index, uint64_t contents) {
	return queueMessagePush(
		&sending_queue,
		(struct queued_message){.message_idx = index, .message_contents = contents}
	);
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

void StartCANDatabaseTask(void* _argument) {
	while (1) {
		// TODO: If there are outstanding message in the queue, send them to the mailbox.
		// TODO: If there are pending received messages, commit them to the DB, and call their callbacks.
		osDelay(50);
	}
}

