/*
 * can_db.c
 *
 *  Created on: Feb 22, 2025
 *      Author: tetra
 */

#include "can_db.h"
#include "gpio.h"
#include "main.h"
#include "stdbool.h"
#include "can_messages.h"
#include "stm32f4xx_hal_can.h"

extern osMessageQueueId_t canDbRxQueueHandle;
extern osMessageQueueId_t canDbTxQueueHandle;
extern osMutexId_t can_db_lockHandle;

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

#define CAN_DB_FLAGS_ALL ((CanDbFlags_t){.flagInt = -1})
#define CAN_DB_FLAGS_NONE ((CanDbFlags_t){.flagInt = 0})
#define CAN_DB_FLAGS_PENDING_RX ((CanDbFlags_t){.flagBits = {.PENDING_RX=1}})
#define CAN_DB_FLAGS_PENDING_TX ((CanDbFlags_t){.flagBits = {.PENDING_TX=1}})

struct queued_message {
	uint32_t message_idx;
	CAN_HandleTypeDef* hcan;
	uint64_t message_contents;
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
	swapMessageEntries(start, pivot);
	pivot = start;

	int i = start+1;
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
bool CANQueueMessageToSend(CANDatabaseEntryId index, uint64_t contents, CAN_HandleTypeDef* hcan) {
	struct queued_message message = {
		.message_contents = contents,
		.message_idx = index,
		.hcan = hcan
	};

	osStatus_t status = osMessageQueuePut(canDbTxQueueHandle, (void*)&message, 0, 0);
	osThreadFlagsSet(can_task_id, CAN_DB_FLAGS_PENDING_TX.flagInt);
	return status == osOK;
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

	osMessageQueuePut(canDbRxQueueHandle, (void*)&message, 0, 0);

	osThreadFlagsSet(can_task_id, CAN_DB_FLAGS_PENDING_RX.flagInt);
	return true;
}


const static CAN_FilterTypeDef CAN1_FILTERS[0] = {};
const static CAN_FilterTypeDef CAN2_FILTERS[1] = {
		{
				.FilterBank = 0,
				.FilterActivation = CAN_FILTER_ENABLE,
				.FilterMode = CAN_FILTERMODE_IDLIST,
				.FilterScale = CAN_FILTERSCALE_16BIT,
				.FilterIdLow = 0x204<<5, .FilterIdHigh = 0,
				.FilterMaskIdLow = 0x0,
				.FilterMaskIdHigh = 0x0,
		}
};

void StartCanDbTask(void* _argument) {
	// Initialize CAN filters
	for (int i=0; i<sizeof(CAN1_FILTERS)/sizeof(CAN_FilterTypeDef); i++) {
		HAL_CAN_ConfigFilter(&hcan1, &CAN1_FILTERS[i]);
	}
	for (int i=0; i<sizeof(CAN2_FILTERS)/sizeof(CAN_FilterTypeDef); i++) {
		HAL_CAN_ConfigFilter(&hcan2, &CAN2_FILTERS[i]);
	}
	// Wait for 5V rail to come up
	while (HAL_GPIO_ReadPin(RAIL_POWER_ENABLE_5V_GPIO_Port, RAIL_POWER_ENABLE_5V_Pin) == GPIO_PIN_RESET) {
		osDelay(1);
	}
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);

	can_task_id = osThreadGetId();
	while (1) {
		CanDbFlags_t flags = {.flagInt = osThreadFlagsGet()};

		// If there are outstanding messages in the queue, send them to the mailbox.
		if (flags.flagBits.PENDING_TX) {
			if (osMutexAcquire(can_db_lockHandle, 1) == osOK) {
				struct queued_message message_tx;
				while (osMessageQueueGet(canDbTxQueueHandle, &message_tx, NULL, 0) == osOK ) {
					CANSetMessageContents(message_tx.message_idx, message_tx.message_contents);
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
					HAL_CAN_AddTxMessage(message_tx.hcan, &pHeader, (uint8_t*)&message_tx.message_contents, &mailbox);
				}
				osMutexRelease(can_db_lockHandle);
			}
		}

		// If there are pending received messages, commit them to the DB, and call their callbacks.
		if (flags.flagBits.PENDING_RX) {
			if (osMutexAcquire(can_db_lockHandle, 1) == osOK) {
				struct queued_message message_rx;
				while (osMessageQueueGet(canDbRxQueueHandle, &message_rx, NULL, 0) == osOK) {
					CANDatabaseMessage_t message_info = can_db.messages[message_rx.message_idx];
					CANSetMessageContents(message_rx.message_idx, message_rx.message_contents);
					CANCallback_t callback = can_db.callbacks[message_rx.message_idx];
					if (callback) {
						void *payload = can_db.callback_payloads[message_rx.message_idx];
						callback(message_info.can_id, message_rx.message_contents, payload);
					}
				}
				osMutexRelease(can_db_lockHandle);
			}
		}

		osThreadFlagsWait(CAN_DB_FLAGS_ALL.flagInt, osFlagsWaitAny, 10);
	}
}

