/*
 * can_db.h
 *
 *  Created on: Feb 21, 2025
 *      Author: bre17
 */

#ifndef INC_CAN_DB_H_
#define INC_CAN_DB_H_

typedef struct {
	uint32_t can_id;
	int num_signals;
	CANDatabaseSignal_t *signals;
} CANDatabaseMessage_t;

typedef struct {
	int bitpos;
	uint64_t bitmask;
} CANDatabaseSignal_t;

#define _DEFINE_CAN_DB(...) \
	typedef struct {} CANDatabase_t; \
	typedef enum {} CANDatabaseEntries_t;

#define _CAN_DB_MESSAGE(name, canid, ...)

#define _CAN_DB_SIGNAL(name, bitpos, bitmask)

_DEFINE_CAN_DB(
	_CAN_DB_ENTRY()
)

#endif /* INC_CAN_DB_H_ */
