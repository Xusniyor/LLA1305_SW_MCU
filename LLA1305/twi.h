/*
 * twi.h
 *
 * Created: 04.06.2023 0:02:17
 *  Author: Xusniyor
 */

#ifndef TWI_H_
#define TWI_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

/* Transaction status defines.*/
#define TWIS_STATUS_READY 0
#define TWIS_STATUS_BUSY 1

/* Transaction result enumeration */
typedef enum __attribute__((packed)) TWIS_RESULT_enum
{
	TWIS_RESULT_UNKNOWN = (0x00 << 0),
	TWIS_RESULT_OK = (0x01 << 0),
	TWIS_RESULT_BUFFER_OVERFLOW = (0x02 << 0),
	TWIS_RESULT_TRANSMIT_COLLISION = (0x03 << 0),
	TWIS_RESULT_BUS_ERROR = (0x04 << 0),
	TWIS_RESULT_FAIL = (0x05 << 0),
	TWIS_RESULT_ABORTED = (0x06 << 0),
} TWIS_RESULT_t;

/*! TWI Modes */
typedef enum __attribute__((packed)) TWI_MODE_enum
{
	TWI_MODE_UNKNOWN = 0,
	TWI_MODE_MASTER = 1,
	TWI_MODE_SLAVE = 2,
	TWI_MODE_MASTER_TRANSMIT = 3,
	TWI_MODE_MASTER_RECEIVE = 4,
	TWI_MODE_SLAVE_TRANSMIT = 5,
	TWI_MODE_SLAVE_RECEIVE = 6
} TWI_MODE_t;

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void TWI_SlaveInterruptHandler(void);
void TWI_SlaveAddressMatchHandler(void);
void TWI_SlaveStopHandler(void);
void TWI_SlaveDataHandler(void);
void TWI_SlaveWriteHandler(void);
void TWI_SlaveReadHandler(void);
void TWI_attachSlaveRxEvent(void (*function)(int), uint8_t *read_data, uint8_t bytes_to_read);
void TWI_attachSlaveTxEvent(uint8_t (*function)(void), uint8_t *write_data);
void TWI_SlaveTransactionFinished(uint8_t result);

#endif /* TWI_H_ */
