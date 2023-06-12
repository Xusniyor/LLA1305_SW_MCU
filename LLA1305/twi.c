/*
 * twi.c
 *
 * Created: 03.06.2023 23:53:01
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "twi.h"

/* Slave variables */
static uint8_t (*TWI_onSlaveTransmit)(void) __attribute__((unused));
static void (*TWI_onSlaveReceive)(int) __attribute__((unused));
static uint8_t *slave_writeData;
static uint8_t *slave_readData;
static uint8_t slave_bytesToWrite;
static uint8_t slave_bytesWritten;
static uint8_t slave_bytesToRead;
static uint8_t slave_bytesRead;
static uint8_t slave_trans_status;
static uint8_t slave_result;
static uint8_t slave_callUserReceive;

/* TWI module mode */
static volatile TWI_MODE_t twi_mode;

/*! \brief Common TWI slave interrupt service routine.
 *
 *  Check current status and calls the appropriate handler.
 *
 */
void TWI_SlaveInterruptHandler()
{
  uint8_t currentStatus = TWI0.SSTATUS;

  /* If bus error */
  if (currentStatus & TWI_BUSERR_bm)
  {
    slave_bytesRead = 0;
    slave_bytesWritten = 0;
    slave_bytesToWrite = 0;
    TWI_SlaveTransactionFinished(TWIS_RESULT_BUS_ERROR);
  }

  /* If Address or Stop */
  else if (currentStatus & TWI_APIF_bm)
  {
    /* Call user onReceive function if end of Master Write/Slave Read.
     * This should be hit when there is a STOP or REPSTART
     */
    if (slave_callUserReceive == 1)
    {
      TWI_onSlaveReceive(slave_bytesRead);
      slave_callUserReceive = 0;
    }

    /* If address match */
    if (currentStatus & TWI_AP_bm)
    {
      TWI_SlaveAddressMatchHandler();
    }

    /* If stop */
    else
    {
      TWI_SlaveStopHandler();

      /* If CLKHOLD is high, we have missed an address match
        from a fast start after stop.
        Because the flag is shared we need to handle this here.
      */
      if (TWI0.SSTATUS & TWI_CLKHOLD_bm)
      {
        /* CLKHOLD will be cleared by servicing the address match */
        TWI_SlaveAddressMatchHandler();
      }
    }
  }

  /* If Data Interrupt */
  else if (currentStatus & TWI_DIF_bm)
  {
    /* If collision flag is raised, slave transmit unsuccessful */
    if (currentStatus & TWI_COLL_bm)
    {
      slave_bytesRead = 0;
      slave_bytesWritten = 0;
      slave_bytesToWrite = 0;
      TWI_SlaveTransactionFinished(TWIS_RESULT_TRANSMIT_COLLISION);
    }

    /* Otherwise, normal data interrupt */
    else
    {
      TWI_SlaveDataHandler();
    }
  }

  /* If unexpected state */
  else
  {
    TWI_SlaveTransactionFinished(TWIS_RESULT_FAIL);
  }
}

/*! \brief TWI slave address interrupt handler.
 *
 *  This is the slave address match handler that takes care of responding to
 *  being addressed by a master
 *
 */
void TWI_SlaveAddressMatchHandler()
{
  slave_trans_status = TWIS_STATUS_BUSY;
  slave_result = TWIS_RESULT_UNKNOWN;

  /* Send ACK, wait for data interrupt */
  TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;

  /* If Master Read/Slave Write */
  if (TWI0.SSTATUS & TWI_DIR_bm)
  {
    slave_bytesWritten = 0;
    /* Call user function  */
    slave_bytesToWrite = TWI_onSlaveTransmit();
    twi_mode = TWI_MODE_SLAVE_TRANSMIT;
  }
  /* If Master Write/Slave Read */
  else
  {
    slave_bytesRead = 0;
    slave_callUserReceive = 1;
    twi_mode = TWI_MODE_SLAVE_RECEIVE;
  }

  /* Data interrupt to follow... */
}

/*! \brief TWI slave stop interrupt handler.
 *
 */
void TWI_SlaveStopHandler()
{
  /* Clear APIF, don't ACK or NACK */
  TWI0.SSTATUS = TWI_APIF_bm;

  TWI_SlaveTransactionFinished(TWIS_RESULT_OK);
}

/*! \brief TWI slave data interrupt handler.
 *
 *  This is the slave data handler that takes care of sending data to or
 *  receiving data from a master
 *
 */
void TWI_SlaveDataHandler()
{
  /* Enable stop interrupt */
  TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm);

  /* If Master Read/Slave Write */
  if (TWI0.SSTATUS & TWI_DIR_bm)
  {
    TWI_SlaveWriteHandler();
  }

  /* If Master Write/Slave Read */
  else
  {
    TWI_SlaveReadHandler();
  }
}

/*! \brief TWI slave data write interrupt handler.
 *
 *  This is the slave data handler that takes care of sending data to a master
 *
 */
void TWI_SlaveWriteHandler()
{
  /* If NACK, slave write transaction finished */
  if ((slave_bytesWritten > 0) && (TWI0.SSTATUS & TWI_RXACK_bm))
  {
    TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    TWI_SlaveTransactionFinished(TWIS_RESULT_OK);
  }

  /* If ACK, master expects more data */
  else
  {
    if (slave_bytesWritten < slave_bytesToWrite)
    {
      uint8_t data = slave_writeData[slave_bytesWritten];
      TWI0.SDATA = data;
      slave_bytesWritten++;

      /* Send data, wait for data interrupt */
      TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
    }

    /* If buffer overflow */
    else
    {
      TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
      TWI_SlaveTransactionFinished(TWIS_RESULT_BUFFER_OVERFLOW);
    }
  }
}

/*! \brief TWI slave data read interrupt handler.
 *
 *  This is the slave data handler that takes care of receiving data from a master
 *
 */
void TWI_SlaveReadHandler()
{
  /* If free space in buffer */
  if (slave_bytesRead < slave_bytesToRead)
  {
    /* Fetch data */
    uint8_t data = TWI0.SDATA;
    slave_readData[slave_bytesRead] = data;
    slave_bytesRead++;

    /* Send ACK and wait for data interrupt */
    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
  }
  /* If buffer overflow, send NACK and wait for next START.
    Set result buffer overflow */
  else
  {
    TWI0.SCTRLB = TWI_ACKACT_bm | TWI_SCMD_COMPTRANS_gc;
    TWI_SlaveTransactionFinished(TWIS_RESULT_BUFFER_OVERFLOW);
  }
}

/*
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void TWI_attachSlaveRxEvent(void (*function)(int), uint8_t *read_data, uint8_t bytes_to_read)
{
  TWI_onSlaveReceive = function;
  slave_readData = read_data;
  slave_bytesToRead = bytes_to_read;
}

/*
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void TWI_attachSlaveTxEvent(uint8_t (*function)(void), uint8_t *write_data)
{
  TWI_onSlaveTransmit = function;
  slave_writeData = write_data;
}

/*! \brief TWI slave transaction finished handler.
 *
 *  Prepares module for new transaction.
 *
 *  \param result  The result of the operation.
 */
void TWI_SlaveTransactionFinished(uint8_t result)
{
  TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm);
  twi_mode = TWI_MODE_SLAVE;
  slave_result = result;
  slave_trans_status = TWIS_STATUS_READY;
}

/************************************************************************/
/* TWI Slave Interrupt                                                  */
/************************************************************************/
ISR(TWI0_TWIS_vect)
{
  TWI_SlaveInterruptHandler();
}
