/*
 * board.c
 *
 * Created: 04.06.2023 14:22:33
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "board.h"

/************************************************************************/
/* Board Port1 Set                                                      */
/************************************************************************/
void Board_Port1_Set(uint8_t status)
{
	if (status)
		BOARD_WDS_PORT1_PORT.OUTSET = BOARD_WDS_PORT1_PIN;
	else
		BOARD_WDS_PORT1_PORT.OUTCLR = BOARD_WDS_PORT1_PIN;
}

/************************************************************************/
/* Board Port2 Set                                                      */
/************************************************************************/
void Board_Port2_Set(uint8_t status)
{
	if (status)
		BOARD_WDS_PORT2_PORT.OUTSET = BOARD_WDS_PORT2_PIN;
	else
		BOARD_WDS_PORT2_PORT.OUTCLR = BOARD_WDS_PORT2_PIN;
}

/************************************************************************/
/* Board Port1 Get                                                      */
/************************************************************************/
uint8_t Board_Port1_Get(void)
{
	return (BOARD_RDS_PORT1_PORT.IN & BOARD_RDS_PORT1_PIN) ? 0 : 1;
}

/************************************************************************/
/* Board Port2 Get                                                      */
/************************************************************************/
uint8_t Board_Port2_Get(void)
{
	return (BOARD_RDS_PORT2_PORT.IN & BOARD_RDS_PORT2_PIN) ? 0 : 1;
}
