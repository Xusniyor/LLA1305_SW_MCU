/*
 * board.h
 *
 * Created: 04.06.2023 14:22:19
 *  Author: Xusniyor
 */

#ifndef BOARD_H_
#define BOARD_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void Board_Port1_Set(uint8_t status);
void Board_Port2_Set(uint8_t status);
uint8_t Board_Port1_Get(void);
uint8_t Board_Port2_Get(void);

#endif /* BOARD_H_ */
