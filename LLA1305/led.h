/*
 * led.h
 *
 * Created: 04.06.2023 4:54:45
 *  Author: Xusniyor
 */

#ifndef LED_H_
#define LED_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void LED_Set_Blinking_Time(uint8_t LED_Blinking_Times, uint16_t LED_Blinking_Period, uint16_t LED_Off_Time);
void LED_Loop(void);

#endif /* LED_H_ */
