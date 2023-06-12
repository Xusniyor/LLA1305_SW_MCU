/*
 * mc60.h
 *
 * Created: 04.06.2023 0:54:26
 *  Author: Xusniyor
 */

#ifndef MC60_H_
#define MC60_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

/************************************************************************/
/* These Functions are used to connect and disconnect                   */
/* the MC60 VBAT from the power source                                  */
/************************************************************************/
void MC60_VBAT_Power_On(void);
void MC60_VBAT_Power_Off(void);

/************************************************************************/
/* This function is used to turn the MC60 on or off                     */
/* after it is connected to a power source                              */
/* You Should Hold for Turn-On 1100ms or Turn-Off 800ms                 */
/************************************************************************/
void MC60_Press_Power_Key(uint16_t HoldTimeMs);

#endif /* MC60_H_ */
