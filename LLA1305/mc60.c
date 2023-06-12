/*
 * mc60.c
 *
 * Created: 04.06.2023 0:52:27
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "mc60.h"

/************************************************************************/
/* These Functions are used to connect and disconnect                   */
/* the MC60 VBAT from the power source                                  */
/************************************************************************/
void MC60_VBAT_Power_On(void)
{
	MC60_VBAT_EN_PORT.OUTCLR = MC60_VBAT_EN_PIN; // Power_On
	_delay_ms(100);								 // Make sure that VBAT is stable before pulling down PWRKEY pin
}
void MC60_VBAT_Power_Off(void)
{
	MC60_VBAT_EN_PORT.OUTSET = MC60_VBAT_EN_PIN; // Power_On
	_delay_ms(1000);							 // Wait for VBAT to completely power down
}

/************************************************************************/
/* This function is used to turn the MC60 on or off                     */
/* after it is connected to a power source                              */
/* You Should Hold for Turn-On 1100ms or Turn-Off 800ms                 */
/************************************************************************/
void MC60_Press_Power_Key(uint16_t HoldTimeMs)
{
	MC60_PWRKEY_PORT.OUTSET = MC60_PWRKEY_PIN; // Press
	for (uint16_t i = 0; i < HoldTimeMs; i++)
		_delay_ms(1);						   // Hold
	MC60_PWRKEY_PORT.OUTCLR = MC60_PWRKEY_PIN; // Back
}
