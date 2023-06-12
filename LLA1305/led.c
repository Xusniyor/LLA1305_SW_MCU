/*
 * led.c
 *
 * Created: 04.06.2023 4:54:58
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "led.h"

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/
uint8_t _LED_Blinking_Times;
uint16_t _LED_Blinking_Period;
uint16_t _LED_Off_Time;

/************************************************************************/
/* LED Set Blinking Time                                                */
/************************************************************************/
void LED_Set_Blinking_Time(uint8_t LED_Blinking_Times, uint16_t LED_Blinking_Period, uint16_t LED_Off_Time)
{
	_LED_Blinking_Times = LED_Blinking_Times;
	_LED_Blinking_Period = LED_Blinking_Period;
	_LED_Off_Time = LED_Off_Time;
}

/************************************************************************/
/* LED Loop                                                             */
/************************************************************************/
void LED_Loop(void)
{
	for (uint8_t i = 0; i < _LED_Blinking_Times; i++)
	{
		LED_PORT.OUTSET = LED_PIN;
		for (uint16_t j = 0; j < _LED_Blinking_Period; j++)
			_delay_ms(1);
		LED_PORT.OUTCLR = LED_PIN;
		for (uint16_t j = 0; j < _LED_Blinking_Period; j++)
			_delay_ms(1);
	}
	for (uint16_t i = 0; i < _LED_Off_Time; i++)
		_delay_ms(1);
}
