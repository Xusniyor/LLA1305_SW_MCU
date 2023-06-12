/*
 * clock.c
 *
 * Created: 04.06.2023 1:31:49
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "clock.h"

/************************************************************************/
/* CLKCTRL - Clock Controller                                           */
/************************************************************************/
void Clock_Controller_Configuration(void)
{
	CPU_CCP = CCP_IOREG_gc;									 // Unlock IOreg
	CLKCTRL.MCLKCTRLB = CLKCTRL_PEN_bm | CLKCTRL_PDIV_2X_gc; // Main Clock Prescaler 2X
}
