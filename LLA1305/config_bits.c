/*
 * config_bits.c
 *
 * Created: 03.07.2023 1:26:20
 *  Author: Xusniyor
 */ 

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include <avr/io.h>

/************************************************************************/
/* Configures Fuse bits                                                 */
/************************************************************************/
FUSES =
{
	.APPEND = 0x0,
	.BODCFG = ACTIVE_DIS_gc | LVL_BODLEVEL0_gc | SAMPFREQ_1KHz_gc | SLEEP_DIS_gc,
	.BOOTEND = 0x0,
	.OSCCFG = FREQSEL_20MHZ_gc,
	.SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_UPDI_gc,
	.SYSCFG1 = SUT_64MS_gc,
	.WDTCFG = PERIOD_8KCLK_gc | WINDOW_OFF_gc,
};

