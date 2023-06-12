/*
 * adc.c
 *
 * Created: 04.06.2023 1:48:32
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "adc.h"

/************************************************************************/
/* ADC - Configuration                                                 */
/************************************************************************/
void ADC_Configuration(void)
{
	VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc; // ADC0 Internal Reference Voltage 2.5V
	VREF.CTRLB = VREF_ADC0REFEN_bm;		 // Enable Internal Reference Voltage
	ADC0.CTRLA = ADC_ENABLE_bm;			 // Enable ADC
	ADC0.CTRLB = 0x00;					 // No Sample Accumulation
	ADC0.CTRLC = ADC_PRESC_DIV32_gc;	 // ADC Prescaler 32X
}

/************************************************************************/
/* ADC - Read Analog Signal                                            */
/************************************************************************/
uint16_t ADC_ReadAnalogSignal(ADC_MUXPOS_t Channel)
{
	ADC0.MUXPOS = Channel;		  // Choose Channel
	ADC0.COMMAND = ADC_STCONV_bm; // Start Converting
	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
	{
		asm("NOP");
	}				 // Waiting Conversion
	return ADC0.RES; // Return Results
}
