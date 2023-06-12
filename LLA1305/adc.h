/*
 * adc.h
 *
 * Created: 04.06.2023 1:48:20
 *  Author: Xusniyor
 */

#ifndef ADC_H_
#define ADC_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void ADC_Configuration(void);
uint16_t ADC_ReadAnalogSignal(ADC_MUXPOS_t Channel);

#endif /* ADC_H_ */
