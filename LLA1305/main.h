/*
 * main.h
 *
 * Created: 04.06.2023 1:18:59
 *  Author: Xusniyor
 */

#ifndef MAIN_H_
#define MAIN_H_

/************************************************************************/
/* CPU Clock is 10MHz                                                   */
/************************************************************************/
#ifndef F_CPU
#define F_CPU 10000000UL
#endif

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "twi.h"

#define DEFAULT_MAX_MC60_NO_ACTIVE_TIME 60000UL

/************************************************************************/
/* I/O ports and pins                                                   */
/************************************************************************/
/* MC60 */
#define MC60_VBAT_EN_PORT PORTA
#define MC60_VBAT_EN_PIN PIN4_bm
#define MC60_PWRKEY_PORT PORTA
#define MC60_PWRKEY_PIN PIN1_bm
/* LED */
#define LED_PORT PORTC
#define LED_PIN PIN2_bm
/* WDS */
#define WDS_PORT1_PORT PORTB
#define WDS_PORT1_PIN PIN4_bm
#define WDS_PORT2_PORT PORTB
#define WDS_PORT2_PIN PIN5_bm
/* RDS */
#define RDS_PORT1_PORT PORTC
#define RDS_PORT1_PIN PIN0_bm
#define RDS_PORT2_PORT PORTC
#define RDS_PORT2_PIN PIN1_bm
/* RAS */
#define RAS_PORT1 ADC_MUXPOS_AIN5_gc
#define RAS_PORT2 ADC_MUXPOS_AIN6_gc
#define RAS_VIN ADC_MUXPOS_AIN7_gc

#endif /* MAIN_H_ */