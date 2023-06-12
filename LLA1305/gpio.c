/*
 * gpio.c
 *
 * Created: 04.06.2023 2:20:29
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "gpio.h"

/************************************************************************/
/* GPIO - Pins Configuration                                         */
/************************************************************************/
void GPIO_Pins_Configuration(void)
{
	MC60_VBAT_EN_PORT.OUTSET = MC60_VBAT_EN_PIN;	   // PA4 Set to High for VBAT to Remain Off
	MC60_PWRKEY_PORT.DIRSET = MC60_PWRKEY_PIN;		   // PA1 Output for MC60_PWRKEY
	MC60_VBAT_EN_PORT.DIRSET = MC60_VBAT_EN_PIN;	   // PA4 Output for MC60_VBAT_EN
	BOARD_WDS_PORT1_PORT.DIRSET = BOARD_WDS_PORT1_PIN; // PB4 Output for WDS_PORT1
	BOARD_WDS_PORT2_PORT.DIRSET = BOARD_WDS_PORT2_PIN; // PB5 Output for WDS_PORT2
	LED_PORT.DIRSET = LED_PIN;						   // PC2 Output for LED_STATUS
}
