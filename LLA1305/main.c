/*
 * LLA1305.c
 *
 * Created: 01.06.2023 23:43:56
 * Author : Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"
#include "clock.h"
#include "uart.h"
#include "adc.h"
#include "twi.h"
#include "gpio.h"
#include "board.h"
#include "mc60.h"
#include "led.h"

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/
uint16_t TWI_Last_Active_Time;
uint8_t TWI_TX_Data_Buff[16];
uint8_t TWI_RX_Data_Buff[8];
uint8_t Board_Port1_Output_Lock;
uint8_t Board_Port2_Output_Lock;

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void TWI_Configuration(void);
void TWI_Slave_Read(int ByteLength);
uint8_t TWI_Slave_Write(void);
void RTC_Configuration(void);
uint16_t RTC_Get_Value(void);
void WDT_Configuration(void);
void WDT_Restart(void);

/************************************************************************/
/* Main Startup Function                                                */
/************************************************************************/
int main(void)
{
	Clock_Controller_Configuration();
	GPIO_Pins_Configuration();
	UART_Configuration(9600);
	ADC_Configuration();
	TWI_Configuration();
	RTC_Configuration();
	WDT_Configuration();
	sei(); /* Enable global interrupts */
	MC60_VBAT_Power_On();
	MC60_Press_Power_Key(1000);
	LED_Set_Blinking_Time(2, 100, 0);
	Board_Port1_Output_Lock = 0;
	Board_Port2_Output_Lock = 0;
	while (1)
	{
		// printf("RTC: %d\n\r", RTC_Get_Value());
		// printf("RAS_PORT1: %d ", ADC_ReadAnalogSignal(RAS_PORT1));
		// printf("RAS_PORT2: %d ", ADC_ReadAnalogSignal(RAS_PORT2));
		// printf("RAS_VIN: %d\n\r", ADC_ReadAnalogSignal(RAS_VIN));

		/* LED Loop for LED Blinking */
		LED_Loop();
		/* Preparation of TWI TX Information in advance */
		((uint16_t *)&TWI_TX_Data_Buff)[0] = ADC_ReadAnalogSignal(RAS_VIN);
		((uint16_t *)&TWI_TX_Data_Buff)[1] = ADC_ReadAnalogSignal(RAS_PORT1);
		((uint16_t *)&TWI_TX_Data_Buff)[2] = ADC_ReadAnalogSignal(RAS_PORT2);
		/* UART echo Client */
		if (UART_Get_Rx_Data_Len() > 0)
		{
			((uint16_t *)&TWI_TX_Data_Buff)[3] = Board_Port1_Get();
			((uint16_t *)&TWI_TX_Data_Buff)[4] = Board_Port2_Get();
			UART_Send_String(TWI_TX_Data_Buff, sizeof(TWI_TX_Data_Buff));
			UART_Send_String(UART_Get_Rx_Data_Buff(), UART_Get_Rx_Data_Len());
			UART_Clean_Rx_Data();
		}
		/* Reset MC60 if TWI Last Active Time more 600 Seconds */
		if (RTC_Get_Value() - TWI_Last_Active_Time > 600)
		{
			MC60_VBAT_Power_Off();
			/* Blocked until WDT Reset */
			while (1)
			{
				asm("NOP");
			}
		}
		/* Watchdog Timer Restart */
		WDT_Restart();
	}
}

/************************************************************************/
/* TWI - Configuration                                                  */
/************************************************************************/
void TWI_Configuration(void)
{
	memset(TWI_TX_Data_Buff, 0x00, sizeof(TWI_TX_Data_Buff));
	memset(TWI_RX_Data_Buff, 0x00, sizeof(TWI_RX_Data_Buff));
	TWI_attachSlaveRxEvent(TWI_Slave_Read, TWI_RX_Data_Buff, sizeof(TWI_RX_Data_Buff));
	TWI_attachSlaveTxEvent(TWI_Slave_Write, TWI_TX_Data_Buff);
	TWI0.SADDR = 0x10;														// Client Address
	TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm; // APIEN enabled; DIEN enabled; ENABLE enabled; PIEN enabled; PMEN disabled; SMEN disabled;
}

/************************************************************************/
/* TWI - Slave Read                                                     */
/************************************************************************/
void TWI_Slave_Read(int ByteLength)
{
	/* Set Status LED Blink Config */
	if (TWI_RX_Data_Buff[6] == 0x01)
	{
		LED_Set_Blinking_Time((uint8_t)((uint16_t *)&TWI_RX_Data_Buff)[0], ((uint16_t *)&TWI_RX_Data_Buff)[1], ((uint16_t *)&TWI_RX_Data_Buff)[2]);
	}
	/* Set Status Board Port1 and Port2 Output Lock */
	else if (TWI_RX_Data_Buff[6] == 0x02)
	{
		Board_Port1_Output_Lock = TWI_RX_Data_Buff[0];
		Board_Port2_Output_Lock = TWI_RX_Data_Buff[2];
	}
	/* Set Status Board Port1 and Port2 Output */
	else if (TWI_RX_Data_Buff[6] == 0x04)
	{
		if (Board_Port1_Output_Lock)
			Board_Port1_Set(TWI_RX_Data_Buff[0]);
		if (Board_Port2_Output_Lock)
			Board_Port2_Set(TWI_RX_Data_Buff[2]);
	}
	TWI_Last_Active_Time = RTC_Get_Value();
}

/************************************************************************/
/* TWI0 - Slave Write                                                   */
/************************************************************************/
uint8_t TWI_Slave_Write(void)
{
	((uint16_t *)&TWI_TX_Data_Buff)[3] = Board_Port1_Get();
	((uint16_t *)&TWI_TX_Data_Buff)[4] = Board_Port2_Get();
	TWI_Last_Active_Time = RTC_Get_Value();
	return sizeof(TWI_TX_Data_Buff);
}

/************************************************************************/
/* RTC Configuration                                                    */
/************************************************************************/
void RTC_Configuration(void)
{
	RTC.CTRLA = RTC_PRESCALER_DIV32768_gc | RTC_RTCEN_bm;
}

/************************************************************************/
/* RTC Get Value                                                        */
/************************************************************************/
uint16_t RTC_Get_Value(void)
{
	return RTC.CNT;
}

/************************************************************************/
/* WDT Configuration                                                    */
/************************************************************************/
void WDT_Configuration(void)
{
	while (WDT.STATUS & WDT_SYNCBUSY_bm)
	{
		asm("NOP");
	} // Check SYNCBUSY
	WDT.CTRLA = WDT_PERIOD_8KCLK_gc;
}

/************************************************************************/
/* WDT Restart                                                          */
/************************************************************************/
void WDT_Restart(void)
{
	asm("WDR");
}
