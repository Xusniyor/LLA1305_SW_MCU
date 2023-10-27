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

/************************************************************************/
/* Program Information                                                  */
/************************************************************************/
const char *BuildVersion = "V0.1";
const char *BuildData = __DATE__;
const char *BuildTime = __TIME__;

/************************************************************************/
/* Typedefs                                                             */
/************************************************************************/
typedef struct
{
	bool FirstToSecond;
	
	uint16_t First_ExInt_us;
	uint16_t Second_ExInt_us;
	uint32_t First_ExInt_ms;
	uint32_t Second_ExInt_ms;
} Port_t;

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/
const ADC_MUXPOS_t ADC_Channel[3] = { RAS_PORT1, RAS_PORT2, RAS_VIN };
uint16_t ADC_Result[3];
uint8_t ADC_Index;

Port_t Port1, Port2;
uint32_t SysTick;

uint32_t Last_MC60_Data_Exchange_Time;
uint32_t Max_MC60_No_Active_Time;
uint8_t TWI_Rx_Data_Buff[128];
uint8_t *pTWI_Tx_Data_Point;
uint8_t TWI_Data_Length;

uint32_t *pDWord_Data_Point;
uint16_t *pWord_Data_Point;
uint8_t  *pBayt_Data_Point;

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void PORT_PWM_Enable(uint8_t Channel, uint8_t DutyCycle, uint16_t Frequency);
void TWI_Slave_Read(int ByteLength);
uint8_t TWI_Slave_Write(void);

/*************************************************************************
int UART_Send_Char(char data, FILE *stream);
static FILE UART_Stream = FDEV_SETUP_STREAM(UART_Send_Char, NULL, _FDEV_SETUP_WRITE);
*************************************************************************/

/************************************************************************/
/* Main Startup Function                                                */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* GPIOs Initialization                                                 */
	/************************************************************************/
	/* WDS_PORT1 and WDS_PORT2 are Output */
	WDS_PORT1_PORT.DIRSET = WDS_PORT1_PIN;
	WDS_PORT2_PORT.DIRSET = WDS_PORT2_PIN;
	/* MC60_VBAT_EN Set to High and Output */
	MC60_VBAT_EN_PORT.OUTSET = MC60_VBAT_EN_PIN;
	MC60_VBAT_EN_PORT.DIRSET = MC60_VBAT_EN_PIN;
	/* MC60_PWRKEY is Output */
	MC60_PWRKEY_PORT.DIRSET = MC60_PWRKEY_PIN;
	/* LED_PORT is Output */
	LED_PORT.DIRSET = LED_PIN;
	/* UART RX to Input and TX to Output */
	/*************************************************************************
	PORTB.OUTSET = PIN2_bm; // PB2 Set Pin to High for TX
	PORTB.DIRSET = PIN2_bm; // PB2 Set Pin to Output for TX
	*************************************************************************/
	
	/************************************************************************/
	/* System Clock Initialization                                          */
	/************************************************************************/
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = CLKCTRL_PEN_bm;
	
	/************************************************************************/
	/* UART Initialization                                                  */
	/************************************************************************/
	/*************************************************************************
	USART0.BAUD = 4167;           // Set UART Baud Rate
	USART0.CTRLB = USART_TXEN_bm; // Enable UART TX
	stdout = &UART_Stream;
	*************************************************************************/
	
	/************************************************************************/
	/* Timer Initialization                                                 */
	/************************************************************************/
	TCB0.CTRLA = TCB_ENABLE_bm;
	TCB0.INTCTRL = TCB_CAPT_bm;
	TCB0.CCMP = 9999;
	
	/************************************************************************/
	/* ADC Initialization                                                   */
	/************************************************************************/
	VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc; // ADC0 Internal Reference Voltage 2.5V
	VREF.CTRLB = VREF_ADC0REFEN_bm;		 // Enable Internal Reference Voltage
	ADC0.CTRLA = ADC_ENABLE_bm;			 // Enable ADC
	ADC0.CTRLC = ADC_PRESC_DIV256_gc;	 // CLK_PER divided by 32
	ADC0.INTCTRL = ADC_RESRDY_bm;        // Result Ready Interrupt Enable
	
	/************************************************************************/
	/* Start ADC Conversion                                                 */
	/************************************************************************/
	ADC_Index = 0;
	ADC0.MUXPOS  = ADC_Channel[ADC_Index]; // Choose Channel
	ADC0.COMMAND = ADC_STCONV_bm;          // Start Converting
	
	/************************************************************************/
	/* TWI Initialization                                                   */
	/************************************************************************/
	TWI_attachSlaveRxEvent(TWI_Slave_Read, TWI_Rx_Data_Buff, sizeof(TWI_Rx_Data_Buff));
	TWI_attachSlaveTxEvent(TWI_Slave_Write, pTWI_Tx_Data_Point);
	TWI0.SADDR = 0x10; // TWI Slave Address
	TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;
	
	/************************************************************************/
	/* Enable global interrupts                                             */
	/************************************************************************/
	sei();
	
	/************************************************************************/
	/* MC60 VBAT Power On                                                   */
	/************************************************************************/
	MC60_VBAT_EN_PORT.OUTCLR = MC60_VBAT_EN_PIN;
	_delay_ms(100); // Make sure that VBAT is stable before pulling down PWRKEY pin
	
	/************************************************************************/
	/* MC60 Press Power Key                                                 */
	/************************************************************************/
	MC60_PWRKEY_PORT.OUTSET = MC60_PWRKEY_PIN; // Press
	_delay_ms(1000); /* You Should Hold for Turn-On 1100ms or Turn-Off 800ms */
	MC60_PWRKEY_PORT.OUTCLR = MC60_PWRKEY_PIN; // Back
	
	/************************************************************************/
	/* Clean and Set Default EEPROM if this is a New Device                 */
	/************************************************************************/
	if (!eeprom_read_dword((const uint32_t *)(0)))
	{
		for (uint8_t i = 0; i < 128; i++)
			eeprom_write_byte(((uint8_t *)(0)) + i, 0);
		eeprom_write_dword(((uint32_t *)(100)), DEFAULT_MAX_MC60_NO_ACTIVE_TIME);
	}
	
	/************************************************************************/
	/* Read Last and Init State of Ports and Restore                        */
	/************************************************************************/
	uint8_t TemporaryBayt;
	uint32_t TemporaryDoubleWord;
	/* PORT 1 */
	TemporaryBayt = eeprom_read_byte(((const uint8_t *)(1)));
	if (TemporaryBayt == 1)
	{
		WDS_PORT1_PORT.OUTSET = WDS_PORT1_PIN;
	}
	else if (TemporaryBayt == 2)
	{
		TemporaryBayt       = eeprom_read_byte(((const uint8_t *)(2)));
		TemporaryDoubleWord = eeprom_read_dword(((const uint32_t *)(3)));
		PORT_PWM_Enable(0, TemporaryBayt, TemporaryDoubleWord);
	}
	/* PORT 2 */
	TemporaryBayt = eeprom_read_byte(((const uint8_t *)(10)));
	if (TemporaryBayt == 1)
	{
		WDS_PORT2_PORT.OUTSET = WDS_PORT2_PIN;
	}
	else if (TemporaryBayt == 2)
	{
		TemporaryBayt       = eeprom_read_byte(((const uint8_t *)(11)));
		TemporaryDoubleWord = eeprom_read_dword(((const uint32_t *)(12)));
		PORT_PWM_Enable(1, TemporaryBayt, TemporaryDoubleWord);
	}
	/* Reading initial values of variables */
	Max_MC60_No_Active_Time = eeprom_read_dword(((const uint32_t *)(100)));
	
	//PORT_PWM_Enable(0, 50, 10);
	//PORT_PWM_Enable(1, 50, 10);
	
	while (1)
	{
		/* Watchdog Timer Restart */
		asm("WDR");
		
		if (SysTick - Last_MC60_Data_Exchange_Time > Max_MC60_No_Active_Time)
		{
			MC60_VBAT_EN_PORT.OUTSET = MC60_VBAT_EN_PIN;
		}
		else
		{
			MC60_VBAT_EN_PORT.OUTCLR = MC60_VBAT_EN_PIN;
		}
	}
}

/************************************************************************/
/* TWI - Slave Read                                                     */
/************************************************************************/
void TWI_Slave_Read(int ByteLength)
{
	Last_MC60_Data_Exchange_Time = SysTick;
	
	if (TWI_Rx_Data_Buff[0] == 1)
	{
		MC60_PWRKEY_PORT.OUTSET = MC60_PWRKEY_PIN;
	}
	else if (TWI_Rx_Data_Buff[0] == 2)
	{
		MC60_PWRKEY_PORT.OUTCLR = MC60_PWRKEY_PIN;
	}
}

/************************************************************************/
/* TWI0 - Slave Write                                                   */
/************************************************************************/
uint8_t TWI_Slave_Write(void)
{
	Last_MC60_Data_Exchange_Time = SysTick;
	return 64;
}

void PORT_PWM_Enable(uint8_t Channel, uint8_t DutyCycle, uint16_t Frequency)
{
	if ((Frequency > 1000) || (Frequency < 10) || (DutyCycle > 100))
		return;
	
	/* Init TCA Timer */
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	TCA0.SINGLE.CTRLC = TCA_SINGLE_CMP1OV_bm | TCA_SINGLE_CMP2OV_bm;
	TCA0.SINGLE.PER = 625000UL / Frequency;
	
	/* Enable TCA Timer Channel */
	if (Channel)
	{
		TCA0.SINGLE.CMP1 = (TCA0.SINGLE.PER / 100) * DutyCycle;
		PORTMUX.CTRLC |= PORTMUX_TCA01_bm;
	}
	else
	{
		TCA0.SINGLE.CMP2 = (TCA0.SINGLE.PER / 100) * DutyCycle;
		PORTMUX.CTRLC |= PORTMUX_TCA02_bm;
	}
}

/************************************************************************/
/* UART - Send 8-Bit Data                                               */
/************************************************************************/
/*************************************************************************
int UART_Send_Char(char data, FILE *stream)
{
	while (!(USART0.STATUS & USART_DREIF_bm)); // Wait Sending Data
	USART0.TXDATAL = data; // Send Data
	return 0;
}
*************************************************************************/

ISR(PORTC_PORT_vect)
{
	/************************************************************************/
	/* RDS_PORT1 EXINT                                                      */
	/************************************************************************/
	// Check Interrupt Port
	if (RDS_PORT1_PORT.INTFLAGS & RDS_PORT1_PIN)
	{
		// Check First or Second Interrupt
		if (Port1.FirstToSecond)
		{
			// Get Second ExInt Time
			Port1.Second_ExInt_us = TCB0.CNT;
			Port1.Second_ExInt_ms = SysTick;
			RDS_PORT1_PORT.PIN0CTRL = 0;
		}
		else
		{
			// Get First ExInt Time
			Port1.First_ExInt_us = TCB0.CNT;
			Port1.First_ExInt_ms = SysTick;
			Port1.FirstToSecond = true;
		}
		// Clear interrupt flag
		RDS_PORT1_PORT.INTFLAGS &= RDS_PORT1_PIN;
	}
	
	/************************************************************************/
	/* RDS_PORT2 EXINT                                                      */
	/************************************************************************/
	else if (RDS_PORT2_PORT.INTFLAGS & RDS_PORT2_PIN)
	{
		// Check First or Second Interrupt
		if (Port2.FirstToSecond)
		{
			// Get Second ExInt Time
			Port2.Second_ExInt_us = TCB0.CNT;
			Port2.Second_ExInt_ms = SysTick;
			RDS_PORT2_PORT.PIN1CTRL = 0;
		}
		else
		{
			// Get First ExInt Time
			Port2.First_ExInt_us = TCB0.CNT;
			Port2.First_ExInt_ms = SysTick;
			Port2.FirstToSecond = true;
		}
		// Clear interrupt flag
		RDS_PORT2_PORT.INTFLAGS &= RDS_PORT2_PIN;
	}
}

ISR(TCB0_INT_vect)
{
	SysTick++;
	/* Clear the Interrupt flag */
	TCB0.INTFLAGS = TCB_CAPT_bm;
}

ISR(ADC0_RESRDY_vect)
{
	// Get ADC Result
	ADC_Result[ADC_Index] = ADC0.RES;
	// Switch to Next
	if (++ADC_Index > 2)
		ADC_Index = 0;
	// Clear Interrupt Flag
	ADC0.INTFLAGS = ADC_RESRDY_bm;
	// Choose Next Channel
	ADC0.MUXPOS  = ADC_Channel[ADC_Index];
	// Start Converting
	ADC0.COMMAND = ADC_STCONV_bm;
}


