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

const char *BuildVersion = "V0.1";
const char *BuildData = __DATE__;
const char *BuildTime = __TIME__;

#define EEPROM_ADDRESS_IS_NEW_DEVICE 0

#define EEPROM_ADDRESS_PORT1_OFFSET 2
#define EEPROM_ADDRESS_PORT2_OFFSET 17

#define EEPROM_ADDRESS_PORT1_INIT_FROM_EEPROM (NULL + EEPROM_ADDRESS_PORT1_OFFSET)
#define EEPROM_ADDRESS_PORT2_INIT_FROM_EEPROM (NULL + EEPROM_ADDRESS_PORT2_OFFSET)

#define EEPROM_ADDRESS_PORT1_DATA_START_ADDRESS (NULL + EEPROM_ADDRESS_PORT1_OFFSET + 1)
#define EEPROM_ADDRESS_PORT2_DATA_START_ADDRESS (NULL + EEPROM_ADDRESS_PORT2_OFFSET + 1)

#define EEPROM_ADDRESS_PORT_CONFIG 0
#define EEPROM_ADDRESS_PORT_DATA1 1
#define EEPROM_ADDRESS_PORT_DATA2 2
#define EEPROM_ADDRESS_PORT_DATA3 6
#define EEPROM_ADDRESS_PORT_DATA4 10

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/
BSP_Pulse_Form_t PulseForm;
uint32_t SysTick;

uint32_t TWI_Last_Active_Time;

uint8_t TWI_TX_Data_Buff[16];
uint8_t TWI_RX_Data_Buff[8];
uint8_t SRAM_DataBuff[32];

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/

void TWI_Configuration(void);

void TWI_Slave_Read(int ByteLength);
uint8_t TWI_Slave_Write(void);

void EEPROM_Clean_All(void);
void Port_Init_From_EEPROM(BSP_Port_t *pPort);

void EEPROM_Write(uint8_t address, uint8_t data)
{
	while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
}
uint8_t EEPROM_Read(uint8_t address)
{
	while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
	return *((uint8_t *)MAPPED_EEPROM_START + address);
}

/************************************************************************/
/* Main Startup Function                                                */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* GPIO Initialization                                                  */
	/************************************************************************/
	/* RDS_PORT1 and RDS_PORT2 do PullUp */
	RDS_PORT1_PORT.PIN0CTRL = PORT_PULLUPEN_bm;
	RDS_PORT2_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
	/* LED_PORT is Output */
	LED_PORT.DIRSET = LED_PIN;
	/* WDS_PORT1 and WDS_PORT2 are Output */
	WDS_PORT1_PORT.DIRSET = WDS_PORT1_PIN;
	WDS_PORT2_PORT.DIRSET = WDS_PORT2_PIN;
	/* MC60_VBAT_EN Set to High and Output */
	MC60_VBAT_EN_PORT.OUTSET = MC60_VBAT_EN_PIN;
	MC60_VBAT_EN_PORT.DIRSET = MC60_VBAT_EN_PIN;
	/* MC60_PWRKEY is Output */
	MC60_PWRKEY_PORT.DIRSET = MC60_PWRKEY_PIN;
	/************************************************************************/
	/* System Clock Initialization                                          */
	/************************************************************************/
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = CLKCTRL_PEN_bm;
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
	ADC0.CTRLC = ADC_PRESC_DIV32_gc;	 // ADC Prescaler 32X
	/************************************************************************/
	/* TWI Initialization                                                   */
	/************************************************************************/
	TWI0.SADDR = 0x10; // TWI Slave Address
	TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;
	/************************************************************************/
	/* Enable global interrupts                                             */
	/************************************************************************/
	sei();
	/************************************************************************/
	/* MC60_VBAT Power On                                                   */
	/************************************************************************/
	MC60_VBAT_EN_PORT.OUTCLR = MC60_VBAT_EN_PIN;
	_delay_ms(100); // Make sure that VBAT is stable before pulling down PWRKEY pin
	/************************************************************************/
	/* MC60 Press Power Key                                                 */
	/************************************************************************/
	MC60_PWRKEY_PORT.OUTSET = MC60_PWRKEY_PIN; // Press
	_delay_ms(1000); // Hold
	MC60_PWRKEY_PORT.OUTCLR = MC60_PWRKEY_PIN; // Back
	/************************************************************************/
	/* Clean EEPROM if is this New Device                                   */
	/************************************************************************/
	if (eeprom_read_byte((const uint8_t *)0))
		for (uint8_t i = 0; i < 128; i++)
			eeprom_write_byte(((uint8_t *)0) + i, 0);
			
	
	//if (eeprom_read_byte(EEPROM_ADDRESS_PORT1_INIT_FROM_EEPROM))
		//Port_Init_From_EEPROM(&BSP_Port1);
	//else
		//BSP_Port_Configuration(&BSP_Port1, DIGITAL_INPUT);
		//
	//if (eeprom_read_byte(EEPROM_ADDRESS_PORT2_INIT_FROM_EEPROM))
		//Port_Init_From_EEPROM(&BSP_Port2);
	//else
		//BSP_Port_Configuration(&BSP_Port2, DIGITAL_INPUT);
	
	//eeprom_write_byte(1, 3);
	//PulseForm.NumberOfPulse = eeprom_read_byte(1);
	//PulseForm.SetTimeMs = 100;
	//PulseForm.ResetTimeMs = 100;
	//PulseForm.DeadTimeMs = 1000;
	//BSP_LED_Set_Blinking_Form(PulseForm);
	
	//BSP_Port_Configuration(&BSP_Port1, FREQUENCY_INPUT);
	//BSP_Port_Configuration(&BSP_Port2, FREQUENCY_OUTPUT);
	//
	//BSP_Port_Set_Frequency(&BSP_Port2, 10);
	
	while (1)
	{
		/* Watchdog Timer Restart */
		asm("WDR");
		/* BSP Loop */
		//BSP_Loop();
		
		//uint32_t frq = BSP_Port_Get_Frequency(&BSP_Port1);
		//BSP_Port_Set_Frequency(&BSP_Port2, frq);
	}
}


/************************************************************************/
/* Timer Configuration                                                  */
/************************************************************************/
void Timer_Configuration(void)
{
	TCB0.CTRLA = TCB_ENABLE_bm;
	TCB0.INTCTRL = TCB_CAPT_bm;
	TCB0.CCMP = 9999;
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
	
}

/************************************************************************/
/* TWI0 - Slave Write                                                   */
/************************************************************************/
uint8_t TWI_Slave_Write(void)
{
	return sizeof(TWI_TX_Data_Buff);
}

/************************************************************************/
/* WDT Restart                                                          */
/************************************************************************/
void WDT_Restart(void)
{
	
}

uint32_t micros(void)
{
	return (TCB0.CNT / 10) + (SysTick * 1000);
}

uint32_t millis(void)
{
	return SysTick;
}

void EEPROM_Clean_All(void)
{
	for (uint8_t i = 0; i < 128; i++)
	{
		eeprom_write_byte(NULL + i, 0x00);
	}
}

void Port_Init_From_EEPROM(BSP_Port_t *pPort)
{
	if (pPort == &BSP_Port1)
		eeprom_read_block(SRAM_DataBuff, EEPROM_ADDRESS_PORT1_DATA_START_ADDRESS, 15);
	else if (pPort == &BSP_Port2)
		eeprom_read_block(SRAM_DataBuff, EEPROM_ADDRESS_PORT2_DATA_START_ADDRESS, 15);
	
	BSP_Port_Config_t ConfigTo = SRAM_DataBuff[EEPROM_ADDRESS_PORT_CONFIG];
	
	uint8_t data1 = SRAM_DataBuff[EEPROM_ADDRESS_PORT_DATA1];
	uint32_t data2 = *((uint32_t *)(&SRAM_DataBuff + EEPROM_ADDRESS_PORT_DATA2));
	uint32_t data3 = *((uint32_t *)(&SRAM_DataBuff + EEPROM_ADDRESS_PORT_DATA3));
	uint32_t data4 = *((uint32_t *)(&SRAM_DataBuff + EEPROM_ADDRESS_PORT_DATA4));
	
	PulseForm.NumberOfPulse = data1;
	PulseForm.SetTimeMs   = data2;
	PulseForm.ResetTimeMs = data3;
	PulseForm.DeadTimeMs  = data4;
		
	BSP_Port_Configuration(pPort, ConfigTo);
	
	switch (ConfigTo)
	{
	case DIGITAL_OUTPUT:
		BSP_Port_Set_Digital_Signal(pPort, data1);
		break;
	case PWM_OUTPUT:
		BSP_Port_Set_PWM(pPort, data2, data3);
		break;
	case FREQUENCY_OUTPUT:
		BSP_Port_Set_Frequency(pPort, data2);
		break;
	case REGULAR_PULSE_OUTPUT:
		BSP_Port_Set_Frequency(pPort, data2);
		break;
	default:
		break;
	}
}

ISR(TCB0_INT_vect)
{
	SysTick++;
	TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
}



