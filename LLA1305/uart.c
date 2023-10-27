/*
 * uart.c
 *
 * Created: 04.06.2023 1:35:58
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "uart.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#define UART_RX_DATA_BUFF_LEN 32

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/
//static FILE UART_Stream = FDEV_SETUP_STREAM(UART_Send_Char, NULL, _FDEV_SETUP_WRITE);
uint8_t UART_RX_Data_Buff[UART_RX_DATA_BUFF_LEN];
uint8_t UART_RX_Data_Len;

/************************************************************************/
/* UART - Configuration                                               */
/************************************************************************/
void UART_Configuration(uint16_t BaudRate)
{
	UART_RX_Data_Len = 0;
	PORTB.OUTSET = PIN2_bm;												// PB2 Set Pin to High for TX
	PORTB.DIRSET = PIN2_bm;												// PB2 Set Pin to Output for TX
	PORTB.DIRCLR = PIN3_bm;												// PB3 Set Pin to Input for RX
	PORTB.PIN3CTRL = PORT_PULLUPEN_bm;									// PB3 Enable Pull Up Resistor for RX
	USART0.BAUD = ((float)(F_CPU * 64 / (16 * (float)BaudRate)) + 0.5); // Set Baud Rate
	USART0.CTRLA = USART_RXCIE_bm;										// Enable USART0 RX Data Interrupt
	USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;						// Enable USART0 RX & TX
	//stdout = &UART_Stream;											// for use printf function
}

/************************************************************************/
/* UART - Get Rx Data Len                                               */
/************************************************************************/
uint8_t UART_Get_Rx_Data_Len(void)
{
	return UART_RX_Data_Len;
}

/************************************************************************/
/* UART - Get Rx Data Buff                                              */
/************************************************************************/
uint8_t *UART_Get_Rx_Data_Buff(void)
{
	return UART_RX_Data_Buff;
}

/************************************************************************/
/* UART - Clean Rx Data                                                 */
/************************************************************************/
void UART_Clean_Rx_Data(void)
{
	UART_RX_Data_Len = 0;
}

/************************************************************************/
/* UART - Send 8-Bit Data                                               */
/************************************************************************/
int UART_Send_Char(char data, FILE *stream)
{
	while (!(USART0.STATUS & USART_DREIF_bm)); // Wait Sending Data
	USART0.TXDATAL = data; // Send Data
	return 0;
}

/************************************************************************/
/* UART - Send String                                               */
/************************************************************************/
void UART_Send_String(uint8_t *data, uint8_t DataLen)
{
	for (uint8_t i = 0; i < DataLen; i++)
		UART_Send_Char(data[i], NULL);
}

/************************************************************************/
/* UART - RX Data Interrupt                                             */
/************************************************************************/
ISR(USART0_RXC_vect)
{
	uint8_t data = USART0.RXDATAL;
	if (UART_RX_Data_Len < UART_RX_DATA_BUFF_LEN)
		UART_RX_Data_Buff[UART_RX_Data_Len++] = data; // Read Data
}
