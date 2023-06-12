/*
 * uart.h
 *
 * Created: 04.06.2023 1:35:39
 *  Author: Xusniyor
 */

#ifndef UART_H_
#define UART_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void UART_Configuration(uint16_t BaudRate);
uint8_t UART_Get_Rx_Data_Len(void);
uint8_t *UART_Get_Rx_Data_Buff(void);
void UART_Clean_Rx_Data(void);
void UART_Send_String(uint8_t *data, uint8_t DataLen);
int UART_Send_Char(char data, FILE *stream);

#endif /* UART_H_ */