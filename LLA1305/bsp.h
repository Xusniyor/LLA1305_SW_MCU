/*
 * bsp.h
 *
 * Created: 04.06.2023 14:22:19
 *  Author: Xusniyor
 */

#ifndef BSP_H_
#define BSP_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "main.h"

typedef enum
{
	PULSE_STATE_SET,
	PULSE_STATE_RESET,
	PULSE_STATE_DEAD
} BSP_Pulse_State_t;

typedef enum
{
	EXINT_FIRST,
	EXINT_SECOND
} BSP_EXINT_State_t;

typedef enum
{
	DIGITAL_INPUT = 0,
	DIGITAL_OUTPUT,
	ANALOG_INPUT,
	PWM_OUTPUT,
	FREQUENCY_INPUT,
	FREQUENCY_OUTPUT,
	ONE_PULSE_OUTPUT,
	REGULAR_PULSE_OUTPUT,
	NOT_ACTIVE
} BSP_Port_Config_t;

typedef struct
{
	/* private */
	BSP_Pulse_State_t State;
	uint32_t SwitchingTime;
	uint8_t CountPulse;
	
	/* public */
	uint8_t NumberOfPulse;
	uint32_t SetTimeMs;
	uint32_t ResetTimeMs;
	uint32_t DeadTimeMs;
} BSP_Pulse_Form_t;

typedef struct
{
	PORT_t *pPORT_DIN;
	uint8_t PIN_DIN;
	
	PORT_t *pPORT_DOUT;
	uint8_t PIN_DOUT;
	
	PORT_t *pPORT_AIN;
	uint8_t PIN_AIN;
	
	ADC_MUXPOS_t ADC_Channel;
	BSP_Port_Config_t Port_Config;
	BSP_Pulse_Form_t Pulse_Form;
	
	uint32_t Frequency;
	uint32_t First_EXINT_TimeUs;
	uint32_t Second_EXINT_TimeUs;
	uint32_t Last_EXINT_Check_Time;
	BSP_EXINT_State_t EXINT_State;
} BSP_Port_t;

/************************************************************************/
/* Prototype functions                                                  */
/************************************************************************/
void BSP_Init(void);
void BSP_Loop(void);

void BSP_Port_Configuration(BSP_Port_t *pPort, BSP_Port_Config_t ConfigTo);

void BSP_Port_Set_PWM(BSP_Port_t *pPort, uint32_t Frequency, uint8_t DutyCycle);
uint16_t BSP_Port_Read_Analog_Signal(BSP_Port_t *pPort);

void BSP_Port_Set_Digital_Signal(BSP_Port_t *pPort, bool State);
bool BSP_Port_Read_Digital_Signal(BSP_Port_t *pPort);

void BSP_Port_Set_Frequency(BSP_Port_t *pPort, uint32_t Frequency);
uint32_t BSP_Port_Get_Frequency(BSP_Port_t *pPort);

void BSP_Port_Set_One_Pulse(BSP_Port_t *pPort, BSP_Pulse_Form_t PulseForm);
void BSP_Port_Set_Regular_Pulse(BSP_Port_t *pPort, BSP_Pulse_Form_t PulseForm);

/************************************************************************/
/* These Functions are used to connect and disconnect                   */
/* the MC60 VBAT from the power source                                  */
/************************************************************************/
void BSP_MC60_VBAT_Power_On(void);
void BSP_MC60_VBAT_Power_Off(void);

/************************************************************************/
/* This function is used to turn the MC60 on or off                     */
/* after it is connected to a power source                              */
/* You Should Hold for Turn-On 1100ms or Turn-Off 800ms                 */
/************************************************************************/
void BSP_MC60_Press_Power_Key(uint16_t HoldTimeMs);

/************************************************************************/
/* LED                                                                  */
/************************************************************************/
void BSP_LED_Set_Blinking_Form(BSP_Pulse_Form_t PulseForm);

extern BSP_Port_t BSP_Port1;
extern BSP_Port_t BSP_Port2;

#endif /* BSP_H_ */
