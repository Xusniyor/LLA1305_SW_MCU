/*
 * bsp.c
 *
 * Created: 04.06.2023 14:22:33
 *  Author: Xusniyor
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "bsp.h"

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

BSP_Port_t BSP_Port1 = 
{
	.pPORT_DOUT = &WDS_PORT1_PORT,
	.PIN_DOUT = WDS_PORT1_PIN,
	
	.pPORT_DIN = &RDS_PORT1_PORT,
	.PIN_DIN = RDS_PORT1_PIN,
	
	.ADC_Channel = RAS_PORT1
};

BSP_Port_t BSP_Port2 =
{
	.pPORT_DOUT = &WDS_PORT2_PORT,
	.PIN_DOUT = WDS_PORT2_PIN,
	
	.pPORT_DIN = &RDS_PORT2_PORT,
	.PIN_DIN = RDS_PORT2_PIN,
	
	.ADC_Channel = RAS_PORT2
};

BSP_Pulse_Form_t LED_Pulse_Form =
{
	.NumberOfPulse = 2,
	.SetTimeMs = 100,
	.ResetTimeMs = 100,
	.DeadTimeMs = 800,
	
	.State = PULSE_STATE_SET
};

void BSP_Init(void)
{

}

void BSP_Port_Configuration(BSP_Port_t *pPort, BSP_Port_Config_t ConfigTo)
{
	if (pPort == NULL)
		return;
		
	if (ConfigTo == FREQUENCY_OUTPUT)
		BSP_Port_Configuration(pPort, PWM_OUTPUT);
	
	switch (ConfigTo)
	{
	case DIGITAL_INPUT:
	{
		break;
	}
	case DIGITAL_OUTPUT:
	{
		pPort->pPORT_DOUT->OUTCLR = pPort->PIN_DOUT;
		break;
	}
	case PWM_OUTPUT:
	{
		TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
		TCA0.SINGLE.PER = 6250;
		if (pPort == &BSP_Port1)
		{
			PORTMUX.CTRLC = PORTMUX_TCA01_bm;
			TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP1_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
			TCA0.SINGLE.CTRLC = TCA_SINGLE_CMP1OV_bm;
			TCA0.SINGLE.CMP1 = 0;
		}
		else if (pPort == &BSP_Port2)
		{
			PORTMUX.CTRLC = PORTMUX_TCA02_bm;
			TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP2_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
			TCA0.SINGLE.CTRLC = TCA_SINGLE_CMP2OV_bm;
			TCA0.SINGLE.CMP2 = 0;
		}
		break;
	}
	case ONE_PULSE_OUTPUT:
	{
		pPort->pPORT_DOUT->OUTCLR = pPort->PIN_DOUT;
		pPort->Port_Config = NOT_ACTIVE;
		break;
	}
	case REGULAR_PULSE_OUTPUT:
	{
		pPort->pPORT_DOUT->OUTCLR = pPort->PIN_DOUT;
		pPort->Port_Config = NOT_ACTIVE;
		break;
	}
	case FREQUENCY_INPUT:
	{
		pPort->First_EXINT_TimeUs = 0;
		pPort->Second_EXINT_TimeUs = 0;
		pPort->EXINT_State = EXINT_FIRST;
		
		if (pPort == &BSP_Port1)
			BSP_Port1.pPORT_DIN->PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
		else if (pPort == &BSP_Port2)
			BSP_Port2.pPORT_DIN->PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
			
		break;
	}
	case ANALOG_INPUT:
	{
		break;
	}
	default:
		return;
	}
	
	pPort->Port_Config = ConfigTo;
}

void BSP_Port_Set_PWM(BSP_Port_t *pPort, uint32_t Frequency, uint8_t DutyCycle)
{
	if (pPort == NULL)
		return;

	if (pPort->Port_Config != PWM_OUTPUT)
		return;

	if ((Frequency > 1000) || (Frequency < 10) || (DutyCycle > 100))
		return;

	TCA0.SINGLE.PER = 625000UL / Frequency;

	if (pPort == &BSP_Port1)
		TCA0.SINGLE.CMP1 = ((float)(TCA0.SINGLE.PER) / 100) * DutyCycle;
	else if (pPort == &BSP_Port2)
		TCA0.SINGLE.CMP2 = ((float)(TCA0.SINGLE.PER) / 100) * DutyCycle;

	pPort->Frequency = Frequency;
}

uint16_t BSP_Port_Read_Analog_Signal(BSP_Port_t *pPort)
{
	if (pPort == NULL)
		return 0;
	return ADC_ReadAnalogSignal(pPort->ADC_Channel);
}

void BSP_Port_Set_Digital_Signal(BSP_Port_t *pPort, bool State)
{
	if (pPort == NULL)
		return;
		
	if (pPort->Port_Config != DIGITAL_OUTPUT)
		return;
		
	if (State)
		pPort->pPORT_DOUT->OUTSET = pPort->PIN_DOUT;
	else
		pPort->pPORT_DOUT->OUTCLR = pPort->PIN_DOUT;
}

bool BSP_Port_Read_Digital_Signal(BSP_Port_t *pPort)
{
	if (pPort == NULL)
		return 0;
		
	return (pPort->pPORT_DIN->IN & pPort->PIN_DIN) ? false : true;
}

void BSP_Port_Set_Frequency(BSP_Port_t *pPort, uint32_t Frequency)
{
	BSP_Port_Set_PWM(pPort, Frequency, 50);
}

uint32_t BSP_Port_Get_Frequency(BSP_Port_t *pPort)
{
	if (pPort == NULL)
		return 0;
		
	if (pPort->Port_Config != FREQUENCY_INPUT)
		return 0;
		
	return pPort->Frequency;
}

void BSP_Port_Set_One_Pulse(BSP_Port_t *pPort, BSP_Pulse_Form_t PulseForm)
{
	if (pPort == NULL)
		return;
	
	pPort->Pulse_Form = PulseForm;
	
	if (pPort->Port_Config == NOT_ACTIVE)
	{
		pPort->Port_Config = ONE_PULSE_OUTPUT;
		pPort->pPORT_DOUT->OUTSET = pPort->PIN_DOUT;
		pPort->Pulse_Form.State = PULSE_STATE_SET;
	}
}

void BSP_Port_Set_Regular_Pulse(BSP_Port_t *pPort, BSP_Pulse_Form_t PulseForm)
{
	if (pPort == NULL)
		return;
	
	pPort->Pulse_Form = PulseForm;
	
	if (pPort->Port_Config == NOT_ACTIVE)
	{
		pPort->Port_Config = REGULAR_PULSE_OUTPUT;
		pPort->pPORT_DOUT->OUTSET = pPort->PIN_DOUT;
		pPort->Pulse_Form.State = PULSE_STATE_SET;
	}
}

/************************************************************************/
/* These Functions are used to connect and disconnect                   */
/* the MC60 VBAT from the power source                                  */
/************************************************************************/
void BSP_MC60_VBAT_Power_On(void)
{
	MC60_VBAT_EN_PORT.OUTCLR = MC60_VBAT_EN_PIN; // Power_On
	_delay_ms(100);								 // Make sure that VBAT is stable before pulling down PWRKEY pin
}
void BSP_MC60_VBAT_Power_Off(void)
{
	MC60_VBAT_EN_PORT.OUTSET = MC60_VBAT_EN_PIN; // Power_On
	_delay_ms(1000);							 // Wait for VBAT to completely power down
}

/************************************************************************/
/* This function is used to turn the MC60 on or off                     */
/* after it is connected to a power source                              */
/* You Should Hold for Turn-On 1100ms or Turn-Off 800ms                 */
/************************************************************************/
void BSP_MC60_Press_Power_Key(uint16_t HoldTimeMs)
{
	MC60_PWRKEY_PORT.OUTSET = MC60_PWRKEY_PIN; // Press
	for (uint16_t i = 0; i < HoldTimeMs; i++)
		_delay_ms(1);						   // Hold
	MC60_PWRKEY_PORT.OUTCLR = MC60_PWRKEY_PIN; // Back
}

void BSP_LED_Set_Blinking_Form(BSP_Pulse_Form_t PulseForm)
{
	LED_Pulse_Form = PulseForm;
}

void BSP_Pulse_Task(BSP_Pulse_Form_t *pPulse, PORT_t *pPORT, uint8_t PIN)
{
	if (pPulse->State == PULSE_STATE_SET)
	{
		if (millis() - pPulse->SwitchingTime > pPulse->SetTimeMs)
		{
			pPulse->SwitchingTime = millis();
			pPORT->OUTCLR = PIN;
			pPulse->CountPulse++;
			pPulse->State = PULSE_STATE_RESET;
		}
	}
	else if (pPulse->State == PULSE_STATE_RESET)
	{
		if (millis() - pPulse->SwitchingTime > pPulse->ResetTimeMs)
		{
			pPulse->SwitchingTime = millis();
			if (pPulse->NumberOfPulse > pPulse->CountPulse)
			{
				pPORT->OUTSET = PIN;
				pPulse->State = PULSE_STATE_SET;
			}
			else
			{
				pPulse->State = PULSE_STATE_DEAD;
				pPulse->CountPulse = 0;
			}
		}
	}
	else if (pPulse->State == PULSE_STATE_DEAD)
	{
		if (millis() - pPulse->SwitchingTime > pPulse->DeadTimeMs)
		{
			pPulse->SwitchingTime = millis();
			pPORT->OUTSET = PIN;
			pPulse->State = PULSE_STATE_SET;
		}
	}
}

bool BSP_EXINT_Task(BSP_Port_t *pPort)
{
	bool EXINT_Restart = false;
	
	if (pPort->Port_Config == FREQUENCY_INPUT)
	{
		if (millis() - pPort->Last_EXINT_Check_Time > 2000)
		{
			pPort->Last_EXINT_Check_Time = millis();
			
			if (pPort->EXINT_State == EXINT_SECOND)
			{
				if (pPort->Second_EXINT_TimeUs == 0)
				{
					if (micros() - pPort->First_EXINT_TimeUs > 1000000UL)
					{
						pPort->First_EXINT_TimeUs = 0;
						pPort->Second_EXINT_TimeUs = 0;
						pPort->EXINT_State = EXINT_FIRST;
						EXINT_Restart = true;
					}
				}
				else
				{
					pPort->Frequency = (uint32_t)(1000000UL / (uint32_t)(pPort->Second_EXINT_TimeUs - pPort->First_EXINT_TimeUs)) + 1;
					
					pPort->First_EXINT_TimeUs = 0;
					pPort->Second_EXINT_TimeUs = 0;
					pPort->EXINT_State = EXINT_FIRST;
					EXINT_Restart = true;
				}
			}
		}
	}
	
	return EXINT_Restart;
}

void BSP_Loop(void)
{
	/************************************************************************/
	/* LED                                                                  */
	/************************************************************************/
	BSP_Pulse_Task(&LED_Pulse_Form, &LED_PORT, LED_PIN);
	
	/************************************************************************/
	/* Pulse Task PORT 1                                                    */
	/************************************************************************/
	if (BSP_Port1.Port_Config == REGULAR_PULSE_OUTPUT)
	{
		BSP_Pulse_Task(&BSP_Port1.Pulse_Form, BSP_Port1.pPORT_DOUT, BSP_Port1.PIN_DOUT);
	}
	else if (BSP_Port1.Port_Config == ONE_PULSE_OUTPUT)
	{
		BSP_Pulse_Task(&BSP_Port1.Pulse_Form, BSP_Port1.pPORT_DOUT, BSP_Port1.PIN_DOUT);
		if (BSP_Port1.Pulse_Form.State == PULSE_STATE_DEAD)
			BSP_Port1.Port_Config = NOT_ACTIVE;
	}
	
	/************************************************************************/
	/* Pulse Task PORT 2                                                    */
	/************************************************************************/
	if (BSP_Port2.Port_Config == REGULAR_PULSE_OUTPUT)
	{
		BSP_Pulse_Task(&BSP_Port2.Pulse_Form, BSP_Port2.pPORT_DOUT, BSP_Port2.PIN_DOUT);
	}
	else if (BSP_Port2.Port_Config == ONE_PULSE_OUTPUT)
	{
		BSP_Pulse_Task(&BSP_Port2.Pulse_Form, BSP_Port2.pPORT_DOUT, BSP_Port2.PIN_DOUT);
		if (BSP_Port2.Pulse_Form.State == PULSE_STATE_DEAD)
			BSP_Port2.Port_Config = NOT_ACTIVE;
	}
	
	/************************************************************************/
	/* EXINT Task                                                           */
	/************************************************************************/
	if (BSP_EXINT_Task(&BSP_Port1) == true)
		// Restart EXINT
		BSP_Port1.pPORT_DIN->PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
	if (BSP_EXINT_Task(&BSP_Port2) == true)
		// Restart EXINT
		BSP_Port2.pPORT_DIN->PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

ISR(PORTC_PORT_vect)
{
	// check interrupt pin
	if (BSP_Port1.pPORT_DIN->INTFLAGS & BSP_Port1.PIN_DIN)
	{
		if (BSP_Port1.EXINT_State == EXINT_FIRST)
		{
			BSP_Port1.First_EXINT_TimeUs = micros();
			BSP_Port1.EXINT_State = EXINT_SECOND;
		}
		else
		{
			BSP_Port1.Second_EXINT_TimeUs = micros();
			// Disable Interrupt
			BSP_Port1.pPORT_DIN->PIN0CTRL = 0x00;
		}
		// Clear interrupt flag
		BSP_Port1.pPORT_DIN->INTFLAGS &= BSP_Port1.PIN_DIN;
	}
	else if (BSP_Port2.pPORT_DIN->INTFLAGS & BSP_Port2.PIN_DIN)
	{
		if (BSP_Port2.EXINT_State == EXINT_FIRST)
		{
			BSP_Port2.First_EXINT_TimeUs = micros();
			BSP_Port2.EXINT_State = EXINT_SECOND;
		}
		else
		{
			BSP_Port2.Second_EXINT_TimeUs = micros();
			// Disable Interrupt
			BSP_Port2.pPORT_DIN->PIN0CTRL = 0x00;
		}
		// Clear interrupt flag
		BSP_Port2.pPORT_DIN->INTFLAGS &= BSP_Port2.PIN_DIN;
	}
}
