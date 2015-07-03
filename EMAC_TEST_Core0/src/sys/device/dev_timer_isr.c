/*
 * (C) Copyright 2012 - Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary and confidential.  By using this software
 * you agree to the terms of the associated Analog Devices License Agreement.
 *
 * Project Name:  	Power_On_Self_Test
 *
 * Hardware:		ADSP-BF609 EZ-Board
 *
 * Description:	This file implements timers on the EZ-Board.
 */

#include <ccblkfn.h>
#include <adi_types.h>
#include <services/tmr/adi_tmr.h>
#include <services/tmr/adi_tmr_bf6xx.h>
#include <services/int/adi_int.h>
#include <services/int/adi_sec.h>
#include <services/gpio/adi_gpio.h>

//#include "post_common.h"
#include "post_debug.h"
#include "dev_timer_isr.h"

#include "task.h"
#include "dev_pwr.h"

#include "comm_pc_protocol.h"
#include "msg.h"


#if defined(__DEBUG_FILE__)
#include <string.h>
extern FILE *pDebugFile;				/* debug file when directing output to a file */
#endif

#define MAX_NUM_COUNTDOWN_TIMERS 5

typedef struct CountDownTimer_TAG
{
	bool m_IsActive;
	unsigned long m_ulTimeoutCounter;
}countdowntimer;

static volatile unsigned long g_ulTickCount;
static countdowntimer sCountDownTimer[MAX_NUM_COUNTDOWN_TIMERS] = { {0,0},{0,0},{0,0},{0,0},{0,0} };

/* Memory required for opening the timer */
uint8_t TimerMemory[ADI_TMR_MEMORY];

/* Timer callback function */
static void TimerHandler(void *pCBParam, uint32_t Event, void *pArg);

#define TEST_TIMER0_NUM      1
//1ms
#define TIMER0_PERIOD        (SYS_CLKIN*MULTIPLIER_SEL/ (1 + DF_SEL) /SYSCLK_SEL/SCLK0_SEL)/1000
#define TIMER0_WIDTH         (TIMER0_PERIOD/2)
#define TIMER0_DELAY          (TIMER0_WIDTH/2)


/* Timer handle */
static ADI_TMR_HANDLE ghTimer = 0;


bool EnableGPTimer(bool bEnable)
{
	ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;
	if(ghTimer)
	{
		/* Enable the timer*/
		if((eTmrResult = adi_tmr_Enable(
				ghTimer,
				bEnable)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to enable the timer 0x%08X \n\n", eTmrResult);
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return false;
	}
}



/*******************************************************************
*   Function:    Init_Timer_Interrupts
*   Description: This function initializes the interrupts for Timer0
*******************************************************************/
void Init_Timer_Interrupts(void)
{
    ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;

    if(!ghTimer)
    {
		/* Open the timer */
		if( (eTmrResult = adi_tmr_Open (
				TEST_TIMER0_NUM,
				TimerMemory,
				ADI_TMR_MEMORY,
				TimerHandler,
				NULL,
				&ghTimer)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Timer open failed 0x%08X \n", eTmrResult);
		}

		/* Set the mode to PWM OUT */
		if((eTmrResult = adi_tmr_SetMode(
				ghTimer,
				ADI_TMR_MODE_CONTINUOUS_PWMOUT)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to open timer in PWM out mode 0x%08X \n\n", eTmrResult);
		}

		/* Set the IRQ mode to get interrupt after timer counts to Delay + Width */
		if((eTmrResult = adi_tmr_SetIRQMode(
				ghTimer,
				ADI_TMR_IRQMODE_WIDTH_DELAY)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer IRQ mode 0x%08X \n\n", eTmrResult);
		}

		/* Set the Period */
		if((eTmrResult = adi_tmr_SetPeriod(
				ghTimer,
				TIMER0_PERIOD)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer Period 0x%08X \n\n", eTmrResult);
		}

		/* Set the timer width */
		if((eTmrResult = adi_tmr_SetWidth(
				ghTimer,
				TIMER0_WIDTH)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer Width 0x%08X \n\n", eTmrResult);
		}

		/* Set the timer Delay */
		if((eTmrResult = adi_tmr_SetDelay(
				ghTimer,
				TIMER0_DELAY)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer Delay 0x%08X \n\n", eTmrResult);
		}

		/* Enable the timer to stop gracefully */
		if((eTmrResult = adi_tmr_EnableGracefulStop(
				ghTimer,
				true)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to enable the timer to stop gracefully 0x%08X \n\n", eTmrResult);
		}
    }
    /* Enable the timer*/
    if((eTmrResult = adi_tmr_Enable(
            ghTimer,
            true)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to enable the timer 0x%08X \n\n", eTmrResult);
    }
}


/*******************************************************************
*   Function:    Delay
*   Description: Delay for a fixed number of Ms, blocks
*******************************************************************/
void Delay(const unsigned long ulMs)
{
	unsigned int uiTIMASK = cli();

	g_ulTickCount = 0;
	unsigned long ulEnd = (g_ulTickCount + ulMs);

	sti(uiTIMASK);

    while( g_ulTickCount < ulEnd )
	{
		asm("nop;");
	}
}


/*******************************************************************
*   Function:    SetTimeout
*   Description: Set a value for a global timeout, return the timer
*******************************************************************/
unsigned int SetTimeout(const unsigned long ulTicks)
{
	unsigned int uiTIMASK = cli();
	unsigned int n;

	/* we don't care which countdown timer is used, so search for a free
		timer structure */
	for( n = 0;  n < MAX_NUM_COUNTDOWN_TIMERS; n++ )
	{
		if( false == sCountDownTimer[n].m_IsActive )
		{
			sCountDownTimer[n].m_IsActive = true;
			sCountDownTimer[n].m_ulTimeoutCounter = ulTicks;

			sti(uiTIMASK);
			return n;
		}
	}

	sti(uiTIMASK);
	return ((unsigned int)-1);
}

/*******************************************************************
*   Function:    ClearTimeout
*   Description: Set a value for a global timeout, return the timer
*******************************************************************/
unsigned long ClearTimeout(const unsigned int nIndex)
{
	unsigned int uiTIMASK = cli();
	unsigned long ulTemp = (unsigned int)(-1);

	if( nIndex < MAX_NUM_COUNTDOWN_TIMERS )
	{
		/* turn off the timer */
		ulTemp = sCountDownTimer[nIndex].m_ulTimeoutCounter;
		sCountDownTimer[nIndex].m_ulTimeoutCounter = 0;
		sCountDownTimer[nIndex].m_IsActive = false;
	}

	sti(uiTIMASK);
	return (ulTemp);
}

/*******************************************************************
*   Function:    IsTimedout
*   Description: Checks to see if the timeout value has expired
*******************************************************************/
bool IsTimedout(const unsigned int nIndex)
{
	unsigned int uiTIMASK = cli();
	if( nIndex < MAX_NUM_COUNTDOWN_TIMERS )
	{
		sti(uiTIMASK);
		return ( 0 == sCountDownTimer[nIndex].m_ulTimeoutCounter );
	}

	sti(uiTIMASK);
	return 0;
}

/*
  *   Function:    void Timer_ISR( void *, uint_32, void *)
 *   Description: Timer ISR
 */
static void TimerHandler(void *pCBParam, uint32_t Event, void *pArg)
{
	unsigned int n;
	
	g_ulTickCount++;
//	DEBUG_PRINT("%d\n\n",g_nISRCounter);

//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

	/* decrement each counter if it is non-zero */
	for( n = 0; n < MAX_NUM_COUNTDOWN_TIMERS; n++ )
	{
		if( 0 != sCountDownTimer[n].m_ulTimeoutCounter )
		{
			sCountDownTimer[n].m_ulTimeoutCounter--;
		}
	}
}


/*******************************************************************
*   Function:    Init_Timer_Interrupts
*   Description: This function initializes the interrupts for Timer0
*******************************************************************/
/* Timer pulse width, delay and period parameters */
#include "dri_ptp_engine.h"
#include "dev_pwr.h"



// ²ÉÑù¼ä¸ô
#define TIMER5_PERIOD        ((SYS_CLKIN*MULTIPLIER_SEL/ (1 + DF_SEL) /SYSCLK_SEL/SCLK0_SEL)/SMP_RATE)

#define TIMER5_WIDTH        (TIMER5_PERIOD/5)

#define TIMER5_DELAY         (10)


#define MAX_NUM_CLOCK_TICK_EVENT_HANDLERS 5

typedef struct CLOCK_TICK_EVENT_HANDLER_TAG
{
	bool m_IsActive;
	Timer_Event_Handler m_pHandler;
}CLOCK_TICK_EVENT_HANDLER;


static CLOCK_TICK_EVENT_HANDLER sClkTickHandlers[MAX_NUM_CLOCK_TICK_EVENT_HANDLERS] = { {0,0},{0,0},{0,0},{0,0},{0,0} };


/* Timer to be used in the example */
#define GP_TIMER5_NUM        5

/* Timer handle */
ADI_TMR_HANDLE   ghTimer5;

/* Memory required for opening the timer */
uint8_t TimerMemory5[ADI_TMR_MEMORY];

static void Timer5_ISR(void *pCBParam, uint32_t Event, void *pArg);

void Init_GP_Timer5(void)
{
    ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;

    if(!ghTimer5)
    {
		/* Open the timer */
		if( (eTmrResult = adi_tmr_Open (
				GP_TIMER5_NUM,
				TimerMemory5,
				ADI_TMR_MEMORY,
				Timer5_ISR,
				NULL,
				&ghTimer5)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Timer open failed 0x%08X \n", eTmrResult);
		}

		/* Set the mode to PWM OUT */
		if((eTmrResult = adi_tmr_SetMode(
				ghTimer5,
				ADI_TMR_MODE_CONTINUOUS_PWMOUT)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to open timer in PWM out mode 0x%08X \n\n", eTmrResult);
		}

		/* Set the IRQ mode to get interrupt after timer counts to Delay + Width */
		if((eTmrResult = adi_tmr_SetIRQMode(
				ghTimer5,
				ADI_TMR_IRQMODE_WIDTH_DELAY)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer IRQ mode 0x%08X \n\n", eTmrResult);
		}

		/* Set the Period */
		if((eTmrResult = adi_tmr_SetPeriod(
				ghTimer5,
				TIMER5_PERIOD)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer Period 0x%08X \n\n", eTmrResult);
		}

		/* Set the timer width */
		if((eTmrResult = adi_tmr_SetWidth(
				ghTimer5,
				TIMER5_WIDTH)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer Width 0x%08X \n\n", eTmrResult);
		}

		/* Set the timer Delay */
		if((eTmrResult = adi_tmr_SetDelay(
				ghTimer5,
				TIMER5_DELAY)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to set the timer Delay 0x%08X \n\n", eTmrResult);
		}

		/* Enable the timer to stop gracefully */
		if((eTmrResult = adi_tmr_EnableGracefulStop(
				ghTimer5,
				true)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to enable the timer to stop gracefully 0x%08X \n\n", eTmrResult);
		}

		 /* Enable the timer*/
		if((eTmrResult = adi_tmr_Enable(
				ghTimer5,
				false)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to enable the timer 0x%08X \n\n", eTmrResult);
		}
    }

}

bool EnableGPTimer5(bool bEnable)
{
	ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;
	if(ghTimer5)
	{
		/* Enable the timer*/
		if((eTmrResult = adi_tmr_Enable(
				ghTimer5,
				bEnable)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to enable the timer 0x%08X \n\n", eTmrResult);
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return false;
	}
}


/*
  *   Function:    void Timer_ISR( void *, uint_32, void *)
 *   Description: Timer ISR
 */


static void Timer5_ISR(void *pCBParam, uint32_t Event, void *pArg)
{
	static uint32_t s_SmpCnt = 0;
	unsigned int n;


//	OutputFT3Frm();

//	StandardSmpDataFormatConverter();

	for( n = 0; n < MAX_NUM_CLOCK_TICK_EVENT_HANDLERS; n++ )
	{
		if( sClkTickHandlers[n].m_IsActive )
		{
			sClkTickHandlers[n].m_pHandler(pCBParam, Event, pArg);
		}
	}
}

/*******************************************************************
*   Function:    SetTimerEventHandler
*   Description: Set a Handler for a global timeout, return the timer
*******************************************************************/
unsigned int SetTimerEventHandler(Timer_Event_Handler handler)
{
	unsigned int uiTIMASK = cli();
	unsigned int n;

	/* we don't care which countdown timer is used, so search for a free
		timer structure */
	for( n = 0;  n < MAX_NUM_CLOCK_TICK_EVENT_HANDLERS; n++ )
	{
		if( false == sClkTickHandlers[n].m_IsActive )
		{
			sClkTickHandlers[n].m_IsActive = true;
			sClkTickHandlers[n].m_pHandler = handler;

			sti(uiTIMASK);
			return n;
		}
	}

	sti(uiTIMASK);
	return ((unsigned int)-1);
}

unsigned int CancelTimerEventHandler(Timer_Event_Handler handler)
{
	unsigned int uiTIMASK = cli();
	unsigned int n;

	/* we don't care which countdown timer is used, so search for a free
		timer structure */
	for( n = 0;  n < MAX_NUM_CLOCK_TICK_EVENT_HANDLERS; n++ )
	{
		if( (true == sClkTickHandlers[n].m_IsActive) &&  (handler == sClkTickHandlers[n].m_pHandler) )
		{
			sClkTickHandlers[n].m_IsActive = false;
			sClkTickHandlers[n].m_pHandler = handler;

			sti(uiTIMASK);
			return n;
		}
	}

	sti(uiTIMASK);
	return ((unsigned int)-1);
}
