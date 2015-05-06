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
 * Description:	This file initializes and tests the UART on the BNF609 EZ-Board.
 */
#include <adi_osal.h>
#include <cdefbf609.h>
#include <ccblkfn.h>
#include <drivers/uart/adi_uart.h>

#include "dev_uart.h"

#include "dev_timer_isr.h"



/*******************************************************************
*  global var
*******************************************************************/

#if defined(__DEBUG_FILE__)
#include <string.h>
extern FILE *pDebugFile;				/* debug file when directing output to a file */
#endif

#define MAX_TEST_CHARS		5000

/* if we are using the UART to print debug info, define the following */
#ifdef __DEBUG_UART__
#define UART_DEBUG_BUFFER_LINE_SIZE 256
char UART_DEBUG_BUFFER[UART_DEBUG_BUFFER_LINE_SIZE];
#endif




/*
 *   Function:    Init_UART
 *   Description: Initialize UART with the appropriate values
 */
void Init_UART0(void)
{
	static bool bInitialized = false;

	*pREG_PORTD_FER_SET |= 0x180;
	ssync();

	/* configure UART0 RX and UART0 TX pins */
	*pREG_PORTD_MUX |= 0x14000;
	ssync();

	/*****************************************************************************
	 *
	 *  First of all, enable UART clock, put in UART mode, word length 8.
	 *
	 ****************************************************************************/
	*pREG_UART0_CTL = 0x301;//data:8bit, stop:1, parity:no


	/* Bit Rate = 		   SCLK
					-------------------				EDBO is a 1
					[16^(1-EDBO)] * Div

			   ##################################

					125 * 1000000
				  -------------------  = 9600.6
					 [16^0] * 13021
	*/

	//*pREG_UART0_CLK = 0x800032DD;
//	*pREG_UART0_CLK = 0x80005161;//SCLK:200M,9600
//	*pREG_UART0_CLK = 0x80000364;//SCLK0:100M,115200
//	*pREG_UART0_CLK = 0x80008555;//SCLK,163.84m,4800
//	*pREG_UART0_CLK = 0x8000058e;//SCLK,163.84m,115200
//	*pREG_UART0_CLK = 0x800003e4;//SCLK,114.688m,115200
	*pREG_UART0_CLK = 0x8000043d;//SCLK0:125M,115200

	/* reroute tx/rx interrupts to status interrupt output */
	*pREG_UART0_IMSK_SET = 0x180;

}


/*
 *   Function:    PutChar
 *   Description: Writes a character to the UART.
 */
int PutChar(const char cVal)
{
	int nStatus = 0;
	unsigned int nTimer = SetTimeout(1000);
	if( ((unsigned int)-1) != nTimer )
	{
		do{
			if( (*pREG_UART0_STAT & 0x20) )
			{
				*pREG_UART0_THR = cVal;
				nStatus = 1;
				break;
			}
		}while( !IsTimedout(nTimer) );
	}

	ClearTimeout(nTimer);

	return nStatus;
}


/*
 *   Function:    ClearReceiver
 *   Description: Clears the receive FIFO
 */
void ClearReceiver(void)
{
	int nStatus = 0;
	char temp;

	while( *pREG_UART0_STAT & 0x1 )
	{
		/* Anomaly 16000030 workaround */
		unsigned int uiTIMASK = cli();
		temp = *pREG_UART0_RBR;
		sti(uiTIMASK);
	}

	*pREG_UART0_STAT = 0xFFFFFFFF;

}


/*
 *   Function:    GetChar
 *   Description: Reads a character from the UART.
 */
int GetChar(char *const cVal)
{
	int nStatus = 0;
	unsigned int nTimer = SetTimeout(1000);
	if( ((unsigned int)-1) != nTimer )
	{
		do{
			if( *pREG_UART0_STAT & 0x1 )
			{
				/* Anomaly 16000030 workaround */
				unsigned int uiTIMASK = cli();
				*cVal = *pREG_UART0_RBR;
				sti(uiTIMASK);
				nStatus = 1;
				break;
			}
		}while( !IsTimedout(nTimer) );
	}

	ClearTimeout(nTimer);

	return nStatus;
}


#ifdef __DEBUG_UART__
/*
 *   Function:    UART_DEBUG_PRINT
 *   Description: Prints debug info over the UART using a predefined
*				 buffer.
 */
int UART_DEBUG_PRINT(void)
{
	unsigned int i = 0;		/* index */
	char temp;				/* temp char */


	/* loop through the debug buffer until the end, a NULL, or an error */
	for ( i = 0; i < UART_DEBUG_BUFFER_LINE_SIZE; i++)
	{
		temp = UART_DEBUG_BUFFER[i];

		/* if not NULL then print it */
		if (temp)
		{
			if( 0 == PutChar(temp) )
			{
				/* if error was detected then quit */
				return 0;
			}

			/* if it was a newline we need to add a carriage return */
			if ( 0x0a == temp )
			{
				if( 0 == PutChar(0x0d) )
				{
					/* if error was detected then quit */
					return 0;
				}
			}
		}
		else
		{
			/*
			 * clear our RX buffer in case any debug info was
			 * looped back
			 */
			ClearReceiver();

			/* else NULL was found */
			return 1;
		}
	}

	/*
	 * clear our RX buffer in case any debug info was
	 * looped back
	 */
	ClearReceiver();

	return 1;
}
#endif


#define UART1_TX_PORTG_FER  ((uint32_t) ((uint32_t) 1<<15))
#define UART1_RX_PORTG_FER  ((uint32_t) ((uint32_t) 1<<14))

#define UART1_TX_PORTG_MUX  ((uint32_t) ((uint32_t) 0<<30))
#define UART1_RX_PORTG_MUX  ((uint32_t) ((uint32_t) 0<<28))

void Init_UART1(void)
{
	static bool bInitialized = false;

	//*pREG_PORTG_FER_SET |= 0xC000;//bit 15,14
	*pREG_PORTG_FER |= UART1_TX_PORTG_FER | UART1_RX_PORTG_FER;
	ssync();

	/* configure UART1 RX and UART1 TX pins */

    *pREG_PORTG_MUX |= UART1_TX_PORTG_MUX | UART1_RX_PORTG_MUX;
	ssync();

	/*****************************************************************************
	 *
	 *  First of all, enable UART clock, put in UART mode, word length 8.
	 *
	 ****************************************************************************/
	*pREG_UART1_CTL = 0x301;//data:8bit, stop:1, parity:no

	/* Bit Rate = 		   SCLK
					-------------------				EDBO is a 1
					[16^(1-EDBO)] * Div

			   ##################################

					125 * 1000000
				  -------------------  = 9600.6
					 [16^0] * 13021
	*/

	*pREG_UART1_CLK = 0x800042aa;//SCLK,163.84m,9600



//	 /* Register the UART driver status interrupt handler with interrupt dispatcher */
//	    if(adi_int_InstallHandler(INTR_UART1_STAT,
//	                              UARTStatusHandler,
//	                              NULL,
//	                              true) != ADI_INT_SUCCESS)
//	    {
//	        /* Could not register the UART status interrupt handler, free up
//	         * UART device and return  */
//
//	         return ;
//	    }
	   /* rx interrupts  */
	  *pREG_UART1_IMSK_SET = 0x180;
}


/*
 *   Function:    PutChar
 *   Description: Writes a character to the UART.
 */
int PutChar1(const char cVal)
{
	int nStatus = 0;
	unsigned int nTimer = SetTimeout(1000);
	if( ((unsigned int)-1) != nTimer )
	{
		do{
			if( (*pREG_UART1_STAT & 0x20) )
			{
				*pREG_UART1_THR = cVal;
				nStatus = 1;
				break;
			}
		}while( !IsTimedout(nTimer) );
	}

	ClearTimeout(nTimer);

	return nStatus;
}


/*
 *   Function:    ClearReceiver
 *   Description: Clears the receive FIFO
 */
void ClearReceiver1(void)
{
	int nStatus = 0;
	char temp;

	while( *pREG_UART1_STAT & 0x1 )
	{
		/* Anomaly 16000030 workaround */
		unsigned int uiTIMASK = cli();
		temp = *pREG_UART1_RBR;
		sti(uiTIMASK);
	}

	*pREG_UART1_STAT = 0xFFFFFFFF;

}


/*
 *   Function:    GetChar
 *   Description: Reads a character from the UART.
 */
int GetChar1(char *const cVal)
{
	int nStatus = 0;
	unsigned int nTimer = SetTimeout(1000);
	if( ((unsigned int)-1) != nTimer )
	{
		do{
			if( *pREG_UART1_STAT & 0x1 )
			{
				/* Anomaly 16000030 workaround */
				unsigned int uiTIMASK = cli();
				*cVal = *pREG_UART1_RBR;
				sti(uiTIMASK);
				nStatus = 1;
				break;
			}
		}while( !IsTimedout(nTimer) );
	}

	ClearTimeout(nTimer);

	return nStatus;
}

#ifdef __DEBUG_UART__
/*
 *   Function:    UART_DEBUG_PRINT
 *   Description: Prints debug info over the UART using a predefined
*				 buffer.
 */
int UART_DEBUG_PRINT1(void)
{
	unsigned int i = 0;		/* index */
	char temp;				/* temp char */


	/* loop through the debug buffer until the end, a NULL, or an error */
	for ( i = 0; i < UART_DEBUG_BUFFER_LINE_SIZE; i++)
	{
		temp = UART_DEBUG_BUFFER[i];

		/* if not NULL then print it */
		if (temp)
		{
			if( 0 == PutChar1(temp) )
			{
				/* if error was detected then quit */
				return 0;
			}

			/* if it was a newline we need to add a carriage return */
			if ( 0x0a == temp )
			{
				if( 0 == PutChar1(0x0d) )
				{
					/* if error was detected then quit */
					return 0;
				}
			}
		}
		else
		{
			/*
			 * clear our RX buffer in case any debug info was
			 * looped back
			 */
			ClearReceiver1();

			/* else NULL was found */
			return 1;
		}
	}

	/*
	 * clear our RX buffer in case any debug info was
	 * looped back
	 */
	ClearReceiver1();

	return 1;
}
#endif

