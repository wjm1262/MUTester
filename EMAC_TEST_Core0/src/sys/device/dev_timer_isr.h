/*
 * (C) Copyright 2012 - Analog Devices, Inc.  All rights reserved.
 *
 * FILE:     timer_isr.h ( )
 *
 * CHANGES:  1.00.0  - initial release
 */

#ifndef _TIMER_ISR_H_
#define _TIMER_ISR_H_

#include <sys\exception.h>
#include <cdefbf609.h>
#include <ccblkfn.h>

/*******************************************************************
*  global variables and defines
*******************************************************************/


/*******************************************************************
*  count down timer Service for time step = 1ms
*******************************************************************/

void Init_Timer_Interrupts(void);
void Delay(const unsigned long ulMs);
unsigned int SetTimeout(const unsigned long ulTicks);
unsigned long ClearTimeout(const unsigned int nIndex);
bool IsTimedout(const unsigned int nIndex);
bool EnableGPTimer(bool);
void Timer_ISR(void);

/*******************************************************************
*  Clock Tick Service for time step = 1ms
*******************************************************************/
/* Timer event handler */
typedef void (*Timer_Event_Handler)(void *pCBParam, uint32_t Event, void *pArg);


void Init_GP_Timer5(void);
bool EnableGPTimer5(bool bEnable);
unsigned int SetTimerEventHandler(Timer_Event_Handler handler);

#endif /* _TIMER_ISR_H_ */

