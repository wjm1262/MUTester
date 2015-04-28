/*
 * core_timer.h
 *
 *  Created on: 2014-10-28
 *      Author: Administrator
 */

#ifndef CORE_TIMER_H_
#define CORE_TIMER_H_

#include <ccblkfn.h>
#include <stdio.h>
#include <services/tmr/adi_ctmr.h>
#include <services/gpio/adi_gpio.h>



/* the handle of the core timer */
extern ADI_CTMR_HANDLE ghCoreTimer;

void Init_CoreTimer(void);

#endif /* CORE_TIMER_H_ */
