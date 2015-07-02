/*
 * dev_core_timer.c
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */


#include "dev_core_timer.h"

/* the handle of the core timer */
ADI_CTMR_HANDLE ghCoreTimer;
/*
 * Core timer initialization
 * 1, #include <services/tmr/adi_ctmr.h>
 * 2, Define the ADI_CTMR_HANDLE of the core timer.
 * 3, Define the period and the prescale to set the timer cycle,the default core clock is 500M.
 * 4, Define the interrupt callback function if used.
 */
/* Period and prescale of the core timer */

#define MEGA (1000000u)
//#define PERIOD (400 000 000)
#define PERIOD (30000)
#define PRESCALE (0u)
/*
 * the handler of the core timer.
 */
static void CoreTimerHandler(void *pCBParam, uint32_t Event, void *pArg)
{
//	/* set GPIO output WDI */
//	adi_gpio_Toggle(ADI_GPIO_PORT_C, ADI_GPIO_PIN_15);
//
//	/* LED1 */
//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

//	Send_9_2Frame();

}

void Init_CoreTimer(void)
{
	ADI_CTMR_RESULT result;

	do{

		result = adi_ctmr_Open(ADI_CTMR_DEV0, CoreTimerHandler, NULL, &ghCoreTimer);
		if (result != 0)
			break;
		result = adi_ctmr_EnableAutoReload(ghCoreTimer, true);
		if (result != 0)
			break;
		result = adi_ctmr_SetPeriod(ghCoreTimer, PERIOD);
		if (result != 0)
			break;
		result = adi_ctmr_SetScale(ghCoreTimer, PRESCALE);
		if (result != 0)
			break;
		result = adi_ctmr_Enable(ghCoreTimer, true);

	}while(0);

	if (result != 0)
		printf("initialization failed! error code is %d\n", result);

}
