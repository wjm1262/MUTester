/*
 * sys.c
 *
 *  Created on: 2015-2-9
 *      Author: WU jm
 */

#include "sys.h"

#include "IEC61850_9_2.h"


static void Initialize(void)
{
	/* Initialize Power service */
	ADI_PWR_RESULT pwrRes = Init_PowerService();
	if(pwrRes != ADI_PWR_SUCCESS)
	{
		printf("adi_pwr_Init Failed :%d\n", pwrRes);
		return;
	}

	Init_GPIO();

//	Init_PTPAuxin();//disable auxin interrupt

 //	Init_CoreTimer();

	Init_Timer_Interrupts();//TIMER1, for UART0 (timer)

#if defined(__DEBUG_UART__)

    Init_UART0();// UART0, Debug Output

#endif

//    Setup_SMC_Bank1();

	RegisterEthnetModual();

	RegisterSysTimeModual();
//
//	RegisterADModual();

//	RegisterExEthnetModual();

}


SYSTEM_STRUCT MuTesterSystem =
{
	Initialize,
};
