/*
 * dev_pwr.c
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */

#include "dev_pwr.h"

ADI_PWR_RESULT Init_PowerService(void)
{
	ADI_PWR_RESULT pwrRes;

	do{
		pwrRes = adi_pwr_Init(SYS_CLKIN,500000000,250000000,100000000);
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes =adi_pwr_SetClkControlRegister(ADI_PWR_CLK_CTL_DF,   0 );
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes =adi_pwr_SetClkControlRegister(ADI_PWR_CLK_CTL_MSEL,   MULTIPLIER_SEL );
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkDivideRegister( ADI_PWR_CLK_DIV_CSEL, CCLK_SEL) ;
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkDivideRegister( ADI_PWR_CLK_DIV_SYSSEL, SYSCLK_SEL) ;
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkDivideRegister( ADI_PWR_CLK_DIV_S0SEL, SCLK0_SEL) ;
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkDivideRegister( ADI_PWR_CLK_DIV_S1SEL, SCLK1_SEL) ;
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkDivideRegister( ADI_PWR_CLK_DIV_DSEL, DDRCLK_SEL) ;
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkDivideRegister( ADI_PWR_CLK_DIV_OSEL, OUTCLK_SEL) ;
		if (pwrRes != ADI_PWR_SUCCESS)
			break;
		pwrRes = adi_pwr_SetClkOutSelectRegister(ADI_PWR_CLK_SELECT_CLKOUT,CLKOUT_SEL);
		if (pwrRes != ADI_PWR_SUCCESS)
			break;

	}while(0);

	return pwrRes;
}

/*
 * FREQ is in MHz, CYCLE is in nanosecond
 */

int GetSCLK0(float *freq, float *cycle)
{
	int ErrorCode = 1;

	/* System Clock Frequencies*/
	uint32_t   fsysclk = 0u;   /* System Clock */
	uint32_t   fsclk0  = 0u;   /* System Clock0 */
	uint32_t   fsclk1  = 0u;   /* System Clock1 */
	ADI_PWR_RESULT tRes;

	if( (tRes = adi_pwr_GetSystemFreq(&fsysclk, &fsclk0, &fsclk1) )!= ADI_PWR_SUCCESS)
	{
		//DEBUG_PRINT("myGetSCLK0 call adi_pwr_GetSystemFreq: failed \n\n", fsysclk, fsclk0, fsclk1);
		return 0;
	}

	*freq = fsclk0/1000000;
	*cycle = 1000 / *freq;

	return ErrorCode;
}
