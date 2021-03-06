/*
 * dev_ad.c
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#include "sys.h"
#include "dev_ad.h"
#include "dev_spi.h"

#include "dev_sport.h"

#include "dev_smc.h"

#include "post_debug.h"


/*
 * ad7608 init
 */
//static uint32_t Init_AD7608(void)
//{
//	uint32_t result = (uint32_t)-1;
//	do
//	{
//		/* setup Pin for control the ad7608*/
//		if((result = Init_AD7608_IO()) != 0)
//		{
//			DEBUG_PRINT("Failed in function Pin_Init : %d \n\n",result);
//			break;
//		}
//
//		/* setup SPI for data receive*/
//		if((result = Init_SPI0()) != 0)
//		{
//			DEBUG_PRINT("Failed in function SPI_Init : %d \n\n",result);
//			break;
//		}
//
//		/* init the AD7608 BUSY pin IRQ*/
//		if((result = Init_Busy_IO_IRQ() ) != 0)
//		{
//			DEBUG_PRINT("Failed in function BusyIRQ_Init : %d \n\n",result);
//			break;
//		}
//
//		RESET_HIGH(); 	//Rising edge of RESET reset the ad7608,
//		RESET_LOW();	//restore the RESET pin
//
//	}while(0);
//
////	GP_Timer_Init();
//
//	return result;
//}

static uint32_t Init_AD7608(void)
{
	uint32_t result = (uint32_t)-1;
	do
	{
		/* setup Pin for control the ad7606*/
		if((result = Init_AD7608_IO()) != 0)
		{
			printf("\n\t Failed in function Pin_Init : %d",result);
			break;
		}

		/* setup SPORT for data receive*/
		if((result = Init_SPORT1B()) != 0)
		{
			printf("\n\t Failed in function SPORT2B_Init : %d",result);
			break;
		}

		/* init the AD7608 BUSY pin IRQ*/
		if((result = Init_Busy_IO_IRQ()) != 0)
		{
			printf("\n\t Failed in function BusyIRQ_Init : %d",result);
			break;
		}

		/* setup AD7608 */
		Setup_AD7608_SMC();


	}while(0);

	return result;
}

static void RegisterBuzyIOCallback( AD7608_Busy_ISR_Handler handler )
{
	Register_Buzy_IO_Callback( handler );
}

static void EnableBuzyIOInterrupt(bool enable)
{
	Enable_Buzy_IO_Interrupt( enable);
}

static void RegisterSPI0Callback( SPI_CallbackFn pfCallback)
{
	Register_SPI0_Callback( pfCallback );
}

static void RegisterSportCallback( SPORT_CallbackFn pfCallback)
{
	Register_SPORT1B_Callback( pfCallback);
}

void RegisterADModual(void)
{
	MuTesterSystem.Device.AD7608.InitADDevice = Init_AD7608;
	MuTesterSystem.Device.AD7608.RegisterBuzyIOCallback = RegisterBuzyIOCallback;
	MuTesterSystem.Device.AD7608.EnableBuzyIOInterrupt  = EnableBuzyIOInterrupt;

	MuTesterSystem.Device.AD7608.RegisterSPI0Callback   = RegisterSPI0Callback;

	MuTesterSystem.Device.AD7608.RegisterSportCallback  = RegisterSportCallback;
}
