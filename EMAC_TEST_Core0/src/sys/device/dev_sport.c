/*
 * dev_sport.c
 *
 *  Created on: 2015-4-22
 *      Author: Administrator
 */

#include "dev_sport.h"
#include "dev_pwr.h"
#include "stdio.h"

#define SPORT1_BCLK_PORTE_MUX  ((uint16_t) ((uint16_t) 2<<8))
#define SPORT1_BFS_PORTE_MUX  ((uint16_t) ((uint16_t) 2<<6))
#define SPORT1_BD0_PORTE_MUX  ((uint16_t) ((uint16_t) 2<<2))
#define SPORT1_BD1_PORTE_MUX  ((uint16_t) ((uint16_t) 2<<0))

#define SPORT1_BD1_PORTE_FER  ((uint16_t) ((uint16_t) 1<<0))
#define SPORT1_BCLK_PORTE_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SPORT1_BFS_PORTE_FER  ((uint16_t) ((uint16_t) 1<<3))
#define SPORT1_BD0_PORTE_FER  ((uint16_t) ((uint16_t) 1<<1))


#define SPORT1B_ELEMMENT_NUM  8
#define SPORT1B_ELEMMENT_SIZE_BYTES  4
#define SPORT1B_RX_BUFFER_SIZE  (SPORT1B_ELEMMENT_NUM*SPORT1B_ELEMMENT_SIZE_BYTES)

int32_t SPORT1B_RxBuffer[SPORT1B_ELEMMENT_NUM] =  {0x00u};

/* Sport Devince number */
#define SPORT_DEVICE_NUM_CTL  1U


/* Macro which controls the mode of operation.
 *  Enabling "ENABLE_DMA_MODE" will enable the DMA mode of operation
 */
#define ENABLE_DMA_MODE

#ifdef ENABLE_DMA_MODE

	/* Memory required by driver for device operation in DMA mode. */
	#define SPORT_MEM_SIZE (ADI_SPORT_DMA_MEMORY_SIZE)
#else

	/* Memory required by the driver for device operation in interrupt mode. */
	#define SPORT_MEM_SIZE (ADI_SPORT_INT_MEMORY_SIZE)
#endif

/* Handle for Rx channel */
#pragma align(4)
ADI_SPORT_HANDLE          g_hSportRx;

/* Memory required by the device for RX operation */
#pragma align(4)
unsigned char g_SportRxMem[SPORT_MEM_SIZE];


/*********************************************************************

    Function:       configureRx

    Description:    Function for initializing the RX  channel.

*********************************************************************/

static ADI_SPORT_RESULT  configureRx(void)
{
	ADI_SPORT_RESULT eResult;
	/* configure the sport for data length, MSB first etc */
	if( (eResult = adi_sport_ConfigData(g_hSportRx,ADI_SPORT_DTYPE_ZERO_FILL,
										17U,   //word length,For Stereo and Multichannel mode the range is between 4 and 31.
										false, //Enable the LSM or MSB
										false,  //Enable pack or not
										false)) != ADI_SPORT_SUCCESS)//Enable/disable the Right Justified mode. Valid only in I2S mode and ignored in other modes.
	{
	  return(eResult);
	}

	/* configure the clock for the SPORT. This API set the whether use the internal clock, SPORT clock etc.
	 * Since this example, RX  gets the clock and Frame sync from TX. */
	uint32_t DivSCLK = 0;
	float sclk = 0, cycle = 0;
	GetSCLK0(&sclk, &cycle);
	DivSCLK = sclk / 16 - 1;//real fre >= 16MHz

	if( (eResult = adi_sport_ConfigClock(g_hSportRx,
										 DivSCLK,    //divider of sclk, sport_clk = 17Mhz
										 true, //enable internal clock or external clock
										 true, //enable the rising edge  or falling edge,control the incoming data,outcome is opposed. transmit and receive functions of any two serial ports connectedtogether shouldalways select  the same value
										 true)) != ADI_SPORT_SUCCESS) //enable the gate clock mode, clock valid only in data transmit or receive
	{
	  return(eResult);
	}
	/* Configure the frame sync. This API configure the SPORT whether to use frame sync or not , external or internal frame-sync etc */
	if( (eResult = adi_sport_ConfigFrameSync(g_hSportRx,
											 18,    //divider of sport_clk,f-frameSync = 18 * sport_clk
											 true,   //Enable, if need a Frame sync
											 true,   //Enable internal FS or external FS signal
											 true,   //Enable Use data-independent frame sync or not,Valid only if the specified device is in "transmit"(TX)mode
											 true,  //Enable active FS signal  low or High
											 true,  //Enable  late FS or early  FS.
											 true)) != ADI_SPORT_SUCCESS) //Boolean flag to indicate whether frame sync is edge sensitive OR level sensitive.
	{
	  return(eResult);
	}

	return(eResult);
}


/*
 * SPORT2-B init
 */

ADI_SPORT_RESULT Init_SPORT1B(void)
{
	/* Variable for storing the return code from UART device */
	ADI_SPORT_RESULT  eSportResult;

	/* PORTx_MUX registers */
	*pREG_PORTE_MUX |= SPORT1_BCLK_PORTE_MUX | SPORT1_BFS_PORTE_MUX
	| SPORT1_BD0_PORTE_MUX | SPORT1_BD1_PORTE_MUX;

	/* PORTx_FER registers */
	*pREG_PORTE_FER |= SPORT1_BCLK_PORTE_FER | SPORT1_BFS_PORTE_FER
	| SPORT1_BD0_PORTE_FER | SPORT1_BD1_PORTE_FER;

	do
	{
		if( (eSportResult = adi_sport_Open(SPORT_DEVICE_NUM_CTL,
				ADI_HALF_SPORT_B,
				ADI_SPORT_DIR_RX,
				ADI_SPORT_SERIAL_MODE,
				g_SportRxMem,
				(uint32_t)(SPORT_MEM_SIZE),
				&g_hSportRx) ) != ADI_SPORT_SUCCESS)
		{
			DEBUG_PRINT("\n\tFailed to open the device in Rx mode :  %d",eSportResult);
			break;
		}

	#ifdef ENABLE_DMA_MODE
		//enable DMA mode
		if((eSportResult = adi_sport_EnableDMAMode(g_hSportRx, true))!= ADI_SPORT_SUCCESS)
		{
			DEBUG_PRINT("\n\t Failed to enable the dma mode :  %d",eSportResult);
			break;
		}

		if((eSportResult = adi_sport_StreamingEnable(g_hSportRx, true))!= ADI_SPORT_SUCCESS)
		{
			DEBUG_PRINT("\n\t Failed to enable the dma streaming mode :  %d",eSportResult);
			break;
		}

	#endif
		/*
		* Setup the CLK and FS parameter.
		*/
		if((eSportResult = configureRx())!= ADI_SPORT_SUCCESS)
		{
			DEBUG_PRINT("\n\t Failed to configure RX:   %d",eSportResult);
			break;
		}



	}while(0);

	return eSportResult;
}

void Register_SPORT1B_Callback(SPORT_CallbackFn pfCallback)
{
	ADI_SPORT_RESULT  eSportResult;

	/* Register a call back function for RX channel. Registered callback function will be called as and when
		* an filled buffer is avilable.
		*/
	if( (eSportResult = adi_sport_RegisterCallback(g_hSportRx, pfCallback, NULL)) != ADI_SPORT_SUCCESS)
	{
		DEBUG_PRINT("\n\t Failed to register the call back for Rx :  %d",eSportResult);
	}

}

void Enable_SPORT1B(void)
{
	ADI_SPORT_RESULT  eSportResult;
	if((eSportResult = adi_sport_Enable(g_hSportRx, true))!= ADI_SPORT_SUCCESS)
	{
		DEBUG_PRINT("\n\t Failed to Enable the Rx:  %d",eSportResult);
	}

#if SPORT_DUAL_DATA_LINE    /* 双数据线读取数据 */
	/* Enable the SPORT2-B Rx  D1 */
	*pREG_SPORT1_CTL_B   |= (uint32_t)( (ENUM_SPORT_CTL_SECONDARY_EN));
#endif
}

void Disable_SPORT1B(void)
{
	ADI_SPORT_RESULT  eSportResult;
	if((eSportResult = adi_sport_Enable(g_hSportRx, false))!= ADI_SPORT_SUCCESS)
	{
		DEBUG_PRINT("\n\t Failed to disable the Rx:  %d",eSportResult);
	}

#if SPORT_DUAL_DATA_LINE    /* 双数据线读取数据 */
	*pREG_SPORT1_CTL_B   &= (uint32_t)(~(ENUM_SPORT_CTL_SECONDARY_EN));
#endif
}

void SubmitBuffer_SPORT1B(void)
{
	ADI_SPORT_RESULT  eSportResult;

	if((eSportResult = adi_sport_SubmitBuffer(g_hSportRx, SPORT1B_RxBuffer, SPORT1B_RX_BUFFER_SIZE))!= ADI_SPORT_SUCCESS)
	{
		DEBUG_PRINT("\n\t Failed to Submit Buffer:  %d",eSportResult);
	}
}

void CancelClk_SPORT1B(void)
{
	/*
	 * Avoiding data offset, we should cancel the SPORT1-B-CLK, set clk in high default.
	 */
	*pREG_PORTE_MUX     &= ~SPORT1_BCLK_PORTE_MUX;
	*pREG_PORTE_FER     &= ~SPORT1_BCLK_PORTE_FER;
	*pREG_PORTE_DATA_SET = ADI_GPIO_PIN_4;
	*pREG_PORTE_MUX     |= SPORT1_BCLK_PORTE_MUX;
	*pREG_PORTE_FER     |= SPORT1_BCLK_PORTE_FER;
}
