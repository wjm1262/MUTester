/*
 * dev_spi.c
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#include "dev_spi.h"

#include "dev_pwr.h"
#include "stdio.h"
/*
 * SPI pin MUX
 */
#define SPI0_CLK_PORTD_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define SPI0_MISO_PORTD_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define SPI0_SEL2_PORTD_MUX  ((uint16_t) ((uint16_t) 2<<2))

#define SPI0_CLK_PORTD_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_MISO_PORTD_FER  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_SEL2_PORTD_FER  ((uint16_t) ((uint16_t) 1<<1))

/*
 * SPI initialization
 * 1, #include <drivers/spi/adi_spi.h>
 * 2, Define the ADI_SPI_HANDLE and memory used by the SPI.
 * 3, Setting the pin multiplex in system.svc
 */

ADI_SPI_HANDLE g_hSPI0;

uint8_t SpiMemory[ADI_SPI_DMA_MEMORY_SIZE];

#define DeviceNum 0


/*
 * enable spi
 */

void EnableSPI0(void)
{
	*pREG_SPI0_RXCTL |= (3u << 4);// Rx
	*pREG_SPI0_CTL   |= 1;      // spi
}
/*
 * Stop SPI
 */

void DisableSPI0(void)
{
	*pREG_SPI0_RXCTL &= ~(3u << 4);//disable Rx
	*pREG_SPI0_CTL   &= ~(1);      //disable spi
}

uint32_t Init_SPI0(void)
{
	uint32_t Result = 0u;

	/*
	 * SPI0 PORTx_MUX registers
	 * CLK, MISO, SEL2.
	 */
	*pREG_PORTD_MUX |= SPI0_CLK_PORTD_MUX | SPI0_MISO_PORTD_MUX
	 | SPI0_SEL2_PORTD_MUX;

	*pREG_PORTD_FER |= SPI0_CLK_PORTD_FER | SPI0_MISO_PORTD_FER
	 | SPI0_SEL2_PORTD_FER;

	Result = adi_spi_Open(DeviceNum, &SpiMemory, (uint32_t)ADI_SPI_DMA_MEMORY_SIZE, &g_hSPI0);
	if (Result == 0u)
	{
	   /* device in master of the SPI interface */
	   Result = (uint32_t)adi_spi_SetMaster(g_hSPI0, true);
	}
	if (Result == 0u)
	{
	   /* SPI slave select in controlled by software not hardware */
	   Result = (uint32_t)adi_spi_SetHwSlaveSelect(g_hSPI0, false);
	}
	if (Result == 0u)
	{
		/* send zeros if tx SPI underflows*/
		Result = (uint32_t)adi_spi_SetTransmitUnderflow(g_hSPI0, true);
	}
	if (Result == 0u)
	{
		/* polar of clock */
		Result = (uint32_t)adi_spi_SetClockPolarity(g_hSPI0, false);
	}
	if (Result == 0u)
	{
		/* data transitions on falling edge of clock */
		Result = (uint32_t)adi_spi_SetClockPhase(g_hSPI0, false);
	}
	if (Result == 0u)
	{
		/* SPI clock is SCLK divided by */
		float sclk, cycle;

		GetSCLK0(&sclk, &cycle);

		//Driver above 3.3v, data clock 17Mhz
		uint32_t DivSCLK  = sclk / 17;
		Result = (uint32_t)adi_spi_SetClock(g_hSPI0, DivSCLK);//real fre <= 17Mhz
	}
	if (Result == 0u)
	{
		/* SPI slave select is on SPI slave select 2 pin */
		Result = (uint32_t)adi_spi_SetSlaveSelect(g_hSPI0, ADI_SPI_SSEL_ENABLE2);
	}
	if (Result == 0u)
	{
		/* interrupt mode, i.e no dma */
		Result = (uint32_t)adi_spi_EnableDmaMode(g_hSPI0, false);
	}
	if (Result == 0u)
	{
		/* SPI data transfers are 16 bit */
		Result = (uint32_t)adi_spi_SetWordSize(g_hSPI0, ADI_SPI_TRANSFER_8BIT);
		//Result = adi_spi_SetDmaTransferSize(phSPI, ADI_SPI_DMA_TRANSFER_32BIT);

	}

//	if (Result == 0u)
//	{
//		/* no callbacks */
//		Result = (uint32_t)adi_spi_RegisterCallback(g_hSPI0, pfCallback, NULL);
//	}
//
//	//the watermark is very important, if the following function have been annotated,the SPI does not work.
//	if (Result == 0u)
//	{
//		/* generate tx data interrupt when watermark level breaches 50% level */
//		/* DMA watermark levels are disabled because SPI is in interrupt mode */
//		Result = (uint32_t)adi_spi_SetTxWatermark(g_hSPI0,
//												  ADI_SPI_WATERMARK_50,
//												  ADI_SPI_WATERMARK_DISABLE,
//												  ADI_SPI_WATERMARK_DISABLE);
//	}
//	if (Result == 0u)
//	{
//		/* generate rx data interrupt when watermark level breaches 50% level */
//		/* DMA watermark levels are disabled because SPI is in interrupt mode */
//		Result = (uint32_t)adi_spi_SetRxWatermark(g_hSPI0,
//												  ADI_SPI_WATERMARK_50,
//												  ADI_SPI_WATERMARK_DISABLE,
//												  ADI_SPI_WATERMARK_DISABLE);
//	}
//
//	DisableSPI0();

	return Result;
}

void Register_SPI0_Callback(SPI_CallbackFn pfCallback)
{
	uint32_t Result = 0u;

	/* no callbacks */
	Result = (uint32_t)adi_spi_RegisterCallback(g_hSPI0, pfCallback, NULL);

	//the watermark is very important, if the following function have been annotated,the SPI does not work.
	if (Result == 0u)
	{
		/* generate tx data interrupt when watermark level breaches 50% level */
		/* DMA watermark levels are disabled because SPI is in interrupt mode */
		Result = (uint32_t)adi_spi_SetTxWatermark(g_hSPI0,
												  ADI_SPI_WATERMARK_50,
												  ADI_SPI_WATERMARK_DISABLE,
												  ADI_SPI_WATERMARK_DISABLE);
	}

	if (Result == 0u)
	{
		/* generate rx data interrupt when watermark level breaches 50% level */
		/* DMA watermark levels are disabled because SPI is in interrupt mode */
		Result = (uint32_t)adi_spi_SetRxWatermark(g_hSPI0,
												  ADI_SPI_WATERMARK_50,
												  ADI_SPI_WATERMARK_DISABLE,
												  ADI_SPI_WATERMARK_DISABLE);
	}

	DisableSPI0();
}
