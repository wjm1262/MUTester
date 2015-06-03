
#include <stdio.h>
#include "BF609_FPGA_Comm.h"
#include <ccblkfn.h>
#include <services/gpio/adi_gpio.h>
#include "BF609_FPGA_Comm_Protocol.h"
int32_t Recv_FPGA_Info(void *pFPGA_BF609_data);
int32_t Recv_FT3_Data(void *pFPGA_BF609_data);

/***********************************************************************************************
 * SPI peripheral
 *
 *  Created on: 2015-4-23
 *      Author: ChiliWang
 *************************************************************************************************/
/*
 * SPI pin MUX
 */
#include <drivers/spi/adi_spi.h>
#define SPI0_CLK_PORTD_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define SPI0_MOSI_PORTD_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define SPI0_CLK_PORTD_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_MOSI_PORTD_FER  ((uint16_t) ((uint16_t) 1<<3))

#define SPI1_CLK_PORTD_FER  ((uint16_t) ((uint16_t) 1<<5))
#define SPI1_MISO_PORTD_FER  ((uint32_t) ((uint32_t) 1<<14))
#define SPI1_CLK_PORTD_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define SPI1_MISO_PORTD_MUX  ((uint32_t) ((uint32_t) 0<<28))

/*
 * SPI0 Global data definition.
 */
ADI_SPI_HANDLE hSPI0;
uint8_t Spi0Memory[ADI_SPI_DMA_MEMORY_SIZE];
/* transceiver configurations */
ADI_SPI_TRANSCEIVER spi0_Tx_Rx_buffer  = {NULL, 0u, NULL, 0u, NULL, 0u};
#define DeviceNum0  0
void SPI0_Callback(void *pCBParam, uint32_t nEvent, void *pArg);


/*
 * SPI1 Global data definition.
 */
ADI_SPI_HANDLE hSPI1;
uint8_t Spi1Memory[ADI_SPI_DMA_MEMORY_SIZE];
/* transceiver configurations */
ADI_SPI_TRANSCEIVER spi1_Tx_Rx_buffer  = {NULL, 0u, NULL, 0u, NULL, 0u};
#define DeviceNum1  1
void SPI1_Callback(void *pCBParam, uint32_t nEvent, void *pArg);

int32_t EnableSPI(uint8_t ucSPI_ID, bool bEnable);
void *pFPGA_BF609_data;
/*
 * 被检PPS 引脚中断处理函数，进入此函数表明 FPGA有被检PPS数据需要发送给BF609
 * 用户需要在此启动FPGA数据读取工作
 */
void MeterPPS_DataIRQ(ADI_GPIO_PIN_INTERRUPT const ePinInt,const uint32_t event, void *pArg)
{
	Recv_FPGA_Info(pFPGA_BF609_data);
}
/*
 * 参考输入的B码解码数据  引脚中断处理函数，进入此函数表明 FPGA有参考输入的B码解码数据需要发送给BF609
 * 用户需要在此启动FPGA数据读取工作
 */
void SyncBcodeDataIRQ(ADI_GPIO_PIN_INTERRUPT const ePinInt,const uint32_t event, void *pArg)
{
	Recv_FPGA_Info(pFPGA_BF609_data);
}
/*
 * SOE遥信捕获  引脚中断处理函数，进入此函数表明 8个遥信输入有跳变数据需要发送给BF609
 * 用户需要在此启动FPGA数据读取工作
 */
void SOE_CapDataIRQ(ADI_GPIO_PIN_INTERRUPT const ePinInt,const uint32_t event, void *pArg)
{
	Recv_FPGA_Info(pFPGA_BF609_data);
}
/*
 * FT3接收时FPGA给DSP的触发信号 引脚中断处理函数，进入此函数表明有FT3数据需要发送给BF609
 * 用户需要在此启动FPGA数据读取工作
 */
void FT3_RecvDataIRQ(ADI_GPIO_PIN_INTERRUPT const ePinInt,const uint32_t event, void *pArg)
{
	Recv_FT3_Data(pFPGA_BF609_data);
}
/*
 * Pin interrupt used for SPI1 Rx and link-port Tx & Rx.
 *  Following pin IRQ used for SPI1 Rx to distinguish which data is .
 * 			PD10				：被检PPS
 * 			PD11				：参考输入的B码解码数据
 * 			PD12/PII0_D20		：SOE遥信捕获，既8个遥信输入有跳变
 * 	Following pin IRQ used for link-port to start Tx or Rx data.
 * 			PD15/PII0_D21		：FT3接收时FPGA给DSP的触发信号，告诉DSP有FT3数据了；
 */
static ADI_GPIO_RESULT Init_FPGA_IRQ_Pin(void)
{
	ADI_GPIO_RESULT result;
	do
	{
		/********************************************************************************
		 * PD10				：被检PPS中断登记
		 *
		 *****************************************************************************/
		/* set PD10 in GPIO input mode */
	    result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_D,
			ADI_GPIO_PIN_10,
			ADI_GPIO_DIRECTION_INPUT);

	    //分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_ASSIGN_BYTE_1, ADI_GPIO_PIN_ASSIGN_PDH_PINT3);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_PinInterruptAssignment : %d",result);
			  break;
		}

		//分配具体引脚和中断方式
		result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_10, ADI_GPIO_SENSE_RISING_EDGE);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_SetPinIntEdgeSense : %d",result);
			  break;
		}

		//登记IRQ回调函数
		result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_10, MeterPPS_DataIRQ, NULL);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}

		//使能中断
		result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_10, true);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}

		/********************************************************************************
		 * PD11				：参考输入的B码解码数据
		 *
		 *****************************************************************************/
		/* set PD11 in GPIO input mode */
	    if(result != ADI_GPIO_SUCCESS)
			result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_D,
				ADI_GPIO_PIN_11,
				ADI_GPIO_DIRECTION_INPUT);
		//分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_ASSIGN_BYTE_1, ADI_GPIO_PIN_ASSIGN_PDH_PINT3);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_PinInterruptAssignment : %d",result);
			  break;
		}

		//分配具体引脚和中断方式
		result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_11, ADI_GPIO_SENSE_RISING_EDGE);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_SetPinIntEdgeSense : %d",result);
			  break;
		}

		//登记IRQ回调函数
		result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_11, SyncBcodeDataIRQ, NULL);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}

		//使能中断
		result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_11, true);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}

		/********************************************************************************
		 * PD12/PII0_D20		：SOE遥信捕获，既8个遥信输入有跳变
		 *
		 *****************************************************************************/
		/* set PD12 in GPIO input mode */
	    if(result != ADI_GPIO_SUCCESS)
			result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_D,
				ADI_GPIO_PIN_12,
				ADI_GPIO_DIRECTION_INPUT);
		//分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_ASSIGN_BYTE_1, ADI_GPIO_PIN_ASSIGN_PDH_PINT3);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_PinInterruptAssignment : %d",result);
			  break;
		}

		//分配具体引脚和中断方式
		result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_12, ADI_GPIO_SENSE_RISING_EDGE);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_SetPinIntEdgeSense : %d",result);
			  break;
		}

		//登记IRQ回调函数
		result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_12, SOE_CapDataIRQ, NULL);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}

		//使能中断
		result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_12, true);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}
		/********************************************************************************
		 * PD15/PII0_D21		：FT3接收时FPGA给DSP的触发信号，告诉DSP有FT3数据了；
		 *
		 *****************************************************************************/
		/* set PD15 in GPIO input mode */
		if(result != ADI_GPIO_SUCCESS)
			result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_D,
				ADI_GPIO_PIN_15,
				ADI_GPIO_DIRECTION_INPUT);
		//分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_ASSIGN_BYTE_1, ADI_GPIO_PIN_ASSIGN_PDH_PINT3);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_PinInterruptAssignment : %d",result);
			  break;
		}

		//分配具体引脚和中断方式
		result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_15, ADI_GPIO_SENSE_RISING_EDGE);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_SetPinIntEdgeSense : %d",result);
			  break;
		}

		//登记IRQ回调函数
		result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_15, FT3_RecvDataIRQ, NULL);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}

		//使能中断
		result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_15, true);
		if(result != ADI_GPIO_SUCCESS)
		{
			  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
			  break;
		}
	}while(0);

	return result;
}
/*
 * SPI0 callback function
 */
void SPI0_Callback(void *pCBParam, uint32_t nEvent, void *pArg)
{
	adi_gpio_Set(ADI_GPIO_PORT_D,ADI_GPIO_PIN_2);  /* CS High */
    switch(nEvent)
    {
        case (uint32_t)ADI_SPI_TRANSCEIVER_PROCESSED:
        	{
        	/* stop the SPI0 process */
        	EnableSPI(0, false);

        	}
            break;
        default:
            break;
    }
}
/*
 * SPI1 callback function
 */
void SPI1_Callback(void *pCBParam, uint32_t nEvent, void *pArg)
{
	adi_gpio_Set(ADI_GPIO_PORT_D,ADI_GPIO_PIN_13);  /* CS High */
    switch(nEvent)
    {
        case (uint32_t)ADI_SPI_TRANSCEIVER_PROCESSED:
        	{
        	/* stop the SPI1 process */
        	EnableSPI(1, false);

        	}
            break;
        default:
            break;
    }
}
/*
 * FREQ is in MHz, CYCLE is in nanosecond
 */
static bool myGetSCLK0(float *freq, float *cycle)
{

	int msel, csel, syssel, s0sel, df;
	uint32_t cgu_ctl, cgu_div;

	cgu_ctl = *pREG_CGU0_CTL;
	cgu_div = *pREG_CGU0_DIV;

	msel = (cgu_ctl & BITM_CGU_CTL_MSEL) >> BITP_CGU_CTL_MSEL;
	df = (cgu_ctl & BITM_CGU_CTL_DF) >> BITP_CGU_CTL_DF;
	csel = (cgu_div & BITM_CGU_DIV_CSEL) >> BITP_CGU_DIV_CSEL;
	syssel = (cgu_div & BITM_CGU_DIV_SYSSEL) >> BITP_CGU_DIV_SYSSEL;
	s0sel = (cgu_div & BITM_CGU_DIV_S0SEL) >> BITP_CGU_DIV_S0SEL;

	*freq = CLKIN_For_FT3 * msel / (1 + df) / syssel / s0sel;
	*cycle = 1000 / *freq;

	return true;
}
/*
 * SPI0 is used for send FPGA CFG information.
 */
static int Init_SPI0_FPGA(void)
{
		int32_t result = 0u;

	    /*
	     * SPI0 PORTx_MUX registers
	     * CLK, MISO
	     */
	    *pREG_PORTD_MUX |= SPI0_CLK_PORTD_MUX | SPI0_MOSI_PORTD_MUX;

	    *pREG_PORTD_FER |= SPI0_CLK_PORTD_FER | SPI0_MOSI_PORTD_FER;

		result = adi_spi_Open(DeviceNum0, &Spi0Memory, (uint32_t)ADI_SPI_DMA_MEMORY_SIZE, &hSPI0);
	    if (result == 0u)
		{
	       /* device in master of the SPI interface */
		   result = (uint32_t)adi_spi_SetMaster(hSPI0, true);
		}
	    if (result == 0u)
		{
	       /* SPI slave select in controlled by software not hardware */
		   result = (uint32_t)adi_spi_SetHwSlaveSelect(hSPI0, false);
		}
	    if (result == 0u)
		{
	    	/* send zeros if tx SPI underflows*/
			result = (uint32_t)adi_spi_SetTransmitUnderflow(hSPI0, true);
		}
	    if (result == 0u)
		{
	    	/* polar of clock */
	    	result = (uint32_t)adi_spi_SetClockPolarity(hSPI0, false);
		}
	    if (result == 0u)
		{
	    	/* data transitions on falling edge of clock */
	    	result = (uint32_t)adi_spi_SetClockPhase(hSPI0, false);
		}
	    if (result == 0u)
		{
	    	/* SPI clock is SCLK divided by */
	    	float sclk, cycle;
	    	myGetSCLK0(&sclk, &cycle);
	    	/* data clock SPI1_CLK Mhz */
	    	uint32_t DivSCLK  = sclk / SPI0_CLK;
	    	result = (uint32_t)adi_spi_SetClock(hSPI1, DivSCLK);//real fre <= SPI0_CLK Mhz
		}
	    if (result == 0u)
		{
	    	/* SPI slave select is on SPI slave select 2 pin */
	    	result = (uint32_t)adi_spi_SetSlaveSelect(hSPI0, ADI_SPI_SSEL_ENABLE2);
		}
	    if (result == 0u)
		{
	    	/* SPI data transfers are 16 bit */
	    	result = (uint32_t)adi_spi_SetWordSize(hSPI0, ADI_SPI_TRANSFER_8BIT);
		}

	    if (result == 0u)
		{
	    	/* no callbacks */
		    result = (uint32_t)adi_spi_RegisterCallback(hSPI0, SPI0_Callback, NULL);
		}
	    if (result == 0u)
		{
	    	/* interrupt mode, i.e no dma */
		    result = (uint32_t)adi_spi_EnableDmaMode(hSPI0, false);
		}
		/*
		 * the watermark is very important, if the following function have been annotated,the SPI does not work.
		 * beacuse following API enable the SPI
		 */
		if (result == 0u)
		{
			/* generate tx data interrupt when watermark level breaches 50% level */
			/* DMA watermark levels are disabled because SPI is in interrupt mode */
			result = (uint32_t)adi_spi_SetTxWatermark(hSPI0,
													  ADI_SPI_WATERMARK_50,
													  ADI_SPI_WATERMARK_DISABLE,
													  ADI_SPI_WATERMARK_DISABLE);
		}
		/* stop SPI0 firstly, Enable when user need data Tx & Rx */
		EnableSPI(0, false);
	    return result;
}

/*
 * SPI1 is used for receive FPGA information.
 */
static int Init_SPI1_FPGA(void)
{
		int32_t result = 0u;

	    /*
	     * SPI0 PORTx_MUX registers
	     * CLK, MISO
	     */
	    *pREG_PORTD_MUX |= SPI1_CLK_PORTD_MUX | SPI1_MISO_PORTD_MUX;

	    *pREG_PORTD_FER |= SPI1_CLK_PORTD_FER | SPI1_MISO_PORTD_FER;

		result = adi_spi_Open(DeviceNum1, &Spi1Memory, (uint32_t)ADI_SPI_DMA_MEMORY_SIZE, &hSPI1);
	    if (result == 0u)
		{
	       /* device in master of the SPI interface */
		   result = (uint32_t)adi_spi_SetMaster(hSPI1, true);
		}
	    if (result == 0u)
		{
	       /* SPI slave select in controlled by software not hardware */
		   result = (uint32_t)adi_spi_SetHwSlaveSelect(hSPI1, false);
		}
	    if (result == 0u)
		{
	    	/* send zeros if tx SPI underflows*/
			result = (uint32_t)adi_spi_SetTransmitUnderflow(hSPI1, true);
		}
	    if (result == 0u)
		{
	    	/* polar of clock */
	    	result = (uint32_t)adi_spi_SetClockPolarity(hSPI1, false);
		}
	    if (result == 0u)
		{
	    	/* data transitions on falling edge of clock */
	    	result = (uint32_t)adi_spi_SetClockPhase(hSPI1, false);
		}
	    if (result == 0u)
		{
	    	/* SPI clock is SCLK divided by */
	    	float sclk, cycle;
	    	myGetSCLK0(&sclk, &cycle);
	    	/* data clock SPI1_CLK Mhz */
	    	uint32_t DivSCLK  = sclk / SPI1_CLK;
	    	result = (uint32_t)adi_spi_SetClock(hSPI1, DivSCLK);//real fre <= SPI1_CLK Mhz
		}
	    if (result == 0u)
		{
	    	/* SPI slave select is on SPI slave select 2 pin */
	    	result = (uint32_t)adi_spi_SetSlaveSelect(hSPI1, ADI_SPI_SSEL_ENABLE2);
		}
	    if (result == 0u)
		{
	    	/* SPI data transfers are 16 bit */
	    	result = (uint32_t)adi_spi_SetWordSize(hSPI1, ADI_SPI_TRANSFER_8BIT);
		}

	    if (result == 0u)
		{
	    	/* no callbacks */
		    result = (uint32_t)adi_spi_RegisterCallback(hSPI1, SPI1_Callback, NULL);
		}
	    if (result == 0u)
		{
	    	/* interrupt mode, i.e no dma */
		    result = (uint32_t)adi_spi_EnableDmaMode(hSPI1, false);
		}
		/*
		 * the watermark is very important, if the following function have been annotated,the SPI does not work.
		 * beacuse following API enable the SPI
		 */
		if (result == 0u)
		{
			/* generate rx data interrupt when watermark level breaches 50% level */
			/* DMA watermark levels are disabled because SPI is in interrupt mode */
			result = (uint32_t)adi_spi_SetRxWatermark(hSPI1,
													  ADI_SPI_WATERMARK_50,
													  ADI_SPI_WATERMARK_DISABLE,
													  ADI_SPI_WATERMARK_DISABLE);
		}

		/* stop SPI firstly, Enable when user need data Tx & Rx */
		EnableSPI(1, false);
	    return result;
}
/*
 * enable spi
 */
int32_t EnableSPI(uint8_t ucSPI_ID, bool bEnable)
{
	int32_t result = 0u;
	if(ucSPI_ID == 0)  /* SPI0 */
	{
		if(bEnable)   /* Enable */
		{
			*pREG_SPI0_TXCTL |= (ENUM_SPI_TXCTL_TX_EN); /* Enable the Tx  */
			*pREG_SPI0_CTL   |= (BITM_SPI_CTL_EN);      /* Enable the SPI */
		}else
		{
			*pREG_SPI0_TXCTL &= ~(ENUM_SPI_TXCTL_TX_EN); /* Disable the Tx */
			*pREG_SPI0_CTL   &= ~(BITM_SPI_CTL_EN);      /* Disable the SPI */
		}
	}
	else if(ucSPI_ID == 1) /* SPI1 */
	{
		if(bEnable) /* Enable */
		{
			*pREG_SPI1_RXCTL |= (ENUM_SPI_RXCTL_RX_EN); /* Enable the Rx  */
			*pREG_SPI1_CTL   |= (BITM_SPI_CTL_EN);      /* Enable the SPI */
		}else
		{
			*pREG_SPI1_RXCTL &= ~(ENUM_SPI_RXCTL_RX_EN); /* Disable the Rx */
			*pREG_SPI1_CTL   &= ~(BITM_SPI_CTL_EN);      /* Disable the SPI */
		}
	}

	return result;
}
/***********************************************************************************************
 * linkPort peripheral(User need setup the pin of link-port)
 *
 *  Created on: 2015-4-23
 *      Author: ChiliWang
 *************************************************************************************************/
#include <drivers/linkport/adi_linkport.h>

#define LP2_CLK_PORTE_MUX  ((uint32_t) ((uint32_t) 2<<18))
#define LP2_ACK_PORTE_MUX  ((uint32_t) ((uint32_t) 2<<16))
#define LP2_D0_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<0))
#define LP2_D1_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<2))
#define LP2_D2_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<4))
#define LP2_D3_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<6))
#define LP2_D4_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<8))
#define LP2_D5_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<10))
#define LP2_D6_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<12))
#define LP2_D7_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<14))
#define LP3_CLK_PORTE_MUX  ((uint16_t) ((uint16_t) 2<<12))
#define LP3_ACK_PORTE_MUX  ((uint16_t) ((uint16_t) 2<<14))
#define LP3_D0_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<16))
#define LP3_D1_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<18))
#define LP3_D2_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<20))
#define LP3_D3_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<22))
#define LP3_D4_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<24))
#define LP3_D5_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<26))
#define LP3_D6_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<28))
#define LP3_D7_PORTF_MUX  ((uint32_t) ((uint32_t) 2<<30))

#define LP2_CLK_PORTE_FER  ((uint32_t) ((uint32_t) 1<<9))
#define LP2_ACK_PORTE_FER  ((uint32_t) ((uint32_t) 1<<8))
#define LP2_D0_PORTF_FER  ((uint16_t) ((uint16_t) 1<<0))
#define LP2_D1_PORTF_FER  ((uint16_t) ((uint16_t) 1<<1))
#define LP2_D2_PORTF_FER  ((uint16_t) ((uint16_t) 1<<2))
#define LP2_D3_PORTF_FER  ((uint16_t) ((uint16_t) 1<<3))
#define LP2_D4_PORTF_FER  ((uint16_t) ((uint16_t) 1<<4))
#define LP2_D5_PORTF_FER  ((uint16_t) ((uint16_t) 1<<5))
#define LP2_D6_PORTF_FER  ((uint16_t) ((uint16_t) 1<<6))
#define LP2_D7_PORTF_FER  ((uint16_t) ((uint16_t) 1<<7))
#define LP3_CLK_PORTE_FER  ((uint16_t) ((uint16_t) 1<<6))
#define LP3_ACK_PORTE_FER  ((uint16_t) ((uint16_t) 1<<7))
#define LP3_D0_PORTF_FER  ((uint32_t) ((uint32_t) 1<<8))
#define LP3_D1_PORTF_FER  ((uint32_t) ((uint32_t) 1<<9))
#define LP3_D2_PORTF_FER  ((uint32_t) ((uint32_t) 1<<10))
#define LP3_D3_PORTF_FER  ((uint32_t) ((uint32_t) 1<<11))
#define LP3_D4_PORTF_FER  ((uint32_t) ((uint32_t) 1<<12))
#define LP3_D5_PORTF_FER  ((uint32_t) ((uint32_t) 1<<13))
#define LP3_D6_PORTF_FER  ((uint32_t) ((uint32_t) 1<<14))
#define LP3_D7_PORTF_FER  ((uint32_t) ((uint32_t) 1<<15))
/*
 * Initialize the Port Control MUX and FER Registers
 */
void Init_LP_Pin(void)
{
    /* PORTx_MUX registers */
    *pREG_PORTE_MUX |= LP2_CLK_PORTE_MUX | LP2_ACK_PORTE_MUX
     | LP3_CLK_PORTE_MUX | LP3_ACK_PORTE_MUX;
    *pREG_PORTF_MUX |= LP2_D0_PORTF_MUX | LP2_D1_PORTF_MUX
     | LP2_D2_PORTF_MUX | LP2_D3_PORTF_MUX | LP2_D4_PORTF_MUX
     | LP2_D5_PORTF_MUX | LP2_D6_PORTF_MUX | LP2_D7_PORTF_MUX
     | LP3_D0_PORTF_MUX | LP3_D1_PORTF_MUX | LP3_D2_PORTF_MUX
     | LP3_D3_PORTF_MUX | LP3_D4_PORTF_MUX | LP3_D5_PORTF_MUX
     | LP3_D6_PORTF_MUX | LP3_D7_PORTF_MUX;

    /* PORTx_FER registers */
    *pREG_PORTE_FER |= LP2_CLK_PORTE_FER | LP2_ACK_PORTE_FER
     | LP3_CLK_PORTE_FER | LP3_ACK_PORTE_FER;
    *pREG_PORTF_FER |= LP2_D0_PORTF_FER | LP2_D1_PORTF_FER
     | LP2_D2_PORTF_FER | LP2_D3_PORTF_FER | LP2_D4_PORTF_FER
     | LP2_D5_PORTF_FER | LP2_D6_PORTF_FER | LP2_D7_PORTF_FER
     | LP3_D0_PORTF_FER | LP3_D1_PORTF_FER | LP3_D2_PORTF_FER
     | LP3_D3_PORTF_FER | LP3_D4_PORTF_FER | LP3_D5_PORTF_FER
     | LP3_D6_PORTF_FER | LP3_D7_PORTF_FER;
}
/* Linkport devince number for TX*/
#define LINKPORT_DEVICE_NUM_TX  2U

/* Linkport devince number for RX*/
#define LINKPORT_DEVICE_NUM_RX  2U

/* Macro which controls the mode of operation.
 *  Enabling "ENABLE_DMA_MODE" will enable the DMA mode of operation
 */
 #define ENABLE_DMA_MODE


#ifdef ENABLE_DMA_MODE
#define LINKPORT_MEM_SIZE (ADI_LINKPORT_DMA_MEMORY_SIZE)
#else
#define LINKPORT_MEM_SIZE (ADI_LINKPORT_INT_MEMORY_SIZE)
#endif

/* Handle for RX  linkport device */
#pragma align(4)
ADI_LINKPORT_HANDLE          hDeviceRx;

/* Handle for TX linkport device */
#pragma align(4)
ADI_LINKPORT_HANDLE          hDeviceTx;

/* Memory required by the device for TX operation */
#pragma align(4)
unsigned char linkport_handlerTx[LINKPORT_MEM_SIZE];

/* Memory required by the device for RX operation */
#pragma align(4)
unsigned char linkport_handlerRx[LINKPORT_MEM_SIZE];

/*Rx callback counter */
volatile uint32_t   nRxCallbackCounter = 0U;

/* Tx callback counter */
volatile uint32_t   nTxCallbackCounter = 0U;

/* Function for Configuring the TX */
ADI_LINKPORT_RESULT  configureTx(void);


/* Callback function for TX  */
void LinkPort_CallbackRx(void  *pAppHandle,uint32_t  nEvent,void *pArg);

/* Callback function for RX  */
void LinkPort_CallbackTx(void  *pAppHandle,uint32_t  nEvent,void *pArg);


/*********************************************************************

    Function:       LINKPORTCallbackTx

    Description:    In callback mode of operation, This function is registered as
                    a callback function and will be called when the content of the
		    buffer is transmitted.


*********************************************************************/
void LinkPort_CallbackTx(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg
)
{
    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case(uint32_t) ADI_LINKPORT_EVENT_TX_BUFFER_PROCESSED:
               nTxCallbackCounter += 1U;
           break;
        default:
            break;
    }
    /* return */
}
/*********************************************************************

    Function:       LINKPORTCallbackRx

    Description:    This function is registered as a callback function for RX
                    and will be called when the buffer is full.

*********************************************************************/
void LinkPort_CallbackRx(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg
)
{
    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case (uint32_t)ADI_LINKPORT_EVENT_RX_BUFFER_PROCESSED:
            nRxCallbackCounter +=1U;
            break;
        default:
             break;
    }
    /* return */
}
/*********************************************************************

    Function:       configureTx

    Description:    Function for configuring the link port clock.

*********************************************************************/

static ADI_LINKPORT_RESULT  configureTx(void)
{
    ADI_LINKPORT_RESULT  elpResult;
    float freq, cycle;
    myGetSCLK0(&freq, &cycle);
    uint8_t div = freq / 20 / 2;
    elpResult = adi_linkport_ConfigClock(hDeviceTx, 2);
    return(elpResult);
}

/*
 * Init link-port
 * input  parameter :
 * output parameter :
 * return value		:
 */
ADI_LINKPORT_RESULT Init_LinkPort_FPGA(ADI_LINKPORT_DIRECTION dir)
{
    /* Variable for storing the return code from UART device */
    ADI_LINKPORT_RESULT  elpResult;

    if(dir == ADI_LINKPORT_DIR_TX)
    {
		do
			{
			/* Close the Link port device for RX */
			//if((elpResult = adi_linkport_Close(hDeviceRx)) != ADI_LINKPORT_SUCCESS)
			{
				//do nothing
			}
			/* Open the Link port device for TX*/
			if((elpResult = adi_linkport_Open(LINKPORT_DEVICE_NUM_TX,ADI_LINKPORT_DIR_TX,linkport_handlerTx,(uint32_t)LINKPORT_MEM_SIZE,&hDeviceTx)) != ADI_LINKPORT_SUCCESS)
			{
				printf("\n\tFailed to open the device in Rx mode ");
				break;
			}
			/* Configure the TX linkport */
			if((elpResult = configureTx())!= ADI_LINKPORT_SUCCESS)
			{
				printf("\n\t Failed to configure TX:   %d",elpResult);
				break;
			}

	#ifdef  ENABLE_DMA_MODE
			/* Enable the DMA associated with device if it is expected to work with DMA mode */
			if((elpResult = adi_linkport_EnableDMAMode(hDeviceTx,true))!= ADI_LINKPORT_SUCCESS)
			{
				printf("\n\t Failed to enable the DMA mode for TX:   %d",elpResult);
				break;
			}
	#endif

			   /* Register a call back function for TX device */
			if( (elpResult = adi_linkport_RegisterCallback(hDeviceTx,LinkPort_CallbackTx,NULL)) != ADI_LINKPORT_SUCCESS)
			{
			   printf("\n\t Failed to register the call back for Tx :  %d",elpResult);
			}


		}while(0);

    }else if(ADI_LINKPORT_DIR_RX == dir)
    {
    	do
			{
			/* Close the Link port device for TX*/
			if((elpResult = adi_linkport_Close(hDeviceTx)) != ADI_LINKPORT_SUCCESS)
			{
				//do nothing
			}
			/* Open the Link port device for RX */
			if((elpResult = adi_linkport_Open(LINKPORT_DEVICE_NUM_RX,ADI_LINKPORT_DIR_RX,linkport_handlerRx,(uint32_t)LINKPORT_MEM_SIZE,&hDeviceRx)) != ADI_LINKPORT_SUCCESS)
			{
				 printf("\n\tFailed to open the device in Tx mode ");
				 break;
			}

	#ifdef  ENABLE_DMA_MODE
			/* Enable the DMA associated with device if it is expected to work with DMA mode */
			if((elpResult = adi_linkport_EnableDMAMode(hDeviceRx,true))!= ADI_LINKPORT_SUCCESS)
			{
				printf("\n\t Failed to enable the DMA mode for RX:   %d",elpResult);
				break;
			}
	#endif

			/* Register a call back function for RX device */
			if( (elpResult = adi_linkport_RegisterCallback(hDeviceRx,LinkPort_CallbackRx,NULL)) != ADI_LINKPORT_SUCCESS)
			{
			   printf("\n\t Failed to register the call back for Rx :  %d",elpResult);
			}

		}while(0);
    }

    return elpResult;
}
/*
 * BF609 GPIO INIT for FPGA control
 */
static int32_t Init_FPGA_Ctl_GPIO(void)
{
	ADI_GPIO_RESULT result = ADI_GPIO_SUCCESS;

    /* set SPI0 MIS0 as GPIO function used for tx CS control */
	if (result == 0u)
	{
	    result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_D,
			ADI_GPIO_PIN_2,
			ADI_GPIO_DIRECTION_OUTPUT);
	}

	/* set SPI1 MOSI as GPIO function used for Rx CS control */
    result = adi_gpio_SetDirection(
		ADI_GPIO_PORT_D,
		ADI_GPIO_PIN_13,
		ADI_GPIO_DIRECTION_OUTPUT);

    /* set PE2 as GPIO function used for FT3 Send trigger */
	if (result == 0u)
	{
	    result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_E,
			ADI_GPIO_PIN_2,
			ADI_GPIO_DIRECTION_OUTPUT);
	}

	/* Set GPIO default level */
	adi_gpio_Set(ADI_GPIO_PORT_D,ADI_GPIO_PIN_2); 	/* CS in high default          */
	adi_gpio_Set(ADI_GPIO_PORT_D,ADI_GPIO_PIN_13);	/* CS in high default          */
	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2); /* FPGA trigger in low default */

	return result;
}
/*
 *  Init_BF609_FPGA_Comm
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int Init_BF609_FPGA_Comm(void)
{
	/* LP pin-mux init */
	Init_LP_Pin();

	/* BF609 GPIO INIT for FPGA control */
	Init_FPGA_Ctl_GPIO();

	/* Init the IRQ pin */
	Init_FPGA_IRQ_Pin();

	/* Init SPI0 for Tx*/
	Init_SPI0_FPGA();

	/* Init SPI for Rx*/
	Init_SPI1_FPGA();

	return 0;
}
/*
 *  Init_BF609_FPGA_Frame
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int Init_BF609_FPGA_Frame(void *pBF609_FPGA_data)
{
//	SYNC_IN_CFG  RefInCfg , MeterInCfg;
//	SYNC_OUT_CFG Out1Cfg,Out2Cfg,Out3Cfg,Out4Cfg;
//	FT3_TX_PARA_CFG FT3_Tx_Para_Cfg;
//	CfgSPORT2B_Device(pBF609_FPGA_data, ADS1278);
//
//	RefInCfg.Port  = INTERNAL_PPS;
//	RefInCfg.Type  = PPS;
//	RefInCfg.Logic = NORMAL;     /* normal or reverse */
//	CfgRefInSrc(pBF609_FPGA_data, RefInCfg);
//
//	MeterInCfg.Port  = IN1;
//	MeterInCfg.Type  = PPS;
//	MeterInCfg.Logic = NORMAL;   /* normal or reverse */
//	CfgMeterInSrc(pBF609_FPGA_data, MeterInCfg);
//
//
//	Out1Cfg.Port 	 = OUT1;
//	Out1Cfg.Type 	 = PPS;
//	Out1Cfg.Logic	 = NORMAL;
//	Out1Cfg.Status   = RUN;
//	CfgSyncOutx(pBF609_FPGA_data, Out1Cfg);
//
//	Out2Cfg.Port 	       = OUT2;
//	Out2Cfg.Type 	       = IRIG_B;
//	Out2Cfg.Logic	  	   = NORMAL;
//	Out2Cfg.BcodeCheck 	   = EVEN_CHECK;
//	Out2Cfg.DelayMode      = BEFORE;
//	Out2Cfg.Delay_Per_25ns = 0;
//	Out2Cfg.Status         = RUN;
//	CfgSyncOutx(pBF609_FPGA_data, Out2Cfg);
//
//
//	FT3_Tx_Para_Cfg.FT3_TxPort     = FT3_TX_PORT_1;
//	FT3_Tx_Para_Cfg.usFT3_TxHead   = 0x0564;
//	FT3_Tx_Para_Cfg.ucFT3_FrameLen = 74;
//	FT3_Tx_Para_Cfg.FT3_Data_Logic = NORMAL;
//	FT3_Tx_Para_Cfg.FT3_Mode       = FT3_ASYNC;
//	FT3_Tx_Para_Cfg.FT3_BaudRate   = FT3_5M_BITS;
//	CfgFT3_TxPort(pBF609_FPGA_data, FT3_Tx_Para_Cfg);
//
//	FT3_Tx_Para_Cfg.FT3_TxPort     = FT3_TX_PORT_2;
//	CfgFT3_TxPort(pBF609_FPGA_data, FT3_Tx_Para_Cfg);
//
//	FT3_Tx_Para_Cfg.FT3_TxPort     = FT3_TX_PORT_4;
//	CfgFT3_TxPort(pBF609_FPGA_data, FT3_Tx_Para_Cfg);
//
//	FT3_Tx_Para_Cfg.FT3_TxPort     = FT3_TX_PORT_5;
//	CfgFT3_TxPort(pBF609_FPGA_data, FT3_Tx_Para_Cfg);
//
//	FT3_Tx_Para_Cfg.FT3_TxPort     = FT3_TX_PORT_6;
//	CfgFT3_TxPort(pBF609_FPGA_data, FT3_Tx_Para_Cfg);


	return 0;
}

/*
 *  Send FPGA CMD
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int Send_FPGA_Cfg(void *pBF609_FPGA_data)
{
	int result = -1;
	if(pBF609_FPGA_data != 	NULL)
	{
		spi0_Tx_Rx_buffer.pTransmitter 	  = pBF609_FPGA_data;
		spi0_Tx_Rx_buffer.TransmitterBytes = sizeof(pBF609_FPGA_data);
		result = adi_spi_SubmitBuffer(hSPI0, &spi0_Tx_Rx_buffer); //give the empty buffer to system, we can read-back in SPI call-back.
		if(result == 0)
		{
			adi_gpio_Clear(ADI_GPIO_PORT_D,ADI_GPIO_PIN_2);  /* CS low */
			result = EnableSPI(0, true);
		}

	}


	return result;
}
/*
 *  Recv FPGA CMD
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int32_t Recv_FPGA_Info(void *pFPGA_BF609_data)
{
	int result = 0;
	if(pFPGA_BF609_data != 	NULL)
	{
		spi0_Tx_Rx_buffer.pReceiver        = pFPGA_BF609_data;
		spi0_Tx_Rx_buffer.ReceiverBytes    = sizeof(pFPGA_BF609_data);
		result = adi_spi_SubmitBuffer(hSPI1, &spi1_Tx_Rx_buffer); //give the empty buffer to system, we can read-back in SPI call-back.
		if(result == 0)
		{
			adi_gpio_Clear(ADI_GPIO_PORT_D,ADI_GPIO_PIN_13);  /* CS low */
			result = EnableSPI(1, true);
		}

	}
	return result;
}
/*
 *  Send FT3 data
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int32_t Send_FT3_Data(void *pBF609_FPGA_data, uint16_t size)
{
	ADI_LINKPORT_RESULT  elpResult = ADI_LINKPORT_SUCCESS;

//	/* Init linkport for Tx */
//	if((elpResult = Init_LinkPort_FPGA(ADI_LINKPORT_DIR_TX)) != ADI_LINKPORT_SUCCESS)
//	{
//        printf("\n\t Failed to Init the Tx channel:  %d",elpResult);
//	}

    /* Submit the  buffer for Tx  */
    if((elpResult = adi_linkport_SubmitBuffer(hDeviceTx,pBF609_FPGA_data, (size+3) / 4U))!= ADI_LINKPORT_SUCCESS)
    {
       printf("\n\t Failed to submit the buffer for Tx channel:  %d",elpResult);
    }

    /* Trigge the FPGA for FT3 receive */
	adi_gpio_Set(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

	/* Enable the data flow for TX */
	if((elpResult = adi_linkport_Enable(hDeviceTx, true))!= ADI_LINKPORT_SUCCESS)
	{
	   printf("\n\t Failed to disable the Tx:  %d",elpResult);
	}
	return elpResult;
}
/*
 *  Recv FT3 data
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int32_t Recv_FT3_Data(void *pFPGA_BF609_data)
{

	ADI_LINKPORT_RESULT  elpResult = ADI_LINKPORT_SUCCESS;
	/* Init linkport for Rx */
	if((elpResult = Init_LinkPort_FPGA(ADI_LINKPORT_DIR_RX)) != ADI_LINKPORT_SUCCESS)
	{
        printf("\n\t Failed to Init the Rx channel:  %d",elpResult);
	}

    /* Submit the  buffer for Tx  */
    if((elpResult = adi_linkport_SubmitBuffer(hDeviceRx,pFPGA_BF609_data, sizeof(pFPGA_BF609_data) / 4U))!= ADI_LINKPORT_SUCCESS)
    {
        printf("\n\t Failed to submit the buffer for Rx channel:  %d",elpResult);
    }
	/* Enable the data flow for TX after registering the call back */
	if((elpResult = adi_linkport_Enable(hDeviceRx, true))!= ADI_LINKPORT_SUCCESS)
	{
	   printf("\n\t Failed to disable the Rx:  %d",elpResult);
	}
	return elpResult;
}
