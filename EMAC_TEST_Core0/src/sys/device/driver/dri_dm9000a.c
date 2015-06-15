/*
 * dri_dm9000a.c
 *
 *  Created on: 2015-3-30
 *      Author: Administrator
 */


/*
 * DM9000A.c
 *
 *  Created on: 2015-3-5
 *      Author: ChiliWang
 */
/*
 * SMC pin MUX
 */
#define SMC0_A03_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<0))
#define SMC0_A04_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<2))
#define SMC0_A05_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define SMC0_A06_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define SMC0_A07_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define SMC0_A08_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define SMC0_A09_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define SMC0_A10_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<14))
#define SMC0_A11_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<16))
#define SMC0_A12_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<18))
#define SMC0_A13_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define SMC0_A14_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<20))
#define SMC0_A15_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<22))
#define SMC0_A16_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define SMC0_A17_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<24))
#define SMC0_A18_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<26))
#define SMC0_A19_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<28))
#define SMC0_A20_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<30))
#define SMC0_A21_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define SMC0_A22_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<14))
#define SMC0_A23_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<16))
#define SMC0_A24_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<20))
#define SMC0_A25_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<22))
#define SMC0_AMS1_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<2))
#define SMC0_AMS2_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define SMC0_AMS3_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define SMC0_NORCLK_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<0))
#define SMC0_BG_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<24))
#define SMC0_BGH_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<18))


#define SMC0_A03_PORTA_FER  ((uint16_t) ((uint16_t) 1<<0))
#define SMC0_A04_PORTA_FER  ((uint16_t) ((uint16_t) 1<<1))
#define SMC0_A05_PORTA_FER  ((uint16_t) ((uint16_t) 1<<2))
#define SMC0_A06_PORTA_FER  ((uint16_t) ((uint16_t) 1<<3))
#define SMC0_A07_PORTA_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SMC0_A08_PORTA_FER  ((uint16_t) ((uint16_t) 1<<5))
#define SMC0_A09_PORTA_FER  ((uint16_t) ((uint16_t) 1<<6))
#define SMC0_A10_PORTA_FER  ((uint16_t) ((uint16_t) 1<<7))
#define SMC0_A11_PORTA_FER  ((uint32_t) ((uint32_t) 1<<8))
#define SMC0_A12_PORTA_FER  ((uint32_t) ((uint32_t) 1<<9))
#define SMC0_A13_PORTB_FER  ((uint16_t) ((uint16_t) 1<<2))
#define SMC0_A14_PORTA_FER  ((uint32_t) ((uint32_t) 1<<10))
#define SMC0_A15_PORTA_FER  ((uint32_t) ((uint32_t) 1<<11))
#define SMC0_A16_PORTB_FER  ((uint16_t) ((uint16_t) 1<<3))
#define SMC0_A17_PORTA_FER  ((uint32_t) ((uint32_t) 1<<12))
#define SMC0_A18_PORTA_FER  ((uint32_t) ((uint32_t) 1<<13))
#define SMC0_A19_PORTA_FER  ((uint32_t) ((uint32_t) 1<<14))
#define SMC0_A20_PORTA_FER  ((uint32_t) ((uint32_t) 1<<15))
#define SMC0_A21_PORTB_FER  ((uint16_t) ((uint16_t) 1<<6))
#define SMC0_A22_PORTB_FER  ((uint16_t) ((uint16_t) 1<<7))
#define SMC0_A23_PORTB_FER  ((uint32_t) ((uint32_t) 1<<8))
#define SMC0_A24_PORTB_FER  ((uint32_t) ((uint32_t) 1<<10))
#define SMC0_A25_PORTB_FER  ((uint32_t) ((uint32_t) 1<<11))
#define SMC0_AMS1_PORTB_FER  ((uint16_t) ((uint16_t) 1<<1))
#define SMC0_AMS2_PORTB_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SMC0_AMS3_PORTB_FER  ((uint16_t) ((uint16_t) 1<<5))
#define SMC0_NORCLK_PORTB_FER  ((uint16_t) ((uint16_t) 1<<0))
#define SMC0_BG_PORTB_FER  ((uint32_t) ((uint32_t) 1<<12))
#define SMC0_BGH_PORTB_FER  ((uint32_t) ((uint32_t) 1<<9))


#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <ccblkfn.h>
#include <math.h>
#include <services/gpio/adi_gpio.h>

#include "dri_adi_gemac.h"
#include "dri_dm9000a.h"

#include "dev_pwr.h"
#include "dev_gpio.h"
#include "dev_smc.h"

#include "post_debug.h"

#include "loop_queue.h"



typedef LoopQueue EXETH_RECV_Queue;

section ("sdram_bank0") EXETH_RECV_Queue g_ExEthRecvQueue;



static uint8_t DM9000A_MAC[6] = {0x00,0x02,0x03,0x04,0x05,0x06};//{0xe1,0x2e,0x4f,0xff,0x55,0xab};
static uint8_t MulticastFilter[8]   = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//{0xff , 0xff , 0xff , 0xff , 0xff, 0xff};

#define HAL_VER1_0 0


/*
 * DM9000A IO operation definition.
 */
#if HAL_VER1_0
#define  pADDR_DM9000A  	((volatile uint16_t *)(0xBC800000))
#define  pDATA_DM9000A  	((volatile uint16_t *)(0xBC800008))
#else
#define  pADDR_DM9000A  	((volatile uint16_t *)(0xBC000000))
#define  pDATA_DM9000A  	((volatile uint16_t *)(0xBE000008))
#endif

/*=============  D A T A  =============*/
/* DMA Manager includes */
#include <services/dma/adi_dma.h>

/* DMA Stream Handle */
ADI_DMA_STREAM_HANDLE   hMemDmaStream3, hMemDmaStream2;
/* Source DMA Handle */
ADI_DMA_CHANNEL_HANDLE  hSrcDmaChannel3, hSrcDmaChannel2;
/* Destination DMA Handle */
ADI_DMA_CHANNEL_HANDLE  hDestDmaChannel3, hDestDmaChannel2;
/* Memory to handle DMA Stream */
static uint8_t MemDmaStreamMem3[160u];
static uint8_t MemDmaStreamMem2[160u];

/*
 * Word/Memory transfer size to use for this example
 * Enable only one word transfer size
 */
//#define MEMCOPY_XFER_1BYTE		/* Enable macro for 1 byte transfer */
#define MEMCOPY_XFER_2BYTES		/* Enable macro for 2 bytes transfer */
//#define MEMCOPY_XFER_4BYTES		/* Enable macro for 4 bytes transfer */

/* Word/Memory transfer size in bytes */
#if defined (MEMCOPY_XFER_4BYTES)	/* 4 bytes transfer */
#define MEMCOPY_MSIZE               ADI_DMA_MSIZE_4BYTES
#define MEMCOPY_MSIZE_IN_BYTES      (4u)
#elif defined (MEMCOPY_XFER_2BYTES)	/* 2 bytes transfer */
#define MEMCOPY_MSIZE               ADI_DMA_MSIZE_2BYTES
#define MEMCOPY_MSIZE_IN_BYTES      (2u)
#else								/* 1 byte transfer */
#define MEMCOPY_MSIZE               ADI_DMA_MSIZE_1BYTE
#define MEMCOPY_MSIZE_IN_BYTES      (1u)
#endif
/*
 * MDMA Stream ID to use for this example
 */
#define MEMCOPY_STREAM_ID3           ADI_DMA_MEMDMA_S3       /* Stream 3 */
#define MEMCOPY_STREAM_ID2           ADI_DMA_MEMDMA_S2       /* Stream 3 */

volatile int TxMDMA3_Param;
volatile int RxMDMA2_Param;
volatile int sended = 0;
volatile int recved = 0;
volatile bool g_bTxMDMAIsReady = true;
volatile bool g_bRxMDMAIsReady = true;


static void TxMDAM_Callback3 (void *pCBParam, uint32_t event, void *pArg);
static void RxMDAM_Callback2 (void *pCBParam, uint32_t event, void *pArg);


/*
 * memcpy DMA init
 */
static uint32_t Init_MDMA_DM9000A(void)
{
	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;


	/* Open a Memory DMA Stream */
	eResult = adi_mdma_Open (MEMCOPY_STREAM_ID3,
							 &MemDmaStreamMem3[0],
							 &hMemDmaStream3,
							 &hSrcDmaChannel3,
							 &hDestDmaChannel3,
							 TxMDAM_Callback3,
							(void*) &TxMDMA3_Param);

	/* IF (Failure) */
	if (eResult != ADI_DMA_SUCCESS)
	{
		return 0;
	}

	/* Open a Memory DMA Stream */
	eResult = adi_mdma_Open (MEMCOPY_STREAM_ID2,
						&MemDmaStreamMem2[0],
						&hMemDmaStream2,
						&hSrcDmaChannel2,
						&hDestDmaChannel2,
						RxMDAM_Callback2,
					   (void*) &RxMDMA2_Param);

	/* IF (Failure) */
	if (eResult != ADI_DMA_SUCCESS)
	{
		return 0;
	}

    return 1;
}




void WriteReg(uint8_t RegAddr, uint8_t RegValue)
{
	*pADDR_DM9000A = RegAddr;

	int clk = 2;//188ns
	while(clk--)
	{
		asm(" NOP; ");
	}

	*pDATA_DM9000A = RegValue;
}


uint8_t ReadReg(uint8_t RegAddr)
{
	*pADDR_DM9000A =  RegAddr;

	int clk = 3;//188ns
	while(clk--)
	{
		asm(" NOP; ");
	}

	return (*pDATA_DM9000A);
}

void phy_w (unsigned char phy_offset , unsigned int reg_data)
{
	WriteReg(EPAR , phy_offset | 0x40); //;;將要寫入寄存位設置於EPAR
	WriteReg(EPDRH , reg_data >> 8);	//;;資料的High Byte 置入EPDRH
	WriteReg(EPDRL , reg_data & 0xff);  //;;資料的Low Byte 置入EPDRL
	WriteReg(EPCR , 0x0a);  			//;;開始進行寫入

	int clk = 7500;   				    //;;delay 150us 減少下面指令對BUS 動作

	while(clk--)
	{
		asm("NOP;");
	}

	while((ReadReg(EPCR) & 0x01) == 0x01); 	//;;反覆確定DM9000A 是否完成

	WriteReg(EPCR , 0x00);  		//;;寫入完成， 回復正常模式
}

void ReadID(void)
{
	uint8_t vendorID1 = ReadReg(0x28);
	uint8_t vendorID2 = ReadReg(0x29);

	uint8_t PID1 = ReadReg(0x2A);
	uint8_t PID2 = ReadReg(0x2B);

}

static void Setup_DM9000A(uint8_t* dstMac)
{
	*pBANK1_EMAC_CTR_BASE    &= ~(1u << 1);

	int clk = 2000;//>5us,  3000
	while(clk--)
	{
		asm(" NOP; ");
	}
	*pBANK1_EMAC_CTR_BASE    |= (1u << 1);

	// 2015-6-11
	clk = 2000;//>5us,  3000
	while(clk--)
	{
		asm(" NOP; ");
	}
	///

	for(int i  = 0; i < 2; i++)/* power on phy in the dm9000, setup twice to make sure done. */
	{
		/* power on phy in the dm9000 */
		WriteReg( GPR, 0x00 );
		/* wait for phy power on*/
		clk = 1000;//200us 100000
		while(clk--)
		{
			asm(" NOP; ");
		}
	}


	/* software reset 1, setup twice to make sure done. */
	WriteReg( NCR, 0x03 );
	clk = 1000;//20us 10000
	while(clk--)
	{
		asm(" NOP; ");
	}
	WriteReg( NCR, 0x00 );
	/* software reset 2, setup twice to make sure done. */
	WriteReg( NCR, 0x03 );
	clk = 1000;//20us 10000
	while(clk--)
	{
		asm(" NOP; ");
	}
	WriteReg( NCR, 0x00 );


//	WriteReg( NCR, 0x00 ); //2015-6-11



//	/* setup 100M full-duplex mode */
//	phy_w(0x00 , 0x2100); // 2015-6-11

	/* setup 100M full-duplex mode */
	phy_w(0x00 , 0x2100);
	/* power on phy in the dm9000 */
	WriteReg( GPR, 0x00 );
	/* wait for phy power on*/
	clk = 1000;//20us 10000
	while(clk--)
	{
		asm(" NOP; ");
	}

	/* MAC address set */
	if(dstMac)
	{
		memcpy(DM9000A_MAC, dstMac, 6 );
	}

	for(int i = 0; i < 6; i++)
	{
		WriteReg(PAR_BASE + i, DM9000A_MAC[i]);
	}

	for(int i = 0; i < 8; i++)
	{
		WriteReg(MAR_BASE + i, MulticastFilter[i]);
	}


	/* turn off the INT to prevent INIT err */
	WriteReg(IMR, 0x80);

	/*read NSR, clear legacy status bit*/
	ReadReg( NSR );

	/* Clear TCR */
	WriteReg(TCR, 0x00);

	/* Clear ROCR */
	ReadReg(ROCR);

	/* Clear ISR */
	WriteReg(ISR, 0xff);

	WriteReg(WUCR, 0x00);
	/* Transmit Control Register 2, set in LED mode1 */
	WriteReg(0x2d, 0x80);



	/*( 0x1 | 0x02 | ( 1 << 7 ) )
	 * received INT and Enable transmitted
	 * Enable address pointer automatically toggle to start when pointer reached summit
	 */
	WriteReg( IMR, (0x01 | ( 1 << 7 ) ));
//		uint8_t imr = ReadReg(IMR);//131

	/*
	 * Enable Rx
	 * Enable Promiscuous Mode( 1 << 1 )
	 * Pass all multi-frame (1 << 3)
	 * Enable Discard CRC Error Packet
	 * Enable Discard Long Packet : Packet length is over 1522byte
	 */
//	WriteReg( RCR, ( 0x1 | ( 1 << 4 ) | ( 1 << 5 ) ) );
	WriteReg( RCR, ( 0x1 ) );
//		uint8_t rcr = ReadReg(RCR);//49

	/* auto send when 75% frame length */
//		WriteReg(ETCSR, 0x83);
}


/*
 * DM9000A software reset
 */
static void DM9000A_Reset(void)
{
	/* 复位标志，复位后第一帧不需要对齐发送指针 */
//	GbReset = 0;

	WriteReg(NCR, 3);			/* 对 DM9000A 进行软件重置 */
	int clk = 25000;//10us
	while(clk--)
	{
		asm(" NOP; ");
	}
	WriteReg(NCR, 0);

	/* MAC address set */
	for(int i = 0; i < 6; i++)
	{
		WriteReg(PAR_BASE + i, DM9000A_MAC[i]);
	}
	for(int i = 0; i < 8; i++)
	{
		WriteReg(MAR_BASE + i, 0);
	}

	WriteReg(IMR, 0x81);			/* 开启 中断模式 */

	WriteReg(RCR, 0x31);			/* 开启 接收工能 */

}

static int Process_DM9000A_Recv( uint8_t * buf, uint8_t RxReady)
{

	uint16_t RxStatus = 0;
	uint16_t RxLen = 0;
	uint16_t HalfRxLen;
	uint16_t MRR_cal;

	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;

	// modified by wjm@2015-5-12

		/* start write the Read FIFO */
		*pADDR_DM9000A = MRCMD;
		*pADDR_DM9000A = MRCMD;


#if HAL_VER1_0
		/* Use DMA we should cancel the multiplex. */
		*pREG_PORTA_FER &= ~SMC0_A03_PORTA_FER;
		/* set the A03 to 1, means always write data in DMA process. */
		*pREG_PORTA_DATA_SET = ADI_GPIO_PIN_0;
#else

//		*pREG_PORTB_FER &= ~SMC0_A25_PORTB_FER;
//		*pREG_PORTB_DATA_SET = ADI_GPIO_PIN_11;
#endif

		RxStatus  = *pDATA_DM9000A;
		RxLen 	= *pDATA_DM9000A;


		/* 调试发现，读取rx_length时地址没有自增， 得到的是status的值，判断后反复读取 */
		while(RxStatus == RxLen)
		{
			RxLen = *pDATA_DM9000A;
		}

		*(uint16_t*)buf = RxLen; // 帧长度

		HalfRxLen = (RxLen + 1) >> 1;

#if 1

		eResult = adi_mdma_Copy1D (hMemDmaStream2, (void*)(buf+2), (void *)pDATA_DM9000A,
				ADI_DMA_MSIZE_2BYTES,
				HalfRxLen);
		if(eResult != ADI_DMA_SUCCESS)
		{
			DEBUG_PRINT("Process_DM9000A_Recv: failed to copy memory : %d \n", eResult);
		}

#else
		for(int i = 0; i < HalfRxLen; i++ )
		{
			(( uint16_t *)(buf+2))[i] = *pDATA_DM9000A;
		}
#endif

	return 1;
}

//rx dma
static void RxMDAM_Callback2 (void *pCBParam, uint32_t event, void *pArg)
{
	int len = *((int *)pCBParam);


	switch ( event )
	{
		case ADI_DMA_EVENT_BUFFER_PROCESSED:
#if HAL_VER1_0
			/* Resume A03 in SMC mode */
			*pREG_PORTA_FER |= SMC0_A03_PORTA_FER;
			g_bRxMDMAIsReady = true;
#else
			/* Resume A25 in SMC mode */
//			*pREG_PORTB_FER |= SMC0_A25_PORTB_FER;
			g_bRxMDMAIsReady = true;
#endif
			break;
		default:
			break;
	}
}


static void Process_DM9000A_INT_Event( void )
{
	LoopQueueItem* pItem = NULL;

	uint8_t RxReady = 0;

	uint16_t status = ReadReg( ISR );

//	uint8_t RxStatusRegister;
	if( status & 0x01 )//接收包中断
	{
		/* 读取Rx Ready，不偏移内存指针 */
		ReadReg( MRCMDX );
		RxReady = ReadReg( MRCMDX );

		while(RxReady & 0x01 == 1)
		{
//			RxStatusRegister = ReadReg(0x06);// modified by wjm@2015-5-12

			ENTER_CRITICAL_REGION();

			pItem =  LoopQueue_push( &g_ExEthRecvQueue );

			EXIT_CRITICAL_REGION();

			if( pItem != NULL )
			{
//				pItem->Size = RxStatusRegister;// modified by wjm@2015-5-12
//				MDMA2_Param = (void*)pItem->Data;
				g_bRxMDMAIsReady = false;
				Process_DM9000A_Recv( pItem->Data, RxReady);
			}

			//wait for RX DMA work over to release SMC.
			while(!g_bRxMDMAIsReady)
			{

			}

			/* 读取Rx Ready，不偏移内存指针 */
			ReadReg( MRCMDX );
			RxReady = ReadReg( MRCMDX );

		}

		WriteReg( ISR, 0x01 );//清除中断

		/* Ready状态不是0或者1为异常，需要重启 */
		// add by wjm@2015-5-12
		if(RxReady & 0xfe )
		{
			DM9000A_Reset();
			return ;
		}
	}

	if(status & 0x02)//发送包中断
	{
		WriteReg( ISR, 0x02 );//清除中断
	}
}

/*
 * DM9000A Send
 */
int DM9000A_Core_Send( void * buf, int len )
{
	/*1036耗时100us*/

	int tmp_length = ( len + 1 ) >> 1 ;
	int i;
	static uint32_t cnt = 1;
	uint8_t nsr;
	uint16_t TRPA;
	uint16_t sram_addr;
	/*
	* Setup fast send mode, when FIFO get 75%, DM9000a send auto.
	*/
	//WriteReg(ETCSR, 0x83);


	*pADDR_DM9000A = MWCMD;
	sended = *pTCOUNT;
	for( i = 0; i < tmp_length; i++ )
	*pDATA_DM9000A = ((uint16_t*)buf)[i];
	recved = *pTCOUNT;
	sended = sended - recved;


	WriteReg( TXPLH, ( len >> 8 ) & 0xff );
	WriteReg( TXPLL, len & 0xff );

	WriteReg( TCR, 0x1 );

	cnt++;
	return 0;
}
/*
 * Send data with DMA.
 */
int DM9000A_DMA_Send( void * buf, int len )
{

	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;

	uint16_t TRPA = 1;
	static uint32_t cnt = 0;
	//ENTER_CRITICAL_REGION();
	/* 从第二帧开始对齐dm9000a的发送指针 */

//	if(cnt)
//	{
//
//		*pREG_PORTA_FER |= SMC0_A03_PORTA_FER;
//		TRPA = ReadReg(0x23);
//		TRPA = (TRPA << 8) + ReadReg(0x22) - 4;
//		if(TRPA <= 0XBFF)
//		{
//			WriteReg(MWRH, TRPA >> 8);
//			WriteReg(MWRL, TRPA);
//		}
//		else
//		{
//			TRPA += (int)TRPA + 0XC00;
//			WriteReg(MWRH, TRPA >> 8);
//			WriteReg(MWRL, TRPA);
//		}
//
//	}


	/* start write the send FIFO */
	*pADDR_DM9000A = MWCMD;

#if HAL_VER1_0
	/* Use DMA we should cancel the multiplex. */
	*pREG_PORTA_FER &= ~SMC0_A03_PORTA_FER;
	/* set the A03 to 1, means always write data in DMA process. */
	*pREG_PORTA_DATA_SET = ADI_GPIO_PIN_0;
#else
	/* Use DMA we should cancel the multiplex. */
//		*pREG_PORTB_FER &= ~SMC0_A25_PORTB_FER;
//		/* set the A25 to 1, means always write data in DMA process. */
//		*pREG_PORTB_DATA_SET = ADI_GPIO_PIN_11;
#endif

	TxMDMA3_Param = len;

	g_bTxMDMAIsReady = false;

	eResult = adi_mdma_Copy1D (hMemDmaStream3, (void *)pDATA_DM9000A, buf, ADI_DMA_MSIZE_2BYTES, (len +1 ) >> 1);
	if(eResult != ADI_DMA_SUCCESS)
	{
		DEBUG_PRINT("failed to copy memory : %d! \n", eResult);
	}
//	EXIT_CRITICAL_REGION();
	cnt = 1;

	return 1;


}

static bool MDMAIsReady(void)
{
	return g_bTxMDMAIsReady;
}

static bool IsSendOver(void)
{
	return( (ReadReg(0x02) & 0x01 ) == 0x0 );
}

// tx DMA
static void TxMDAM_Callback3 (void *pCBParam, uint32_t event, void *pArg)
{

	int len = *((int *)pCBParam);

	switch ( event )
	{
		case ADI_DMA_EVENT_BUFFER_PROCESSED:
			//StartSend_DM9000A(len);

#if HAL_VER1_0
			/* Resume A03 in SMC mode */
			*pREG_PORTA_FER |= SMC0_A03_PORTA_FER;
#else
			/* Resume A25 in SMC mode */
//			*pREG_PORTB_FER |= SMC0_A25_PORTB_FER;
#endif

			WriteReg( TXPLH, ( len >> 8 ) & 0xff );
			WriteReg( TXPLL, len & 0xff );

			/* start send */
			WriteReg( TCR, 0x1 );

			g_bTxMDMAIsReady = true;

			break;
		default:
			break;
	}
}




/*
 * DM9000A_IRQ_Init
 *
 */
static ADI_GPIO_RESULT Init_IRQ_DM9000A(void)
{
	ADI_GPIO_RESULT result;


#if HAL_VER1_0

	/* set GPIO output DM9000A PIIN A03 */
	result = adi_gpio_SetDirection(
		ADI_GPIO_PORT_A,
		ADI_GPIO_PIN_0,
		ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		printf("%s failed\n", result);
	}

#else

	/* set GPIO output DM9000A PIIN A25 */
	result = Set_GPIO_PB11_IODirection(ADI_GPIO_DIRECTION_OUTPUT );

#endif
	/*
	 * PD9 is IRQ pin for DM9000A, Setup in input mode
	 */
	result = Set_GPIO_PD09_IODirection(ADI_GPIO_DIRECTION_INPUT);



	return result;
}


/*
 * control dm9000a with SMC module bank1，so we should configure the SMC
 */
static uint32_t Setup_SMC_DM9000A(void)
{

	float freq, cycle;
	int bclk;
	int wst, wht, wat, rst, rht, rat;
	int prest, preat, tt, it, pgws;
	uint32_t smc_b3ctl, smc_b3tim, smc_b3etim;

    /* SMC0 PORTx_MUX registers */
	*pREG_PORTA_MUX |= SMC0_A03_PORTA_MUX | SMC0_A04_PORTA_MUX
	 | SMC0_A05_PORTA_MUX | SMC0_A06_PORTA_MUX | SMC0_A07_PORTA_MUX
	 | SMC0_A08_PORTA_MUX | SMC0_A09_PORTA_MUX | SMC0_A10_PORTA_MUX
	 | SMC0_A11_PORTA_MUX | SMC0_A12_PORTA_MUX | SMC0_A14_PORTA_MUX
	 | SMC0_A15_PORTA_MUX | SMC0_A17_PORTA_MUX | SMC0_A18_PORTA_MUX
	 | SMC0_A19_PORTA_MUX | SMC0_A20_PORTA_MUX;
	*pREG_PORTB_MUX |= SMC0_A13_PORTB_MUX | SMC0_A16_PORTB_MUX
	 | SMC0_A21_PORTB_MUX | SMC0_A22_PORTB_MUX | SMC0_A23_PORTB_MUX
	 | SMC0_A24_PORTB_MUX | SMC0_A25_PORTB_MUX | SMC0_AMS1_PORTB_MUX
	 | SMC0_AMS2_PORTB_MUX | SMC0_AMS3_PORTB_MUX | SMC0_NORCLK_PORTB_MUX
	 | SMC0_BG_PORTB_MUX | SMC0_BGH_PORTB_MUX;

    /* SMC0 PORTx_FER registers */
    *pREG_PORTA_FER |= SMC0_A03_PORTA_FER | SMC0_A04_PORTA_FER
     | SMC0_A05_PORTA_FER | SMC0_A06_PORTA_FER | SMC0_A07_PORTA_FER
     | SMC0_A08_PORTA_FER | SMC0_A09_PORTA_FER | SMC0_A10_PORTA_FER
     | SMC0_A11_PORTA_FER | SMC0_A12_PORTA_FER | SMC0_A14_PORTA_FER
     | SMC0_A15_PORTA_FER | SMC0_A17_PORTA_FER | SMC0_A18_PORTA_FER
     | SMC0_A19_PORTA_FER | SMC0_A20_PORTA_FER;
    *pREG_PORTB_FER |= SMC0_A13_PORTB_FER | SMC0_A16_PORTB_FER
     | SMC0_A21_PORTB_FER | SMC0_A22_PORTB_FER | SMC0_A23_PORTB_FER
     | SMC0_A24_PORTB_FER | SMC0_A25_PORTB_FER | SMC0_AMS1_PORTB_FER
     | SMC0_AMS2_PORTB_FER | SMC0_AMS3_PORTB_FER | SMC0_NORCLK_PORTB_FER
     | SMC0_BG_PORTB_FER | SMC0_BGH_PORTB_FER;

    GetSCLK0(&freq, &cycle);

	/* Write setup time, 0ns + Write operation interval 20ns */
	wst = ceilf(2 / cycle);//3

	/* Write access time >= 10ns */
	wat = ceilf (12 / cycle);//10

	/* Write hold time , >=3ns*/
	wht = ceilf(5 / cycle);//3


	/* Read setup time , 0ns + read operation interval 40ns*/
	rst = ceilf (3 / cycle);//5
	if (rst < 1)
		rst = 1;

	/* Read access time >= 33ns */
	rat = ceilf (33 / cycle);//30

	/* Read hold time <= 3ns */
	rht = ceilf (3 / cycle);


/////must set the following pre-time,cannot be zero at the same time/////
	/* Pre setup time, must */
	prest = 0;

	/* Pre access time */
	preat = 0;

	/* Memory transition time */
	tt = 0;

	/* Memory idle time */
	it = 0;


	smc_b3tim = (((wst << BITP_SMC_B3TIM_WST) & BITM_SMC_B3TIM_WST)
		     | ((wht << BITP_SMC_B3TIM_WHT) & BITM_SMC_B3TIM_WHT)
		     | ((wat << BITP_SMC_B3TIM_WAT) & BITM_SMC_B3TIM_WAT)
		     | ((rst << BITP_SMC_B3TIM_RST) & BITM_SMC_B3TIM_RST)
		     | ((rht << BITP_SMC_B3TIM_RHT) & BITM_SMC_B3TIM_RHT)
		     | ((rat << BITP_SMC_B3TIM_RAT) & BITM_SMC_B3TIM_RAT));

	smc_b3etim = (((prest << BITP_SMC_B3ETIM_PREST) & BITM_SMC_B3ETIM_PREST)
		      | ((preat << BITP_SMC_B3ETIM_PREAT) & BITM_SMC_B3ETIM_PREAT))
		      | ((tt << BITP_SMC_B0ETIM_TT) & BITM_SMC_B0ETIM_TT)
		      | ((it << BITP_SMC_B0ETIM_IT) & BITM_SMC_B0ETIM_IT);

	smc_b3ctl = ((1 << BITP_SMC_B3CTL_EN)  //Enable bank3
		     | (0 << BITP_SMC_B3CTL_MODE)); //setup async  RAM mode



	*pREG_SMC0_B3TIM = smc_b3tim;
	asm(" ssync ; ");
	*pREG_SMC0_B3ETIM = smc_b3etim;
	asm(" ssync ; ");
	*pREG_SMC0_B3CTL = smc_b3ctl;
	asm(" ssync ; ");

	return 0;
}






/*
 * DM9000A INIT
 */
void Init_DM9000A(uint8_t* srcMac )
{
	/*
	 * init the smc0 bank3
	 */

	Init_MDMA_DM9000A();

	Setup_SMC_DM9000A();

	Init_IRQ_DM9000A();

	Setup_DM9000A( srcMac );

	LoopQueue_init( &g_ExEthRecvQueue );


//	ReadID();
//	ReadID();

}

int ExEthSend(void*buffer, int len)
{

	DM9000A_DMA_Send(buffer, len);

	while (false == MDMAIsReady())
	{
		;
	}

	while( false == IsSendOver() )
	{
		;
	}
	return 1;
}


void* ExEthRecv(void)
{
	int ret=0, Elem = 0;

	LoopQueueItem* pRecvItem = NULL;


	uint16_t irq = *pREG_PORTD_DATA & ADI_GPIO_PIN_9;
	if(irq)
	{
		Process_DM9000A_INT_Event();
	}

	ENTER_CRITICAL_REGION();

	pRecvItem = LoopQueue_pop( &g_ExEthRecvQueue );

	EXIT_CRITICAL_REGION();

	if(pRecvItem)
	{
		return (pRecvItem->Data );
	}

	return NULL;
}

//void DM9000A_Test(void)
//{
//	static int i = 0;
//
//	uint8_t isr = ReadReg( ISR );
//	uint8_t rsr = ReadReg( RSR );
//	uint8_t imr = ReadReg( IMR );
//	if(isr & 0x01)
//	{
//		WriteReg(ISR, 0x01);
//		DM9000A_Recv(pBuf);
//		recved = 0;
//	}
//}
