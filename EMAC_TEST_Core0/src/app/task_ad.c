/*
 * task_ad.c
 *
 *  Created on: 2015-3-17
 *      Author: Wu JM
 */


#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"

#include "myapp_cfg.h"

#include "comm_pc_protocol.h"
//#include "xl-6004_forward_protocol.h"
#include "msg.h"

#include "IEC61850_9_2.h"

//FT3
#include "FT3_Frame_Definition_Test.h"
#include "FT3_Test.h"
#include "post_debug.h"

TASK_AD_PARA g_TaskADPara ={NULL, 0};

#define SPI0_RX_SIZE  18
static uint8_t SPI0RxBuffer[SPI0_RX_SIZE] =  {0x00u};

/* transceiver configurations */
ADI_SPI_TRANSCEIVER spi_Tx_Rx_buffer  = {NULL, 0u, NULL, 0u, &SPI0RxBuffer[0], SPI0_RX_SIZE};

/*=============  D A T A  =============*/
/* DMA Manager includes */
#include <services/dma/adi_dma.h>

/* DMA Stream Handle */
ADI_DMA_STREAM_HANDLE   hMemDmaStream;
/* Source DMA Handle */
ADI_DMA_CHANNEL_HANDLE  hSrcDmaChannel;
/* Destination DMA Handle */
ADI_DMA_CHANNEL_HANDLE  hDestDmaChannel;
/* Memory to handle DMA Stream */
static uint8_t MemDmaStreamMem[ADI_DMA_STREAM_REQ_MEMORY];
/*
 * Word/Memory transfer size to use for this example
 * Enable only one word transfer size
 */
//#define MEMCOPY_XFER_1BYTE	 /* Enable macro for 1 byte transfer */
//#define MEMCOPY_XFER_2BYTES	 /* Enable macro for 2 bytes transfer */
//#define MEMCOPY_XFER_4BYTES	 /* Enable macro for 4 bytes transfer */

/* Word/Memory transfer size in bytes */
#if defined (MEMCOPY_XFER_4BYTES)	/* 4 bytes transfer */
#define MEMCOPY_MSIZE               ADI_DMA_MSIZE_4BYTES
#define MEMCOPY_MSIZE_IN_BYTES      (4u)
#elif defined (MEMCOPY_XFER_2BYTES)	/* 2 bytes transfer */
#define MEMCOPY_MSIZE               ADI_DMA_MSIZE_2BYTES
#define MEMCOPY_MSIZE_IN_BYTES      (2u)
#else	 /* 1 byte transfer */
#define MEMCOPY_MSIZE               ADI_DMA_MSIZE_1BYTE
#define MEMCOPY_MSIZE_IN_BYTES      (1u)
#endif
/*
 * MDMA Stream ID to use for this example
 */
#define MEMCOPY_STREAM_ID           ADI_DMA_MEMDMA_S0       /* Stream 0 */

/*
 * MDMA callback function
 */
void myMDAM_Callback (void *pCBParam, uint32_t event, void *pArg)
{
	ADI_ETHER_BUFFER *ptr = g_TaskADPara.pStandardADFrmSendBuf;
	ADI_ETHER_BUFFER * pNewBuffer = NULL;
	switch ( event )
	{
		case ADI_DMA_EVENT_BUFFER_PROCESSED:

			//send by eth0
//			MuTesterSystem.Device.Eth0.Write( g_hEthDev[0], ptr);

			//send by exEth
			MuTesterSystem.Device.exEth.PushUnprocessElem(&g_ExEthXmtQueueAD, ptr);


			break;
		default:
			break;
	}
}
/*
 * memcpy DMA init
 */
uint32_t MDMA_Init(void)
{
	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;
    /* IF (Success) */
    if (eResult == ADI_DMA_SUCCESS)
    {
        /* Open a Memory DMA Stream */
        eResult = adi_mdma_Open (MEMCOPY_STREAM_ID,
                                 &MemDmaStreamMem[0],
                                 &hMemDmaStream,
                                 &hSrcDmaChannel,
                                 &hDestDmaChannel,
                                 myMDAM_Callback,
                                 NULL);

        /* IF (Failure) */
        if (eResult != ADI_DMA_SUCCESS)
        {
        	printf("Failed to open MDMA stream", eResult);
        }
    }

    return 0;
}

///////////////////
static int ADData2StandardSmpData(STAND_SAMP_TYPE* OutStandardSmpData,
		void *pAD_Value,
		uint8_t AD_ByteLength,
		uint16_t SmpCnt)
{

	STAND_SAMP_TYPE  *pPktAD = OutStandardSmpData;

	AD7608_DATA*pAD7608_Data = (AD7608_DATA*)pAD_Value;

	if( !pPktAD || !pAD7608_Data)
	{
		return 0;
	}

	pPktAD->stateLabel		= 0;		//状态标识
 	pPktAD->sampCnt			= SmpCnt;			//采样序号
//	pPktAD->sampTMark		=;		//采样延时
//	pPktAD->netSendTMark	=;	//点对点发送延时
//	pPktAD->FT3SendTMark	=;	//FT3发送延时


	pPktAD->UaSampData   = (pAD7608_Data[0].ad *5000./0x1ffff);
	pPktAD->UbSampData   = (pAD7608_Data[1].ad *5000./0x1ffff);
	pPktAD->UcSampData   = (pAD7608_Data[2].ad *5000./0x1ffff);

	pPktAD->IaSampData   = (pAD7608_Data[3].ad *5000./0x1ffff);
	pPktAD->IbSampData   = (pAD7608_Data[4].ad *5000./0x1ffff);
	pPktAD->IbSampData   = (pAD7608_Data[5].ad *5000./0x1ffff);

	return 1;
}

int OutputStandardADFrm( const STAND_SAMP_TYPE*  pStandSmpData)
{
	ADI_ETHER_BUFFER *pNewBuffer = NULL;
	ADI_ETHER_BUFFER *pBuf = NULL;
	uint8_t* pADDataBuf = NULL;

	int nFrmLen = 0;


	/* user process the AD data*/
	pBuf = MuTesterSystem.Device.exEth.PopProcessedElem( &g_ExEthXmtQueueAD, 1 );
	if(!pBuf)
	{
		DEBUG_PRINT("%s[#%d]:exEth.PopProcessedElem failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	pADDataBuf = (uint8_t*)pBuf->Data + 2
					+ sizeof(MUTestMsgHeader)
					+ sizeof(STAND_SAMP_HEAD_TYPE);


	//注意：在这里，StandSmpData填充到FRM里面没有大小端转换
	memcpy(pADDataBuf, pStandSmpData, sizeof(STAND_SAMP_TYPE));
	nFrmLen = msgPackStandADFrm( (uint8_t*)pBuf->Data + 2, 1);

	*( unsigned short * ) pBuf->Data = nFrmLen;
	pBuf->ElementCount  = nFrmLen + 2;
	pBuf->PayLoad =  0; 	// payload is part of the packet
	pBuf->StatusWord = 0; 	// changes from 0 to the status info

	//send by exEth
	MuTesterSystem.Device.exEth.PushUnprocessElem(&g_ExEthXmtQueueAD, pBuf);

	return 1;
}

int OutputStandardSmvFrm(const STAND_SAMP_TYPE*  pStandSmpData )
{
	ADI_ETHER_BUFFER *pBuf = NULL;
	int nFrmLen = 0;
	uint8_t PortNo = 0;


	/* user process the AD data*/
	if( g_rtParams.U8Parameter[U8PARA_NET_SEND1] )
	{
		//send by eth0
		pBuf = pop_queue( &user_net_config_info[0].xmt_buffers_queue );
		PortNo = 1;
	}
	else if( g_rtParams.U8Parameter[U8PARA_NET_SEND2]  )
	{
		//send by eth1
		pBuf = pop_queue( &user_net_config_info[1].xmt_buffers_queue );
		PortNo = 2;
	}
	else
	{
		DEBUG_PRINT("%s[#%d]:bad U8 Parameter setting for no eth send port...\n\n", __FILE__, __LINE__);
		return 0;
	}

	if(!pBuf)
	{
		//ERROR
		DEBUG_PRINT("%s[#%d]:pop_queue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	nFrmLen = PackSmvFrm( (uint8_t*)pBuf->Data + 2,
								pStandSmpData,
								PortNo );

	*( unsigned short * ) pBuf->Data = nFrmLen;
	pBuf->ElementCount  = nFrmLen + 2;
	pBuf->PayLoad =  0; 	// payload is part of the packet
	pBuf->StatusWord = 0; 	// changes from 0 to the status info

	if(1 == PortNo )
	{
		//send by eth0
		MuTesterSystem.Device.Eth0.Write ( g_hEthDev[0], pBuf);
	}
	else if(2 == PortNo )
	{
		//send by eth1
		MuTesterSystem.Device.Eth1.Write ( g_hEthDev[1], pBuf);
	}
	else
	{
		DEBUG_PRINT("%s[#%d]:bad U8 Parameter setting for no eth send port...\n\n", __FILE__, __LINE__);

		return 0;
	}

	return 1;

}


int CreateFT3Frm( const STAND_SAMP_TYPE*  pStandSmpData)
{
	//get buffer
	Ft3FrmItem* pFrmItem = PushFt3FrmQueue(&g_Ft3FrmQueue );
	if(!pFrmItem)
	{
		DEBUG_PRINT("%s[#%d]:PushFt3FrmQueue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	pFrmItem->FrmLen = PackFT3Frm(pFrmItem->Ft3FrmData, pStandSmpData);


	return 1;
}

int OutputFT3Frm(void)
{
	Ft3FrmItem* pFrmItem = PopFt3FrmQueue(&g_Ft3FrmQueue );
	if(!pFrmItem)
	{
		DEBUG_PRINT("%s[#%d]:PopFt3FrmQueue failed...\n\n", __FILE__, __LINE__);
		return 0;
	}

	/* set the FPGA trigger pin in low */
	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

	Send_FT3_Data(pFrmItem->Ft3FrmData, pFrmItem->FrmLen );

	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

	return 1;
}

int ProcessStandardSmpDataOutput()
{
	STAND_SAMP_TYPE* pStandSmpData = PopQueue( &g_StandardSmpDataQueue );


	//9-2组帧发送
	if( (g_rtParams.U8Parameter[U8PARA_NET_SEND1])
			|| ( g_rtParams.U8Parameter[U8PARA_NET_SEND2] ) )
	{
		OutputStandardSmvFrm(pStandSmpData );
	}

	//FT3组帧发送
	if( (g_rtParams.U8Parameter[U8PARA_FT3_SEND1])
	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND2] )
	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND3] )
	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND4] )
	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND5] )
	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND6] )
	       )
	{
		CreateFT3Frm( pStandSmpData);
		OutputFT3Frm();
	}

	return 1;
}


/*
 * net to 16
 */
static inline uint16_t myHtons(uint16_t netShort)
{
	uint16_t retValue;
	*(((uint8_t*)&retValue)+0) = (uint8_t)(netShort >> 8);
	*(((uint8_t*)&retValue)+1) = (uint8_t)(netShort);
	return retValue;
}

static void ADData2Ft3(uint8_t* OutBuf, void *pAD_Value, uint8_t AD_ByteLength,
		uint16_t SmpCnt)
{

	uint16_t crc;

	FT3_TEST_DATA *pBF609_FPGA_FT3 = (FT3_TEST_DATA*)OutBuf;

	/* set the FPGA trigger pin in low */
	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

#if 1
//	SmpCnt = (SmpCnt + 1) % 4000;
	uint16_t tmp_cnt = myHtons(SmpCnt);
	/*
	 *
	 *  single phase FT3 data
	 *
	 */
	pBF609_FPGA_FT3->SinglePhaseData[0].SmpCnt    = tmp_cnt;
	pBF609_FPGA_FT3->SinglePhaseData[0].data1[0] = 1;
	pBF609_FPGA_FT3->SinglePhaseData[0].data1[1] = 1;

	/* cal 16 bytes crc*/
	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData[0].data1), 16);
	pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = 1111;

	/* cal 4 bytes crc */
	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->SinglePhaseData[0].Status), 4);
	pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = 1111;


	pBF609_FPGA_FT3->SinglePhaseData[1].SmpCnt    = tmp_cnt;
	pBF609_FPGA_FT3->SinglePhaseData[1].data1[0] = 1;
	pBF609_FPGA_FT3->SinglePhaseData[1].data1[1] = 2;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData[1].data1), 16);
//	pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = myHtons(crc);
	pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = 1111;

	/* cal 4 bytes crc */
//	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->SinglePhaseData[1].Status), 4);
//	pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = myHtons(crc);
	pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = 1111;

	pBF609_FPGA_FT3->SinglePhaseData[2].SmpCnt    = tmp_cnt;
	pBF609_FPGA_FT3->SinglePhaseData[2].data1[0] = 1;
	pBF609_FPGA_FT3->SinglePhaseData[2].data1[1] = 3;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData[2].data1), 16);
//	pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = myHtons(crc);
	pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = 1111;

	/* cal 4 bytes crc */
//	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->SinglePhaseData[2].Status), 4);
//	pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = myHtons(crc);
	pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = 1111;

	/*
	 *
	 *  Three phase FT3 data
	 *
	 */
	pBF609_FPGA_FT3->ThreePhaseData[0].SmpCnt     = tmp_cnt;
	pBF609_FPGA_FT3->ThreePhaseData[0].data1[0]  = 4;
	pBF609_FPGA_FT3->ThreePhaseData[0].data1[1]  = 4;

	/* cal 16 bytes crc*/
	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[0].data1), 16);
	pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = 1111;

	/* cal 16 bytes crc */
	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[0].data2), 16);
	pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = 1111;

	/* cal 8 bytes crc */
	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
	pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = 1111;


	pBF609_FPGA_FT3->ThreePhaseData[1].SmpCnt     = tmp_cnt;
	pBF609_FPGA_FT3->ThreePhaseData[1].data1[0]  = 4;
	pBF609_FPGA_FT3->ThreePhaseData[1].data1[1]  = 5;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[1].data1), 16);
//	pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = myHtons(crc);
	pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = 1111;

	/* cal 16 bytes crc */
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[1].data2), 16);
//	pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = myHtons(crc);
	pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = 1111;

	/* cal 8 bytes crc */
//	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseData[1].CphaseData), 8);
//	pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = myHtons(crc);
	pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = 1111;


	pBF609_FPGA_FT3->ThreePhaseData[2].SmpCnt     = tmp_cnt;
	pBF609_FPGA_FT3->ThreePhaseData[2].data1[0]  = 4;
	pBF609_FPGA_FT3->ThreePhaseData[2].data1[1]  = 6;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[2].data1), 16);
//	pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = myHtons(crc);
	pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = 1111;

	/* cal 16 bytes crc */
//	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[2].data2), 16);
//	pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = myHtons(crc);
	pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = 1111;

	/* cal 8 bytes crc */
//	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
//	pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = myHtons(crc);
	pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = 1111;

	Send_FT3_Data(pBF609_FPGA_FT3, sizeof(FT3_TEST_DATA));
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#else   /* 12.8k sample rate */
	SmpCnt = (SmpCnt + 1) % 12800;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[15] = SmpCnt;


	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC1      = crc;


	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data2, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC2      = crc;

	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data3, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC3      = crc;

	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data4, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC4      = crc;
	Send_FT3_Data(pBF609_FPGA_FT3_ex, sizeof(EX_FT3_TEST_DATA));
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#endif

}

static void AD7608_Busy_ISR(ADI_GPIO_PIN_INTERRUPT const ePinInt, const uint32_t event, void *pArg)
{
#if 1
	SubmitBuffer_SPORT1B();

	/* Enable the SPORT2-B Rx  D0 and DMA */
	Enable_SPORT1B();
#endif
}


static void SPORTCallbackRx(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg)
{

    ADI_SPORT_RESULT eResult;
    static int i = 0;
    AD7608_DATA *pAD7608_Data = pArg;

    ADI_ETHER_BUFFER *des = NULL;
    IEC61850_9_2 *pFrmData   = NULL;
   /*
	 * Disable the SPORT1-B, stop the SPORT process
	 */
    Disable_SPORT1B();


	/*
	 * Avoiding data offset, we should cancel the SPORT1-B-CLK, set clk in high default.
	 */

    CancelClk_SPORT1B();


    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case (uint32_t)ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:

          		/* user process the AD data*/
				des = pop_queue( &user_net_config_info[0].xmt_buffers_queue );
        		g_TaskADPara.pStandardADFrmSendBuf = des;

        		des = MuTesterSystem.Device.exEth.PopProcessedElem( &g_ExEthXmtQueueAD, 1 );
        		g_TaskADPara.pStandardADFrmSendBuf = des;

				pFrmData = (IEC61850_9_2 *)Packet_9_2Frame2( pArg, 2);

				des->ElementCount  = pFrmData->FrameLen + 2;
				des->pNext         = NULL;

				adi_mdma_Copy1D(hMemDmaStream,
									(uint8_t *)des->Data+2,
									(pFrmData->SendBuff + pFrmData->offset + 1),
									ADI_DMA_MSIZE_1BYTE,
									pFrmData->FrameLen);

            break;
        default:
             break;
    }
    /* return */
}

static void SPORTCallbackRx2(
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg)
{

    ADI_SPORT_RESULT eResult;

    AD7608_DATA *pAD7608_Data = pArg;

    STAND_SAMP_TYPE* pStandardSmpData, *pOutStandardSmpData;
   /*
	 * Disable the SPORT1-B, stop the SPORT process
	 */
    Disable_SPORT1B();


	/*
	 * Avoiding data offset, we should cancel the SPORT1-B-CLK, set clk in high default.
	 */

    CancelClk_SPORT1B();


    /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case (uint32_t)ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
        	/* user process the AD data*/

        	pStandardSmpData = PushQueue( &g_StandardSmpDataQueue );
        	if(!pStandardSmpData)
        	{
        		DEBUG_STATEMENT("in SPORTCallbackRx2: PushQueue failed!\n\n");
        		return;
        	}

        	ADData2StandardSmpData(pStandardSmpData,
									pArg,
									32,
									g_TaskADPara.usSmpCnt);

        	//
        	pOutStandardSmpData = PopQueue( &g_StandardSmpDataQueue );

        	//标准数据组帧发送
        	OutputStandardADFrm( pOutStandardSmpData );

        	//9-2组帧发送
        	if( (g_rtParams.U8Parameter[U8PARA_NET_SEND1])
        			|| ( g_rtParams.U8Parameter[U8PARA_NET_SEND2] ) )
			{
        		OutputStandardSmvFrm(pOutStandardSmpData);
			}

        	//FT3组帧发送
        	if( (g_rtParams.U8Parameter[U8PARA_FT3_SEND1])
        	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND2] )
        	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND3] )
        	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND4] )
        	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND5] )
        	       || ( g_rtParams.U8Parameter[U8PARA_FT3_SEND6] )
        	       )
			{
        		ADData2Ft3( (uint8_t*)g_pBF609_FPGA_FT3, pArg, 32, g_TaskADPara.usSmpCnt);
			}


        	g_TaskADPara.usSmpCnt = (g_TaskADPara.usSmpCnt + 1)%4000;

            break;
        default:
            break;
    }
    /* return */
}



///////////////////////////////////////////

void Task_AD7608( void* p_arg )
{
//	OS_ERR osErr;

	g_TaskADPara.pStandardADFrmSendBuf = NULL;

	g_TaskADPara.usSmpCnt = 0;

	MDMA_Init();

	MuTesterSystem.Device.AD7608.InitADDevice();

	MuTesterSystem.Device.AD7608.RegisterBuzyIOCallback( AD7608_Busy_ISR );

//	MuTesterSystem.Device.AD7608.RegisterSPI0Callback( SPI0_Callback );

	MuTesterSystem.Device.AD7608.RegisterSportCallback( SPORTCallbackRx2 );

	MuTesterSystem.Device.AD7608.EnableBuzyIOInterrupt( true );


//	OSTaskDel((OS_TCB*)0, &osErr);

}




