/*
 * task_ad.c
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */


#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"

#include "myapp_cfg.h"

#include "mutester_comm_protocol.h"
//#include "xl-6004_forward_protocol.h"
#include "msg.h"

#include "IEC61850_9_2.h"

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



void ADData2SmvFrm( void *pAD_Value, uint8_t AD_ByteLength, uint16_t smpCnt)
{
	ADI_ETHER_BUFFER *pNewBuffer = NULL;
	ADI_ETHER_BUFFER *pBuf = NULL;
	uint8_t* pADDataBuf = NULL;
	int nFrmLen = 0;
	uint8_t PortNo = 0;

	AD7608_DATA*pAD7608_Data = (AD7608_DATA*)pAD_Value;

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
	}

	if(!pBuf)
	{
		//ERROR
		DEBUG_PRINT("%s[#%d]:pop_queue failed...\n\n", __FILE__, __LINE__);
		return ;
	}

	uint32_t data[6];

	data[0]   = netHostChangeL(pAD7608_Data[0].ad *5000./0x1ffff);
	data[1]   = netHostChangeL(pAD7608_Data[1].ad *5000./0x1ffff);
	data[2]   = netHostChangeL(pAD7608_Data[2].ad *5000./0x1ffff);

	data[3]   = netHostChangeL(pAD7608_Data[3].ad *5000./0x1ffff);
	data[4]   = netHostChangeL(pAD7608_Data[4].ad *5000./0x1ffff);
	data[5]   = netHostChangeL(pAD7608_Data[5].ad *5000./0x1ffff);

	nFrmLen = PackSmvFrm( (uint8_t*)pBuf->Data + 2,
			data,
			smpCnt, PortNo );

//	g_TaskADPara.unSmpCnt++;

	*( unsigned short * ) pBuf->Data = nFrmLen;
	pBuf->ElementCount  = nFrmLen + 2;
	pBuf->PayLoad =  0; 	// payload is part of the packet
	pBuf->StatusWord = 0; 	// changes from 0 to the status info

	if( g_rtParams.U8Parameter[U8PARA_NET_SEND1] )
	{
		//send by eth0
		MuTesterSystem.Device.Eth0.Write ( g_hEthDev[0], pBuf);
	}
	else if( g_rtParams.U8Parameter[U8PARA_NET_SEND2]  )
	{
		//send by eth1
		MuTesterSystem.Device.Eth1.Write ( g_hEthDev[1], pBuf);
	}
	else
	{
		DEBUG_PRINT("%s[#%d]:bad U8 Parameter setting for no eth send port...\n\n", __FILE__, __LINE__);
	}

}


static void PackStandADData(uint8_t* OutBuf, void *pAD_Value, uint8_t AD_ByteLength,
		uint16_t SmpCnt)
{
	STAND_SAMP_TYPE  *pPktAD =(STAND_SAMP_TYPE*)( OutBuf );

	AD7608_DATA*pAD7608_Data = (AD7608_DATA*)pAD_Value;

	pPktAD->stateLabel		= 0;		//状态标识
 	pPktAD->sampCnt			= SmpCnt;			//采样序号
//	pPktAD->sampTMark		=;		//采样延时
//	pPktAD->netSendTMark	=;	//点对点发送延时
//	pPktAD->FT3SendTMark	=;	//FT3发送延时


	pPktAD->UaSampData   = netHostChangeL(pAD7608_Data[0].ad *5000./0x1ffff);
	pPktAD->UbSampData   = netHostChangeL(pAD7608_Data[1].ad *5000./0x1ffff);
	pPktAD->UcSampData   = netHostChangeL(pAD7608_Data[2].ad *5000./0x1ffff);

	pPktAD->IaSampData   = netHostChangeL(pAD7608_Data[3].ad *5000./0x1ffff);
	pPktAD->IbSampData   = netHostChangeL(pAD7608_Data[4].ad *5000./0x1ffff);
	pPktAD->IbSampData   = netHostChangeL(pAD7608_Data[5].ad *5000./0x1ffff);



}



void ADData2StandardADFrm( void *pAD_Value, uint8_t AD_ByteLength, uint32_t smpCnt)
{
	ADI_ETHER_BUFFER *pNewBuffer = NULL;
	ADI_ETHER_BUFFER *pBuf = NULL;
	uint8_t* pADDataBuf = NULL;

	int nFrmLen = 0;

	/* user process the AD data*/
	pBuf = MuTesterSystem.Device.exEth.PopProcessedElem( &g_ExEthXmtQueueAD, 1 );
	g_TaskADPara.pStandardADFrmSendBuf = pBuf;

	if(!pBuf)
	{
		//ERROR
		DEBUG_PRINT("%s[#%d]:pop_queue failed...\n\n", __FILE__, __LINE__);
		return ;
	}

	pADDataBuf = (uint8_t*)g_TaskADPara.pStandardADFrmSendBuf->Data + 2
			+ sizeof(MUTestMsgHeader)
			+ sizeof(STAND_SAMP_HEAD_TYPE);

	PackStandADData( pADDataBuf, pAD_Value,  AD_ByteLength, smpCnt );

//	g_TaskADPara.unSmpCnt++;

	nFrmLen = msgPackStandADFrm( (uint8_t*)g_TaskADPara.pStandardADFrmSendBuf->Data + 2, 1);

	*( unsigned short * ) g_TaskADPara.pStandardADFrmSendBuf->Data = nFrmLen;
	g_TaskADPara.pStandardADFrmSendBuf->ElementCount  = nFrmLen + 2;
	g_TaskADPara.pStandardADFrmSendBuf->PayLoad =  0; 	// payload is part of the packet
	g_TaskADPara.pStandardADFrmSendBuf->StatusWord = 0; 	// changes from 0 to the status info

	//send by exEth
	MuTesterSystem.Device.exEth.PushUnprocessElem(&g_ExEthXmtQueueAD, g_TaskADPara.pStandardADFrmSendBuf);
	g_TaskADPara.pStandardADFrmSendBuf = NULL;
}



static void AD7608_Busy_ISR(ADI_GPIO_PIN_INTERRUPT const ePinInt, const uint32_t event, void *pArg)
{
#if 1
	SubmitBuffer_SPORT1B();

	/* Enable the SPORT2-B Rx  D0 and DMA */
	Enable_SPORT1B();
#endif
}


/*********************************************************************

    Function:       SPORTCallbackRx

    Description:    This is a callback function registered with the  driver.
                    This function will be called when the RX completes the data
            transfer.

*********************************************************************/

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

        	if( (g_rtParams.U8Parameter[U8PARA_NET_SEND1]) || ( g_rtParams.U8Parameter[U8PARA_NET_SEND2] ) )
        	{
        		ADData2SmvFrm( pArg, 32, g_TaskADPara.usSmpCnt);
        	}


        	ADData2StandardADFrm( pArg, 32, g_TaskADPara.usSmpCnt);

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

	MuTesterSystem.Device.AD7608.RegisterSportCallback( SPORTCallbackRx );

	MuTesterSystem.Device.AD7608.EnableBuzyIOInterrupt( true );


//	OSTaskDel((OS_TCB*)0, &osErr);

}




