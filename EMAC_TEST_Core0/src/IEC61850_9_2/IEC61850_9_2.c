/*
 * IEC61850_9_2.c
 *
 *  Created on: 2015-1-23
 *      Author: ChiliWang
 *
 */
#include "IEC61850_9_2.h"
#include <stdint.h>
#include <services/gpio/adi_gpio.h>
#include <drivers/ethernet/adi_ether.h>
#include <stdio.h>
#include <string.h>
#include <adi_osal.h>
#include <stdlib.h>

#include "post_debug.h"

#include "dri_ad7608.h"
#include "sys.h"
/*
 * use to calculate the system time
 */
//#include "../CGU/cgu_init.h"

//prof_t cycle;


#define  ID_OF_HEAP   2
IEC61850_9_2 *pIEC61850_9_2 = NULL;


/*
 * General Timer initialization.
 * 1, #include <services/tmr/adi_tmr.h>
 * 2, #include <services/int/adi_int.h>  when you use adi_int_InstallHandler function.
 * 3, Define the ADI_TMR_HANDLE and the Memory for the GP timer.
 * 4, #define the value of the period, the default SCK is 125M.
 * 5, Setting the pin multiplex in system.svc if you want to output the PWM.
 * 6, Define the interrupt Handler of the GP timer if used.
 */
#include <services/tmr/adi_tmr.h>
#include <services/int/adi_int.h>
#include <services/gpio/adi_gpio.h>

/* Timer pulse width, delay and period parameters */
#define TIMER6_WIDTH         (2000ul)
#define TIMER6_DELAY         (0)
#define TIMER6_PERIOD        (20000ul)
/* Timer to be used in the example */
#define GP_TIMER6_NUM        6
/* Timer handle */
ADI_TMR_HANDLE   ghTimer6;
/* Memory required for opening the timer */
uint8_t TimerMemory6[ADI_TMR_MEMORY];

/* Timer event handler */
void Timer6_ISR(void *pCBParam, uint32_t Event, void *pArg);

void GP_Timer6_Init(void)
{
	ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;
    /* Open the timer */
    if( (eTmrResult = adi_tmr_Open (
            GP_TIMER6_NUM,
            TimerMemory6,
            ADI_TMR_MEMORY,
            Timer6_ISR,
            NULL,
            &ghTimer6)) != ADI_TMR_SUCCESS)
    {
        printf("Timer open failed 0x%08X \n", eTmrResult);
    }
    // Set the mode to PWM OUT
     if((eTmrResult = adi_tmr_SetMode(
             ghTimer6,
             ADI_TMR_MODE_CONTINUOUS_PWMOUT)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to open timer in PWM out mode 0x%08X \n", eTmrResult);
     }
      //Set the IRQ mode to get interrupt after timer counts to Delay + Width
     if((eTmrResult = adi_tmr_SetIRQMode(
             ghTimer6,
             ADI_TMR_IRQMODE_PERIOD)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer IRQ mode 0x%08X \n", eTmrResult);
     }

      //Set the Period
     if((eTmrResult = adi_tmr_SetPeriod(
             ghTimer6,
             TIMER6_PERIOD)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer Period 0x%08X \n", eTmrResult);
     }

      //Set the timer width
     if((eTmrResult = adi_tmr_SetWidth(
             ghTimer6,
             TIMER6_WIDTH)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer Width 0x%08X \n", eTmrResult);
     }

     // Set the timer Delay
     if((eTmrResult = adi_tmr_SetDelay(
             ghTimer6,
             TIMER6_DELAY)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer Delay 0x%08X \n", eTmrResult);
     }

      //Enable the timer to stop gracefully,used in PWM mode,set the PWM wave more graceful.
     if((eTmrResult = adi_tmr_EnableGracefulStop(
             ghTimer6,
             true)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to enable the timer to stop gracefully 0x%08X \n", eTmrResult);
     }
     adi_tmr_Enable(ghTimer6, true);

}

void Send_9_2Frame(void)
{
//	PROFBEG(cycle);
	uint16_t index = pIEC61850_9_2->offset;
	uint16_t FrameCnt = 0;
	static bool once = 1;
	/************************************************************************
	* 更新9-2帧的采样数据,采样计数器
	*
	************************************************************************/

		for (int i = 0; i < pIEC61850_9_2->ASDU_Num; i++)
		{
			FrameCnt = myHtons(( pIEC61850_9_2->pASDU_info[0].SmpCnt + 1 + i) % 4000);
			*(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset) 	= (uint8_t)FrameCnt;
			*(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset+1) = (uint8_t)(FrameCnt>>8);

		}
		pIEC61850_9_2->pASDU_info[0].SmpCnt = myHtons(FrameCnt);
#if 1
		DM9000A_DMA_Send(pIEC61850_9_2->SendBuff + pIEC61850_9_2->offset + 1, pIEC61850_9_2->FrameLen);
#else
		DM9000A_Core_Send(pIEC61850_9_2->SendBuff + pIEC61850_9_2->offset + 1, pIEC61850_9_2->FrameLen);
#endif
}

static void Timer6_ISR(void *pCBParam, uint32_t Event, void *pArg)
{

	Send_9_2Frame();
}


#if 0


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
//#define MEMCOPY_XFER_1BYTE		/* Enable macro for 1 byte transfer */
//#define MEMCOPY_XFER_2BYTES		/* Enable macro for 2 bytes transfer */
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
#define MEMCOPY_STREAM_ID           ADI_DMA_MEMDMA_S0       /* Stream 0 */


void myMDAM_Callback (void *pCBParam, uint32_t event, void *pArg);

/*
 * MDMA callback function
 */
void myMDAM_Callback (void *pCBParam, uint32_t event, void *pArg)
{
	ADI_ETHER_BUFFER *ptr;
	switch ( event )
	{
		case ADI_DMA_EVENT_BUFFER_PROCESSED:
			memcpy(&ptr, &(((uint8_t *)pArg)[1540]), 4);
			EtherSend ( g_hEthDev[1], ptr);
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

#endif

/*
 *Index: last one available buffer location.
 */
int TLV_Pack(char tag, char *pData, UINT16 len)
{
	UINT16 index    = pIEC61850_9_2->offset;
	char  *SendBuff = pIEC61850_9_2->SendBuff;

	if (pIEC61850_9_2->offset < 0)
	{
		DEBUG_STATEMENT("ASDU和通道配置超过以太网帧长度限制！");
		return -1;
	}

	if (pData != &SendBuff[index+1])//不是buffer本身，需要copy
	{
		index -= len;
		memcpy(&SendBuff[index+1], pData, len);
	}


	/*
	* 根据ASN.1规则TLV，计算长度，设置长度信息
	* len>0x100, 第一字节的高位为1，后面7为表示后面用几个字节表示长度信息
	* len>0x100,以此类推
	*/

	if(len>=0x100)
	{
		SendBuff[index-2]= 0x82;
		SendBuff[index-1]= (uint8_t)(len>>8);
		SendBuff[index]  = (uint8_t)(len);
		index -= 3;
	}
	else if(len>=0x80)
	{
		SendBuff[index-1]=0x81;
		SendBuff[index]=(uint8_t)(len);
		index -= 2;
	}
	else
	{
		SendBuff[index] = (uint8_t)(len);
		index -= 1;
	}
	SendBuff[index] = tag;
	index          -= 1;
	pIEC61850_9_2->offset = index;
	return 0;

}
/*
 * net to 64
 */
#pragma inline
uint64_t myHtonll(uint64_t netlong)
{
	uint32_t retValue;
	*(((uint8_t*)&retValue)+0) = *(((uint8_t*)&netlong)+7);
	*(((uint8_t*)&retValue)+1) = *(((uint8_t*)&netlong)+6);
	*(((uint8_t*)&retValue)+2) = *(((uint8_t*)&netlong)+5);
	*(((uint8_t*)&retValue)+3) = *(((uint8_t*)&netlong)+4);
	*(((uint8_t*)&retValue)+4) = *(((uint8_t*)&netlong)+3);
	*(((uint8_t*)&retValue)+5) = *(((uint8_t*)&netlong)+2);
	*(((uint8_t*)&retValue)+6) = *(((uint8_t*)&netlong)+1);
	*(((uint8_t*)&retValue)+7) = *(((uint8_t*)&netlong)+0);
	return retValue;
}
/*
 * net to 32
 */
#pragma inline
uint32_t myHtonl(int32_t netlong)
{
	uint32_t retValue;
	*(((uint8_t*)&retValue)+0) = *(((uint8_t*)&netlong)+3);
	*(((uint8_t*)&retValue)+1) = *(((uint8_t*)&netlong)+2);
	*(((uint8_t*)&retValue)+2) = *(((uint8_t*)&netlong)+1);
	*(((uint8_t*)&retValue)+3) = *(((uint8_t*)&netlong)+0);
	return retValue;
}
/*
 * net to 16
 */
#pragma inline
uint16_t myHtons(uint16_t netlong)
{
	uint16_t retValue;
	*(((uint8_t*)&retValue)+0) = *(((uint8_t*)&netlong)+1);
	*(((uint8_t*)&retValue)+1) = *(((uint8_t*)&netlong)+0);
	return retValue;
}
/*
 * 打包IEC61850_9_2帧
 */
int32_t  Init_9_2_Frame(uint8_t ASDU_Num, uint16_t DataChannelNum)
{
	if(NULL == pIEC61850_9_2)
	{
		pIEC61850_9_2 = (IEC61850_9_2 *)heap_malloc(ID_OF_HEAP, sizeof(IEC61850_9_2));
		if(NULL != pIEC61850_9_2)
		{
			memset(pIEC61850_9_2, 0, sizeof(IEC61850_9_2));
		}else
		{
			DEBUG_STATEMENT("IEC61850-9-2 memory allocation failed.\n\n");
		}
	}
	char des[6] = {0x20,0x22,0x23,0x24,0x25,0x26};
	char src[6] = {0x10,0x12,0x13,0x14,0x15,0x16};
	memcpy(&pIEC61850_9_2->FrameHead.des, des, 6);
	memcpy(&pIEC61850_9_2->FrameHead.src, src, 6);
	pIEC61850_9_2->FrameHead.TPID = 0x0081;
	pIEC61850_9_2->FrameHead.TCI  = 0x0180;
	pIEC61850_9_2->FrameHead.EtheType = 0xba88;
	pIEC61850_9_2->FrameHead.APPID    = 0x0040;
	memset(pIEC61850_9_2->FrameHead.reserver, 0, 4);
	/***********************************************************************
	功能：获取配置信息
	************************************************************************/
	//Get the asdu number
	pIEC61850_9_2->offset = 1517;
	uint16_t index = 0;
	static uint8_t last_asdu_num = 0;
	int RT = 0;

	pIEC61850_9_2->ASDU_Num = ASDU_Num;

	/***********************************************************************
	功能：申请内存
	************************************************************************/
	if (pIEC61850_9_2->ASDU_Num != last_asdu_num)
	{
		if (pIEC61850_9_2->pASDU_info == NULL)
		{
			pIEC61850_9_2->pASDU_info = (ASDU_INFO *)heap_malloc(ID_OF_HEAP, sizeof(ASDU_INFO)*pIEC61850_9_2->ASDU_Num);
			if(NULL != pIEC61850_9_2->pASDU_info)
			{
				memset(pIEC61850_9_2->pASDU_info, 0, sizeof(ASDU_INFO)*pIEC61850_9_2->ASDU_Num);
			}else
			{
				DEBUG_STATEMENT("ASDU_info memory allocation failed.\n\n");
			}

		}else
		{
			free(pIEC61850_9_2->pASDU_info);
			pIEC61850_9_2->pASDU_info = (ASDU_INFO *)heap_malloc(ID_OF_HEAP, sizeof(ASDU_INFO)*pIEC61850_9_2->ASDU_Num);
			if(NULL != pIEC61850_9_2->pASDU_info)
			{
				memset(pIEC61850_9_2->pASDU_info, 0, sizeof(ASDU_INFO)*pIEC61850_9_2->ASDU_Num);
			}else
			{
				DEBUG_STATEMENT("ASDU_info memory allocation failed.\n\n");
			}

		}
		last_asdu_num = pIEC61850_9_2->ASDU_Num;
	}
	/***********************************************************************
	功能：ASDU数据打包，从里到外的打包，有点协议栈的味道
	************************************************************************/
	for (int i = pIEC61850_9_2->ASDU_Num -1 ; i >= 0 ; i--)//pack data
	{
		/*
		 * 初始化参数
		 */
		pIEC61850_9_2->pASDU_info[i].SmpSync  = 1;
		pIEC61850_9_2->pASDU_info[i].bDataSet = 0;
		pIEC61850_9_2->pASDU_info[i].bRefrTm  = 0;
		pIEC61850_9_2->pASDU_info[i].bSmpRate = 1;
		pIEC61850_9_2->pASDU_info[i].SmpRate  = 4000;
		pIEC61850_9_2->pASDU_info[i].ASDU_End = pIEC61850_9_2->offset;
		//get the channel number
		pIEC61850_9_2->pASDU_info[i].DataChannelNum = DataChannelNum;

		/*
		 * Pack channel data sequence
		 * 0x87
		 */
//		for (int m = 0; m < pIEC61850_9_2->ASDU_Num; m++)
//		{
//			for (int j = 0; j < pIEC61850_9_2->pASDU_info[i].DataChannelNum; j++)
//			{
//				pIEC61850_9_2->pASDU_info[i].ChannelData[j].value   = myHtonl(j*100);
//				pIEC61850_9_2->pASDU_info[i].ChannelData[j].quality = myHtonl(j*100);
//			}
//		}
		pIEC61850_9_2->pASDU_info[i].Data_Offset = pIEC61850_9_2->offset - pIEC61850_9_2->pASDU_info[i].DataChannelNum*8 + 1;
		RT = TLV_Pack(0x87, (char *)pIEC61850_9_2->pASDU_info[i].ChannelData, pIEC61850_9_2->pASDU_info[i].DataChannelNum*8);

		if(RT)
			return -1;
		/* check if it has SmpRate, and then pack it
		 * 0x86
		 */
		if (pIEC61850_9_2->pASDU_info[i].bSmpRate)
		{
			uint16_t SmpRate = pIEC61850_9_2->pASDU_info[i].SmpRate;
			pIEC61850_9_2->pASDU_info[i].SmpRate_Offset = pIEC61850_9_2->offset - 2 + 1;
			SmpRate = myHtons(SmpRate);
			RT = TLV_Pack(0x86, (char *)&SmpRate, 2);//T-L-V
			if(RT)
				return -1;
		}

		/*
		 * 0x85,SmpSync
		 */
		char sync = pIEC61850_9_2->pASDU_info[i].SmpSync;
		pIEC61850_9_2->pASDU_info[i].SmpSync_Offset = pIEC61850_9_2->offset - 1 + 1;
		RT = TLV_Pack(0x85, &sync, 1);//T-L-V
		if(RT)
			return -1;
		/*
		 * 0x84, RefrTm : UTC time,8Bytes = 64bits
		 */
		if (pIEC61850_9_2->pASDU_info[i].bRefrTm)
		{

			char time[8]={1,2,3,4,5,6,7,8};
			pIEC61850_9_2->pASDU_info[i].RefrTm_Offset = pIEC61850_9_2->offset - 8 + 1;
			//T-L-V
			RT = TLV_Pack(0x84, time, 8);
			if(RT)
				return -1;
		}
		/*
		 * 0x83,ConfRev
		 */
		uint32_t ConfRev =  pIEC61850_9_2->pASDU_info[i].ConfRev;
		pIEC61850_9_2->pASDU_info[i].ConfRev_Offset = pIEC61850_9_2->offset - 4 + 1;
		ConfRev = myHtonl(ConfRev);
		RT = TLV_Pack(0x83, (char *)&ConfRev, 4);
		if(RT)
			return -1;
		/*
		 * 0x82,SmpCnt
		 */
		uint16_t SmpCnt =  pIEC61850_9_2->pASDU_info[i].SmpCnt;
		pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset = pIEC61850_9_2->offset - 2 + 1;
		SmpCnt = myHtons(SmpCnt);
		RT = TLV_Pack(0x82, (char *)&SmpCnt, 2);
		if(RT)
			return -1;
		/*
		 * 0x81,DateSet
		 */
		if (pIEC61850_9_2->pASDU_info[i].bDataSet)
		{

			char datasetName[]= "Test_dataset_name";
			pIEC61850_9_2->pASDU_info[i].DataSet_Offset = pIEC61850_9_2->offset - sizeof(datasetName) + 1;
			//T-L-V
			RT = TLV_Pack(0x81, (char *)datasetName, sizeof(datasetName));
			if(RT)
				return -1;
		}
		/*
		* 0x80,SvID
		*/
		char SvID[]= "XJPA_MU0001";
		pIEC61850_9_2->pASDU_info[i].DataSet_Offset = pIEC61850_9_2->offset - sizeof(SvID) + 1;
		//T-L-V
		RT = TLV_Pack(0x80, (char *)SvID, sizeof(SvID));
		if(RT)
			return -1;

		///save the end location of the ASDU
		pIEC61850_9_2->pASDU_info[i].ASDU_Start = pIEC61850_9_2->offset + 1;
		/*
		 * 0x30,ASDU
		 */
		index = pIEC61850_9_2->offset;
		uint16_t ASDU_Len = pIEC61850_9_2->pASDU_info[i].ASDU_End - pIEC61850_9_2->pASDU_info[i].ASDU_Start + 1;
		RT = TLV_Pack(0x30, &pIEC61850_9_2->SendBuff[index+1], ASDU_Len);
		if(RT)
			return -1;
	}
	/***********************************************************************
	功能：APDU帧头的打包
	************************************************************************/
	/*
	* 0xA2, sequence of ASDU
	*/

	index = pIEC61850_9_2->offset;
	uint16_t Total_ASDU_Len = 1517 - index;
	RT = TLV_Pack(0xA2, &pIEC61850_9_2->SendBuff[index+1], Total_ASDU_Len);
	if(RT)
		return -1;
	/*
	* 0x81, Security, reserve
	*/
	if (pIEC61850_9_2->bSecurity)
	{

	}

	/*
	 * 0x80, number of ASDU
	 */
	char NoASDU = pIEC61850_9_2->ASDU_Num;
	RT=TLV_Pack(0x80, &NoASDU, 1);//T-L-V
	if(RT)
		return -1;
	/*
	 * 0x60, SMV APDU
	 */
	index = pIEC61850_9_2->offset;
	uint16_t APDU_Len = 1517 - index;
	RT = TLV_Pack(0x60, &pIEC61850_9_2->SendBuff[index+1], APDU_Len);
	if(RT)
		return -1;
	pIEC61850_9_2->FrameHead.FrameLen = myHtons(1517 - pIEC61850_9_2->offset + 8);
	/***********************************************************************
	功能：Ethernet帧头的打包
	************************************************************************/
	index = pIEC61850_9_2->offset;
	index = index - sizeof(FRAME_HEAD);
	if ((1517 - index) > 1517)
	{
		printf("ASDU和通道配置超过以太网帧长度限制！");
		return -1;
	}
	memcpy(&pIEC61850_9_2->SendBuff[index+1], &pIEC61850_9_2->FrameHead, sizeof(FRAME_HEAD));
	pIEC61850_9_2->offset  = index;
	pIEC61850_9_2->FrameLen = 1517- pIEC61850_9_2->offset;
	/*
	 *若打包帧后的buffer起始地址为奇数，必须进行数据偏移，否则后面memcpy函数无法进行拷贝奇数地址
	 */
	if((uint32_t)&(pIEC61850_9_2->SendBuff[pIEC61850_9_2->offset+1]) % 2 != 0)
	{
		/* 将buffer集体往左移动一个字节，保证起始地址为偶数*/
		pIEC61850_9_2->offset -= 1;
		for(int i = index; i <= 1516; i++)
		{
			pIEC61850_9_2->SendBuff[i] = pIEC61850_9_2->SendBuff[i+1];
		}

		/* 地址偏移后，9-2数据偏移发送变化，必须更新*/
		for (int i = pIEC61850_9_2->ASDU_Num -1 ; i >= 0 ; i--)//pack data
		{
			pIEC61850_9_2->pASDU_info[i].ASDU_End 		-= 1;
			pIEC61850_9_2->pASDU_info[i].ASDU_Start 	-= 1;
			pIEC61850_9_2->pASDU_info[i].ConfRev_Offset -= 1;
			pIEC61850_9_2->pASDU_info[i].DataSet_Offset -= 1;
			pIEC61850_9_2->pASDU_info[i].Data_Offset    -= 1;
			pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset  -= 1;
			pIEC61850_9_2->pASDU_info[i].SmpRate_Offset -= 1;
			pIEC61850_9_2->pASDU_info[i].SmpSync_Offset -= 1;
			pIEC61850_9_2->pASDU_info[i].SvID_Offset    -= 1;
		}
	}
	/*
	 * 将采样计数器置为最大，方便发送从0开始的帧
	 */
	for (int i = 0; i < pIEC61850_9_2->ASDU_Num; i++)
	{
		pIEC61850_9_2->pASDU_info[i].SmpCnt = 3999;
	}
	return 0;
}




void* Packet_9_2Frame(void *buf , void *pAD_Value, uint8_t AD_ByteLength)
{
//	PROFBEG(cycle);
	ADI_ETHER_BUFFER *des = (ADI_ETHER_BUFFER *)buf;

	uint16_t index = pIEC61850_9_2->offset;
	uint16_t FrameCnt = 0;
	int16_t *pAD16 = (int16_t *)pAD_Value;

//	ADI_ETHER_BUFFER *des = pop_queue(&bsInfo->xmt_queue);

	uint8_t  *pcData = (uint8_t  *)pAD_Value;
	AD7608_DATA AD_Value;
	int AD = 0;

	int nSeconds0, nNanoSeconds0;
	int nSeconds1, nNanoSeconds1;
	int nSeconds2, nNanoSeconds2;


   /*发送的buffer长度是： 实际需要发送的长度+2（长度信息），实际发送的仍然是用户的buffer长度 */
	des->ElementCount  = pIEC61850_9_2->FrameLen + 2;
	des->pNext         = NULL;

	/************************************************************************
	* 更新9-2帧的采样数据,采样计数器
	*
	************************************************************************/
	GetSysTime(g_hEthDev[0], &nSeconds0, &nNanoSeconds0);

	for (int i = 0; i < pIEC61850_9_2->ASDU_Num; i++)
	{
		FrameCnt = myHtons(( pIEC61850_9_2->pASDU_info[0].SmpCnt + 1 + i) % 4000);
		*(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset) 	= (uint8_t)FrameCnt;
		*(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset+1) = (uint8_t)(FrameCnt>>8);
		/*
		 * 将8通道，每通道的18位AD数据，转换为带符号的十进制数值
		 */
		if(pAD_Value != NULL)
		{
			AD_Value.ad = (int)pcData[0] << 10 | (int)pcData[1] << 2 | pcData[2] >> 6;
			int vol = AD_Value.ad*10000./0x1ffff;
			pIEC61850_9_2->pASDU_info[i].ChannelData[1].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[2] & 0x3f) << 12 | (int)pcData[3] << 4 | pcData[4] >> 4;
			pIEC61850_9_2->pASDU_info[i].ChannelData[2].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[4] & 0x0f) << 14 | (int)pcData[5] << 6 | pcData[6] >> 2;
			pIEC61850_9_2->pASDU_info[i].ChannelData[3].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[6] & 0x03) << 16 | (int)pcData[7] << 8 | pcData[8];
			pIEC61850_9_2->pASDU_info[i].ChannelData[4].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[9]) << 10 | (int)pcData[10] << 2 | pcData[11] >> 6;
			pIEC61850_9_2->pASDU_info[i].ChannelData[5].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[11] & 0x3f) << 12 | (int)pcData[12] << 4 | pcData[13] >> 4;
			pIEC61850_9_2->pASDU_info[i].ChannelData[6].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[13] & 0x0f) << 14 | (int)pcData[14] << 6 | pcData[15] >> 2;
			pIEC61850_9_2->pASDU_info[i].ChannelData[7].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			AD_Value.ad = (int)(pcData[15] & 0x03) << 16 | (int)pcData[16] << 8 | pcData[17];
			pIEC61850_9_2->pASDU_info[i].ChannelData[8].value   = myHtonl(AD_Value.ad*500000./0x1ffff);

			GetSysTime(g_hEthDev[0], &nSeconds1, &nNanoSeconds1);

			memcpy(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].Data_Offset+8, pIEC61850_9_2->pASDU_info[i].ChannelData+1, 64);

			GetSysTime(g_hEthDev[0], &nSeconds2, &nNanoSeconds2);
		}
	}

	pIEC61850_9_2->pASDU_info[0].SmpCnt = myHtons(FrameCnt);

//	memcpy((uint8_t *)des->Data+2, (pIEC61850_9_2->SendBuff + pIEC61850_9_2->offset + 1), pIEC61850_9_2->FrameLen);

//	DEBUG_PRINT("0- %d:%d,1- %d:%d,2- %d:%d\n\n", nSeconds0, nNanoSeconds0,nSeconds1, nNanoSeconds1, nSeconds2, nNanoSeconds2);

	return (void*)des;

}

void* Packet_9_2Frame2( void *pAD_Value, uint8_t AD_ByteLength)
{
	uint16_t index = pIEC61850_9_2->offset;
	uint16_t FrameCnt = 0;


	AD7608_DATA*pAD7608_Data = (AD7608_DATA*)pAD_Value;

	int AD = 0;

	int nSeconds0, nNanoSeconds0;
	int nSeconds1, nNanoSeconds1;
	int nSeconds2, nNanoSeconds2;

	/************************************************************************
	* 更新9-2帧的采样数据,采样计数器
	*
	************************************************************************/
	GetSysTime(g_hEthDev[0], &nSeconds0, &nNanoSeconds0);

	for (int i = 0; i < pIEC61850_9_2->ASDU_Num; i++)
	{
		FrameCnt = myHtons(( pIEC61850_9_2->pASDU_info[0].SmpCnt + 1 + i) % 4000);
		*(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset) 	= (uint8_t)FrameCnt;
		*(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].SmpCnt_Offset+1) = (uint8_t)(FrameCnt>>8);
		/*
		 * 将8通道，每通道的18位AD数据，转换为带符号的十进制数值
		 */
		if(pAD_Value != NULL)
		{
			pIEC61850_9_2->pASDU_info[i].ChannelData[1].value   = myHtonl(pAD7608_Data[0].ad *5000./0x1ffff + 0.5);
			pIEC61850_9_2->pASDU_info[i].ChannelData[5].value   = myHtonl(pAD7608_Data[1].ad *5000./0x1ffff);
			pIEC61850_9_2->pASDU_info[i].ChannelData[2].value   = myHtonl(pAD7608_Data[2].ad *5000./0x1ffff + 0.5);
			pIEC61850_9_2->pASDU_info[i].ChannelData[6].value   = myHtonl(pAD7608_Data[3].ad *5000./0x1ffff);
			pIEC61850_9_2->pASDU_info[i].ChannelData[3].value   = myHtonl(pAD7608_Data[4].ad *5000./0x1ffff);
			pIEC61850_9_2->pASDU_info[i].ChannelData[7].value   = myHtonl(pAD7608_Data[5].ad *5000./0x1ffff);;
			pIEC61850_9_2->pASDU_info[i].ChannelData[4].value   = myHtonl(pAD7608_Data[6].ad *5000./0x1ffff);
			pIEC61850_9_2->pASDU_info[i].ChannelData[8].value   = myHtonl(pAD7608_Data[7].ad *5000./0x1ffff);



			memcpy(pIEC61850_9_2->SendBuff+pIEC61850_9_2->pASDU_info[i].Data_Offset+8, pIEC61850_9_2->pASDU_info[i].ChannelData+1, 64);


		}
	}

	pIEC61850_9_2->pASDU_info[0].SmpCnt = myHtons(FrameCnt);

//	memcpy((uint8_t *)des->Data+2, (pIEC61850_9_2->SendBuff + pIEC61850_9_2->offset + 1), pIEC61850_9_2->FrameLen);

//	DEBUG_PRINT("0- %d:%d,1- %d:%d,2- %d:%d\n\n", nSeconds0, nNanoSeconds0,nSeconds1, nNanoSeconds1, nSeconds2, nNanoSeconds2);

	return (void*)pIEC61850_9_2;

}



/*
 * 配置9-2发送的基本信息，包括DMA初始化，9-2帧框架
 */
void Init_IEC_9_2(void)
{
	/* 搭建9-2帧框架 */
	Init_9_2_Frame(1, 9);

	/* 初始化9-2帧的MDMA外设*/
	//MDMA_Init();

	/* 初始化内核定时器，用于定时250us发送一次 */
//	coreTimerInit();
//	GP_Timer6_Init();
//	Init_CoreTimer();


}

