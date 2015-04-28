/*
 * dev_exEth.c
 *
 *  Created on: 2015-3-30
 *      Author: Administrator
 */

#include "sys.h"
#include "dev_exEth.h"

EXEMAC_FRAME_Q g_ExEthXmtQueue;

section ("sdram_bank0") EXETH_RECV_QUEUE g_ExEthRecvQueue;
DM9000A_INT_EVENT_QUEUE g_Dm9000aIntEventQueue;

/********************/
static void clear_exemac_queue ( EXEMAC_FRAME_Q *pQueue )
{
	pQueue->pQueueHead = NULL;
	pQueue->pQueueProcessed = NULL;
	pQueue->pQueueTail = NULL;

	pQueue->ElementCount = 0;
	pQueue->ProcessedElementCount = 0;
}


static int init_exemac_queue ( EXEMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pBuffer )
{

	int32_t NumInputBuffers = 0;
	ADI_ETHER_BUFFER *pTempBuffer = pBuffer, *pLastBuffer = NULL;

	if ( !pQueue || !pBuffer )
	{
		return 0;
	}

	clear_exemac_queue ( pQueue );

	/* typically the number of incoming buffers are small */
	do
	{
		NumInputBuffers++;
		pLastBuffer = pTempBuffer;
		pTempBuffer = pTempBuffer->pNext;

	}while ( pTempBuffer != NULL );

	ENTER_CRITICAL_REGION();

	/* Now insert and update the queue */
	pQueue->pQueueHead = pBuffer;

	pQueue->pQueueTail = pLastBuffer;

	pQueue->pQueueProcessed = pLastBuffer;

	pQueue->ProcessedElementCount = pQueue->ElementCount = NumInputBuffers;

	EXIT_CRITICAL_REGION();

	return NumInputBuffers;
}


static int push_unprocessed_elem_exemac_queue ( EXEMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pBuffer )
{

	int32_t NumInputBuffers = 0;
	ADI_ETHER_BUFFER *pTempBuffer = pBuffer, *pLastBuffer = NULL;

	if ( !pQueue || !pBuffer )
	{
		return 0;
	}

	/* typically the number of incoming buffers are small */
	do
	{
		NumInputBuffers++;
		pLastBuffer = pTempBuffer;
		pTempBuffer = pTempBuffer->pNext;

	}while ( pTempBuffer != NULL );

	ENTER_CRITICAL_REGION();

	/* Now insert and update the queue */
	if ( ( pQueue->pQueueHead == NULL ) && ( pQueue->pQueueTail == NULL ) )
	{
		pQueue->pQueueHead = pBuffer;
		pQueue->pQueueProcessed = pBuffer;
		pQueue->ProcessedElementCount = 0;
	}

	else
	{
		pQueue->pQueueTail->pNext = pBuffer;
	}

	pQueue->pQueueTail    = pLastBuffer;
	pQueue->ElementCount += NumInputBuffers;

	EXIT_CRITICAL_REGION();

	return NumInputBuffers;
}



static ADI_ETHER_BUFFER *pop_processed_elem_exemac_queue ( EXEMAC_FRAME_Q *pQueue, int n)
{
	ADI_ETHER_BUFFER *pOutHead  = NULL;
	ADI_ETHER_BUFFER *pOutTail  = NULL;
	ADI_ETHER_BUFFER *pOutTmp  = NULL;

	ENTER_CRITICAL_REGION();

	pOutTmp = pOutTail = pOutHead = pQueue->pQueueHead;

	if ( !pOutHead )
	{
		EXIT_CRITICAL_REGION();

		return NULL;
	}

	while( n > 0 && pQueue->ProcessedElementCount > 0 )
	{
		pOutTail = pOutTmp;
		pOutTmp  = pOutTmp->pNext;
		pQueue->ElementCount--;
		pQueue->ProcessedElementCount--;
		n--;
	}

	if( pQueue->ProcessedElementCount > 0 )
	{
		pQueue->pQueueHead = pOutTmp;
		pOutTail->pNext = NULL;
	}
	else if( n > 0 )
	{
//		pQueue->pQueueTail = NULL;
		pQueue->pQueueHead  = pOutTmp;
		pQueue->pQueueProcessed = pQueue->pQueueHead;

		pOutTail->pNext 	= NULL;
	}
	else
	{
		pQueue->pQueueHead  = pOutTmp;
		pQueue->pQueueTail = NULL;
		pQueue->pQueueProcessed = NULL;

		pOutTail->pNext 	= NULL;
	}

	EXIT_CRITICAL_REGION();

	return pOutHead;
}


static ADI_ETHER_BUFFER *get_unprocessed_elem_exemac_queue ( EXEMAC_FRAME_Q *pQueue )
{
	ADI_ETHER_BUFFER *p = NULL;

	ENTER_CRITICAL_REGION();


	p = pQueue->pQueueProcessed;

	if ( !p )
	{
		EXIT_CRITICAL_REGION();

		return NULL;
	}

	if ( pQueue->pQueueProcessed  == pQueue->pQueueTail  )
	{
		p = NULL;
	}

	else
	{
		pQueue->pQueueProcessed = p->pNext;
		pQueue->ProcessedElementCount++;
	}

	EXIT_CRITICAL_REGION();

	return p;
}

static void RegisterMACIntCallback( DM9000A_ISR_Handler handler )
{
	Register_MAC_INT_Callback( handler );
}

static void EnableMACIntInterrupt( bool enable )
{
	Enable_MAC_INT_Interrupt( enable );
}

static int EthSend(void*buffer, int len)
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

static void Process_DM9000A_INT_Event( void )
{
	QueueItem* pItem = NULL;

	uint8_t RxReady = 0;

	uint16_t status = ReadReg( ISR );
	uint16_t temp;
	uint8_t RxStatusRegister;
	if( status & 0x01 )//接收包中断
	{

//		temp=ReadReg(MRRH); // 读取这两个寄存器
//		temp=ReadReg(MRRL);
		/* 读取Rx Ready，不偏移内存指针 */
		ReadReg( MRCMDX );
		RxReady = ReadReg( MRCMDX );

		while(RxReady & 0x01 == 1)
		{
			RxStatusRegister = ReadReg(0x06);

			ENTER_CRITICAL_REGION();

			pItem =  Queue_pop_unused_buf( &g_ExEthRecvQueue );
			if( pItem != NULL )
			{
				pItem->Size = RxStatusRegister;

				MDMA2_Param = (void*)pItem->Data;
				Process_DM9000A_Recv( pItem->Data, RxReady);
			}

			EXIT_CRITICAL_REGION();

//			temp=ReadReg(MRRH); // 读取这两个寄存器
//			temp=ReadReg(MRRL);
			/* 读取Rx Ready，不偏移内存指针 */
			ReadReg( MRCMDX );
			RxReady = ReadReg( MRCMDX );


		}

		WriteReg( ISR, 0x01 );//清除中断
//		WriteReg( ISR, 0x01 );//清除中断

	}

	if(status & 0x02)//发送包中断
	{
		WriteReg( ISR, 0x02 );//清除中断
	}
}

static void* EthRecv(void)
{
	int ret=0, Elem = 0;

	QueueItem* pRecvItem = NULL;

	ENTER_CRITICAL_REGION();

	ret = De_DM9000A_INT_EVENT_Queue(&g_Dm9000aIntEventQueue, &Elem);

	EXIT_CRITICAL_REGION();

	if(ret == 1 && Elem == 1)
	{
		Process_DM9000A_INT_Event();
	}

	ENTER_CRITICAL_REGION();
	pRecvItem = Queue_pop_unprocesed_buf( &g_ExEthRecvQueue );
	EXIT_CRITICAL_REGION();

	if(pRecvItem)
	{
		return (pRecvItem->Data );
	}

	return NULL;
}

void RegisterExEthnetModual( void )
{
	MuTesterSystem.Device.exEth.InitExEthnet		= Init_DM9000A;
	MuTesterSystem.Device.exEth.EthSend				= EthSend;
	MuTesterSystem.Device.exEth.EthRecv				= EthRecv;

	MuTesterSystem.Device.exEth.RegisterMACIntCallback = RegisterMACIntCallback;
	MuTesterSystem.Device.exEth.EnableMACIntInterrupt  = EnableMACIntInterrupt;


	MuTesterSystem.Device.exEth.InitExEthQueue		= init_exemac_queue;
	MuTesterSystem.Device.exEth.PushUnprocessElem 	= push_unprocessed_elem_exemac_queue;
	MuTesterSystem.Device.exEth.PopProcessedElem 	= pop_processed_elem_exemac_queue;
	MuTesterSystem.Device.exEth.PopUnprocessElem 	= get_unprocessed_elem_exemac_queue;
}
