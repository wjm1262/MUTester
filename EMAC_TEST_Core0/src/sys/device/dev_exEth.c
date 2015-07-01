/*
 * dev_exEth.c
 *
 *  Created on: 2015-3-30
 *      Author: Administrator
 */

#include "sys.h"
#include "dev_exEth.h"

EXEMAC_FRAME_Q g_ExEthXmtQueueEth0;
EXEMAC_FRAME_Q g_ExEthXmtQueueEth1;
EXEMAC_FRAME_Q g_ExEthXmtQueueAD;

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

/////////////////////////////


void RegisterExEthnetModual( void )
{
	MuTesterSystem.Device.exEth.InitExEthnet		= DM9000A_Init;
	MuTesterSystem.Device.exEth.EthSend				= ExEthSend;
	MuTesterSystem.Device.exEth.EthRecv				= ExEthRecv;

//	MuTesterSystem.Device.exEth.RegisterMACIntCallback = RegisterMACIntCallback;
//	MuTesterSystem.Device.exEth.EnableMACIntInterrupt  = Enable_MAC_INT_Interrupt;


	MuTesterSystem.Device.exEth.InitExEthQueue		= init_exemac_queue;
	MuTesterSystem.Device.exEth.PushUnprocessElem 	= push_unprocessed_elem_exemac_queue;
	MuTesterSystem.Device.exEth.PopProcessedElem 	= pop_processed_elem_exemac_queue;
	MuTesterSystem.Device.exEth.PopUnprocessElem 	= get_unprocessed_elem_exemac_queue;
}
