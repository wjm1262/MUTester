/*
 * myapp_cfg.c
 *
 *  Created on: 2015-2-9
 *      Author: Administrator
 */


#include "myapp_cfg.h"

#include "post_debug.h"



/* bf51x_hw reference v1.2 P22-18
 * The length of each RX DMA buffer must be at least 1556 (0x614) bytes.
 * This is the maximum number of bytes that the MAC can deliver by DMA
 * on any receive frame. Frames longer than the 1556-byte hardware limit
 * are truncated by the MAC.The 1556-byte hardware limit accommodates the
 * longest legal Ethernet frames (1518 bytes for untagged frames, or
 * 1522 bytes for tagged 802.1Q frames) plus a small margin to accommodate
 * future standards extensions.
 * The MAC does not support RX DMA data buffers composed of more
 * than one descriptor.
 * */

/* bf51x_hw reference v1.2 P22-25
 * End of frame – At the end of the frame, the MAC issues a finish command to the DMA controller,
 * causing it to advance to the next (status) descriptor.
 * If the TX frame exceeds the maximum length limit (1560 bytes, or 0x618),
 * the frame’s DMA transfer is truncated.
 * Only 1543 (0x607) are transmitted on the MII.
 * */
// 1600,1548

/*! size of the memory block to allocate to the stack.  */
const unsigned g_ctEthHeapSize[3]={1024*1024*12, 1024*1024*12,1024*1024*14};


//0:eth0; 1:eth1; 2:exEth;
ETH_CFG_INFO user_net_config_info[3] =
{
	//eth0
	{
		0,
		8000,
		6000,
		648,
		648,
		0,
		0,
		0,
		0,
		{0x04u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u},
		0,
		0,
		{NULL, NULL, 0},
		0,
		0,
		{NULL, NULL, 0},
	},
	//eth1
	{
		0,
		8000,
		6000,
		648,
		648,
		0,
		0,
		0,
		0,
		{0x08u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u},
		0,
		0,
		{NULL, NULL, 0},
		0,
		0,
		{NULL, NULL, 0},
	},

	//exEth
	{
		0,
		0,//rx
		14000,//tx
		648,
		648,
		0,
		0,
		0,
		0,
		{0x00u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u},
		0,
		0,
		{NULL, NULL, 0},
		0,
		0,
		{NULL, NULL, 0},
	},

};


static int InitFrameBuff ( ETH_CFG_INFO *bsInfo )
{
	ADI_ETHER_BUFFER *p;

	int rx_len, tx_len, count;
	int i;
	ADI_ETHER_BUFFER *rx_head = NULL;
	ADI_ETHER_BUFFER *tx_head = NULL;

	char *buffer;
	int bufSize;

	if ( !bsInfo->buff_area || 0 >= bsInfo->buff_area_size )
		return -1;

	clear_queue ( &bsInfo->xmt_buffers_queue );
	clear_queue ( &bsInfo->rx_completed_queue );


	// calculate total requirement for each rx and tx buffer

	rx_len = bsInfo->rx_len_align;
	tx_len = bsInfo->tx_len_align;

//	rx_len = ( ( rx_len + 3 ) / 4 ) * 4;
//	tx_len = ( ( tx_len + 3 ) / 4 ) * 4;
//
//	rx_len = ( rx_len + 31 ) & ~31;
//	tx_len = ( tx_len + 31 ) & ~31;

	buffer = ( char * ) ( ( ( unsigned int ) bsInfo->buff_area + 31 ) & ~31 );
	bufSize = bsInfo->buff_area_size;

//	bsInfo->buff_area = buf;

	// allocate buffers in required ratio from supplied memory area
	//while (bsInfo->buff_area_size > rx_len || bsInfo->buff_area_size > tx_len)
	if ( bufSize > rx_len || bufSize > tx_len )
	{
		int n;

		for ( n = 0; n < bsInfo->rx_buffs; n++ )
		{
			if ( bufSize < rx_len )
				break;

			p = ( ADI_ETHER_BUFFER * )buffer;

			buffer += rx_len;

			bufSize -= rx_len;

			p->pNext = rx_head;
			rx_head = p;
		}

		for ( n = 0; n < bsInfo->tx_buffs; n++ )
		{
			if ( bufSize < tx_len )
				break;

			p = ( ADI_ETHER_BUFFER * ) buffer;

			buffer += tx_len;

			bufSize -= tx_len;

			p->pNext = tx_head;
			tx_head = p;

			if ( 0 == n )
			{
				bsInfo->xmt_buffers_queue.pQueueTail = p;
			}
		}
	}

	// initialise each buffer's ADI_ETHER_BUFFER descriptor
	p = rx_head;
	count = 0;

	while ( p )
	{
		p->Data = ( char * ) p + sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead;

		p->ElementCount = bsInfo->rx_buff_datalen;
		p->ElementWidth = 1;
		p->CallbackParameter = p;
		p->ProcessedElementCount = 0;
		p->ProcessedFlag = 0;
		p->PayLoad = 0;
		p->x = 0;

		count += 1;
		p = p->pNext;
	}

	bsInfo->rx_buffs = count;

	p = tx_head;
	count = 0;

	while ( p )
	{
		p->Data = ( char * ) p + sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead;

		p->ElementCount = bsInfo->tx_buff_datalen;
		p->ElementWidth = 1;
		p->CallbackParameter = p;
		p->x = 0;
		p->PayLoad = 0;

		count += 1;
		p = p->pNext;
	}

	bsInfo->tx_buffs = count;

	// save the list of rx buffers until they are needed
	bsInfo->rcv_list = rx_head;

	// save the tx buffers in to queue until they are needed
	bsInfo->xmt_buffers_queue.pQueueHead = tx_head;
	bsInfo->xmt_buffers_queue.ElementCount = bsInfo->tx_buffs;

	return 1;
}


static int Init_EthCfgInfo ( char *MemArea, unsigned int inMemSize, int overhead, ETH_CFG_INFO *bsInfo )
{
	int i = 0;
	int nw_ifce_buffer_length = 0;
	unsigned int BUFF_AREA_START = 0;
	unsigned int BUFF_AREA_LEN = 0;
	int rx_len, tx_len;
	short buffs;

	BUFF_AREA_START = ( unsigned int ) ( ( ( unsigned int ) ( MemArea + 31 ) ) & ~31 );
	BUFF_AREA_LEN   = ( unsigned int ) inMemSize;

	// get driver specific overhead
	//adi_ether_GetBufferPrefix ( hDevice, ( void * ) &overhead );
	bsInfo->buff_overhead  = overhead;

	// Caluculate the total buffer space for rx/tx and for all n/w interfaces.
	nw_ifce_buffer_length = 0;

	rx_len = sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead + bsInfo->rx_buff_datalen;
	tx_len = sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead + bsInfo->tx_buff_datalen;

	/* make rx and tx lengths multiple of 32 byte cache lines */
	rx_len = ( rx_len + 31 ) & ~31;
	tx_len = ( tx_len + 31 ) & ~31;

	bsInfo->rx_len_align = rx_len;
	bsInfo->tx_len_align = tx_len;

	nw_ifce_buffer_length   = bsInfo->rx_buffs * rx_len;
	nw_ifce_buffer_length  += bsInfo->tx_buffs * tx_len;

	if ( BUFF_AREA_LEN  < nw_ifce_buffer_length )
	{
		//先满足最小的buff需求
		buffs = (bsInfo->rx_buffs < bsInfo->tx_buffs) ? bsInfo->rx_buffs:bsInfo->tx_buffs;

		nw_ifce_buffer_length  = buffs * ( rx_len + tx_len );

		if ( BUFF_AREA_LEN  < nw_ifce_buffer_length )
		{
			DEBUG_STATEMENT ( "InitBuff: NO enough memory!\n\n" );
			return -1;
		}

		if( bsInfo->rx_buffs == buffs )
		{
			bsInfo->tx_buffs = buffs + (BUFF_AREA_LEN  - nw_ifce_buffer_length)/tx_len;
		}
		else if( bsInfo->tx_buffs == buffs )
		{
			bsInfo->rx_buffs = buffs + (BUFF_AREA_LEN  - nw_ifce_buffer_length)/rx_len;
		}

//		nw_ifce_buffer_length  = bsInfo->rx_buffs * rx_len + bsInfo->tx_buffs * tx_len;

	}

//	bsInfo->buff_area = ( char * ) ( BUFF_AREA_START  );
//	bsInfo->buff_area_size = nw_ifce_buffer_length;


	bsInfo->buff_area = MemArea;
	bsInfo->buff_area_size = inMemSize;

	//
	return InitFrameBuff ( bsInfo );
}

int Alloc_EthMem(void)
{
	char *EthHeapBlock;
	int nRet;


	//eth0
	/* allocate memory  */
	EthHeapBlock = heap_malloc ( 1, g_ctEthHeapSize[0] );
	if ( EthHeapBlock == NULL )
	{
		DEBUG_PRINT ( " heap_malloc: in heap %d, failed to allocate memory to the stack \n\n" , 1);
		return -1;
	}

	/* init buf mem */
	nRet = Init_EthCfgInfo (EthHeapBlock, g_ctEthHeapSize[0], 2, &user_net_config_info[0] );
	if( nRet < 0 )
	{
		DEBUG_STATEMENT ( " Init_EthCfgInfo: failed!\n\n" );
		return nRet;
	}


	//eth1

	/* allocate memory  */
	EthHeapBlock = heap_malloc ( 2, g_ctEthHeapSize[1] );
	if ( EthHeapBlock == NULL )
	{
		DEBUG_PRINT ( " heap_malloc: in heap %d, failed to allocate memory to the stack \n\n" , 2);
		return -1;
	}

	/* init buf mem */
	nRet = Init_EthCfgInfo (EthHeapBlock, g_ctEthHeapSize[1], 2, &user_net_config_info[1] );
	if( nRet < 0 )
	{
		DEBUG_STATEMENT ( " Init_EthCfgInfo: failed!\n\n" );
		return nRet;
	}

	// exeth

	/* allocate memory  */
	EthHeapBlock = heap_malloc ( 3, g_ctEthHeapSize[2] );
	if ( EthHeapBlock == NULL )
	{
		DEBUG_PRINT ( " heap_malloc: in heap %d, failed to allocate memory to the stack \n\n" , 3);
		return -1;
	}
	/* init buf mem */
	nRet = Init_EthCfgInfo (EthHeapBlock, g_ctEthHeapSize[2], 2, &user_net_config_info[2] );
	if( nRet < 0 )
	{
		DEBUG_STATEMENT ( " Init_EthCfgInfo: failed!\n\n" );
		return nRet;
	}

	return 0;
}


