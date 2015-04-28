/*
 * loop_queue_pointer.c
 *
 *  Created on: 2015-4-3
 *      Author: Administrator
 */


#include "loop_queue.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "stdlib_bf.h"

//注意 这个队列的操作是对外提供存储单元的。所以，pop操作是在
int     Queue_init(Queue *q)
{
    memset((char*)q, 0, MAX_QUEUE_ITEM_NUM* sizeof(QueueItem) );
    //
    q->tail =  q->header = 0;

    return 0;
}


int     Queue_getCount(Queue *q)
{
    int    n=-1;
    if(q == NULL)
    {
    	return -1;
    }

#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    /* if there is no item in the queue, header and tail are both -1 */
    /* other wise, header and tail neither is -1 */

    n = ( q->tail - q->header + MAX_QUEUE_ITEM_NUM ) % MAX_QUEUE_ITEM_NUM;


#ifdef QUEUE_DEBUG
    printf("[%s()]: count=[%d]\n", __LINE__, n);
#endif
    return n;
}

int    Queue_getHeader(Queue *q)
{
    if(q == NULL)    return -1;
    return q->header;
}

int    Queue_getTail(Queue *q)
{
    if(q == NULL)    return -1;
    return q->tail;
}

/*
* Return: 1 - the queue is empty
*         0 - the queue is not empty
*        -1 - failed
*/
int    Queue_isEmpty(Queue *q)
{
    int    is_empty = 0;
    if(q == NULL)    return -1;

#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    /* if there is no item in the queue, header and tail are both -1 */
    /* other wise, header and tail neither is -1 */
    if(q->header ==  q->tail ) return QUEUE_IS_EMPTY;
    return is_empty;
}

/*
* Return: 1 - the queue is full
*         0 - the queue is not full
*        -1 - failed
*/

int    Queue_isFull(Queue *q)
{
    int    is_full = 0;

    if(q == NULL)    return -1;

#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif


    if ( ( q->tail + 1 ) % MAX_QUEUE_ITEM_NUM == q->header )
	{
		return QUEUE_IS_FULL;  // 入队前判断(预留一个存储单元)
	}

    return is_full;
}

/*
* Return:  0 - succeed
*         -1 - failed
*
*/
QueueItem*    Queue_pop_unused_buf(Queue *q)
{

    QueueItem* p = NULL;
    if(q == NULL)
    {
        return NULL;
    }
#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    if ( ( q->tail + 1 ) % MAX_QUEUE_ITEM_NUM == q->header )
    {
        return NULL;
    }

    p = q->item + q->tail ;

    q->tail = (q->tail+1)%MAX_QUEUE_ITEM_NUM;


#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    return p;
}

QueueItem*    Queue_pop_unprocesed_buf(Queue *q)
{
	QueueItem* p = NULL;
	if(q == NULL)
	{
		return NULL;
	}
#ifdef QUEUE_DEBUG
	printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

	if(q->header ==  q->tail )
	{
		return NULL;
	}

	p =  q->item + q->header;

	q->header = ( q->header + 1 ) % MAX_QUEUE_ITEM_NUM;


#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    return p;
}

/*
* Return:  0 - succeed
*
*/
int    Queue_clear(Queue *q)
{

	if(q == NULL)
    {
        return 0;
    }

#ifdef QUEUE_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    if(Queue_isEmpty(q) == QUEUE_IS_EMPTY)
    {
        return 0;
    }

    q->header = q->tail = 0;

    return 0;
}

#if 0
int   Init_RingBufferManager(RingBufferManager* pBufferManager,
		unsigned char* pMemSection,
		unsigned int nMemSize)
{
	unsigned int nTmp, nCount;

	if(!pBufferManager || !pMemSection || 0==nMemSize)
	{
		return 0;
	}

	nTmp = sizeof(BufferItem);
	nCount = (nMemSize + nTmp-1)/nTmp;

	pBufferManager->nItemSize = nTmp;

	pBufferManager->pItem =  ( BufferItem * ) ( ( ( unsigned int ) pMemSection + nTmp ) & ~nTmp );

#ifdef RING_BUFFER_MANAGER_DEBUG
    printf("[%s(%d)]: BufferItem Size =[%d], Item Start =[%d]\n",__FILE__, __LINE__, nTmp, pBufferManager->pItem);
#endif

	pBufferManager->nItemCapacity = nCount;
	pBufferManager->nStartIdx = 0;
	pBufferManager->nStopIdx = nCount;
	pBufferManager->nReadIdx = 0;
	pBufferManager->nWriteIdx = 0;

	return 1;
}

BufferItem*    Write_RingBufferManager(RingBufferManager* pBm, unsigned int Bytes)
{
	BufferItem* pRet = NULL;

	unsigned int nNeed, nWritePos;

	ENTER_RING_BUF_MUTEX_SECTION();

	if(pBm->nWriteIdx +1 ==  pBm->nReadIdx)
	{
#ifdef RING_BUFFER_MANAGER_DEBUG
		printf("[%s(%d)]: buffer is  full.\n",__FILE__, __LINE__);
#endif
		return NULL;
	}

	nNeed = (Bytes + 4 + pBm->nItemSize -1)/ pBm->nItemSize;

	nWritePos = pBm->nWriteIdx + nNeed;

	if(  pBm->nWriteIdx >= pBm->nReadIdx &&   pBm->nWriteIdx < pBm->nStopIdx)
	{
		if( nWritePos < pBm->nStopIdx )
		{
			pRet = pBm->pItem + pBm->nWriteIdx;
			pBm->nWriteIdx += nNeed;
		}
		else if( pBm->nStartIdx + nNeed < pBm->nReadIdx )
		{

			pRet =  pBm->pItem + pBm->nStartIdx;
			pBm->nWriteIdx =  pBm->nStartIdx + nNeed;

		}
		else
		{
#ifdef RING_BUFFER_MANAGER_DEBUG
			printf("[%s(%d)]: no more BufferItem for %d Bytes.\n",__FILE__, __LINE__, Bytes);
#endif
			return NULL;
		}
	}
	else if( pBm->nWriteIdx < pBm->nReadIdx )
	{
		if( nWritePos < pBm->nReadIdx )
		{
			pRet =  pBm->pItem + pBm->nWriteIdx;
			pBm->nWriteIdx += nNeed;

		}
		else
		{
#ifdef RING_BUFFER_MANAGER_DEBUG
			printf("[%s(%d)]: no more BufferItem for %d Bytes.\n",__FILE__, __LINE__, Bytes);
#endif
			return NULL;
		}
	}

	pRet->nUsedItems = nNeed;
	pRet->nDataLen = Bytes;

	EXIT_RING_BUF_MUTEX_SECTION();

	return pRet;
}


BufferItem*    Read_RingBufferManager(RingBufferManager* pBm)
{
	BufferItem* pRet = NULL;
	ENTER_RING_BUF_MUTEX_SECTION();

	if(pBm->nWriteIdx  ==  pBm->nReadIdx)
	{
	#ifdef RING_BUFFER_MANAGER_DEBUG
		printf("[%s(%d)]: buffer is  empty.\n",__FILE__, __LINE__);
	#endif
		return NULL;
	}

	pRet = pBm->pItem + pBm->nReadIdx;

	pBm->nReadIdx += pRet->nUsedItems;

	EXIT_RING_BUF_MUTEX_SECTION();
	return pRet;
}

#endif
