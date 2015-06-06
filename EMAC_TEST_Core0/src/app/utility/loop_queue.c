/*
 * loop_LoopQueue_pointer.c
 *
 *  Created on: 2015-4-3
 *      Author: wu jm
 */


#include "loop_Queue.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "stdlib_bf.h"

//注意 这个队列的操作是对外提供存储单元的。所以，pop操作是在
int     LoopQueue_init(LoopQueue *q)
{
    memset((char*)q, 0, QUEUE_MAX_ITEM_NUM* sizeof(LoopQueueItem) );
    //
    q->tail =  q->header = 0;

    return 0;
}


int     LoopQueue_getCount(LoopQueue *q)
{
    int    n=-1;
    if(q == NULL)
    {
    	return -1;
    }

#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    /* if there is no item in the LoopQueue, header and tail are both -1 */
    /* other wise, header and tail neither is -1 */

    n = ( q->tail - q->header + QUEUE_MAX_ITEM_NUM ) % QUEUE_MAX_ITEM_NUM;


#ifdef LoopQueue_DEBUG
    printf("[%s()]: count=[%d]\n", __LINE__, n);
#endif
    return n;
}

int    LoopQueue_getHeader(LoopQueue *q)
{
    if(q == NULL)    return -1;
    return q->header;
}

int    LoopQueue_getTail(LoopQueue *q)
{
    if(q == NULL)    return -1;
    return q->tail;
}

/*
* Return: 1 - the LoopQueue is empty
*         0 - the LoopQueue is not empty
*        -1 - failed
*/
int    LoopQueue_isEmpty(LoopQueue *q)
{
    int    is_empty = 0;
    if(q == NULL)    return -1;

#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    /* if there is no item in the LoopQueue, header and tail are both -1 */
    /* other wise, header and tail neither is -1 */
    if(q->header ==  q->tail ) return LoopQueue_IS_EMPTY;
    return is_empty;
}

/*
* Return: 1 - the LoopQueue is full
*         0 - the LoopQueue is not full
*        -1 - failed
*/

int    LoopQueue_isFull(LoopQueue *q)
{
    int    is_full = 0;

    if(q == NULL)    return -1;

#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif


    if ( ( q->tail + 1 ) % QUEUE_MAX_ITEM_NUM == q->header )
	{
		return LoopQueue_IS_FULL;  // 入队前判断(预留一个存储单元)
	}

    return is_full;
}

/*
* Return:  0 - succeed
*         -1 - failed
*
*/
LoopQueueItem*    LoopQueue_push(LoopQueue *q)
{

    LoopQueueItem* p = NULL;
    if(q == NULL)
    {
        return NULL;
    }
#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    if ( ( q->tail + 1 ) % QUEUE_MAX_ITEM_NUM == q->header )
    {
        return NULL;
    }

    p = q->item + q->tail ;

    q->tail = (q->tail+1)%QUEUE_MAX_ITEM_NUM;


#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    return p;
}

LoopQueueItem*    LoopQueue_pop(LoopQueue *q)
{
	LoopQueueItem* p = NULL;
	if(q == NULL)
	{
		return NULL;
	}
#ifdef LoopQueue_DEBUG
	printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

	if(q->header ==  q->tail )
	{
		return NULL;
	}

	p =  q->item + q->header;

	q->header = ( q->header + 1 ) % QUEUE_MAX_ITEM_NUM;


#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    return p;
}

/*
* Return:  0 - succeed
*
*/
int    LoopQueue_clear(LoopQueue *q)
{

	if(q == NULL)
    {
        return 0;
    }

#ifdef LoopQueue_DEBUG
    printf("[%s()]: q->header=[%d], q->tail=[%d]\n", __LINE__, q->header, q->tail);
#endif

    if(LoopQueue_isEmpty(q) == LoopQueue_IS_EMPTY)
    {
        return 0;
    }

    q->header = q->tail = 0;

    return 0;
}

