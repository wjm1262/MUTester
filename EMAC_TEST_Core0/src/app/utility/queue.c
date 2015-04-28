
/**************************************
***  循环队列 Circular Queue
***
**************************************/


#include "stdlib.h"
#include "stdlib_bf.h"
#include "queue.h"
#include "post_debug.h"


int InitQueue ( QType *pQ )
{
	pQ->front = pQ->rear = 0;
	return Q_OK;
}

int QueueLength ( QType Q )
{
	return ( Q.rear - Q.front + QUEUE_BUFFER_SIZE ) % QUEUE_BUFFER_SIZE;
}



int EnQueue( QType *pQ, QElem* pElem)
{
	if ( ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE == pQ->front )
	{
		return Q_ERROR;  // 入队前判断(预留一个存储单元)
	}


//	pQ->base[pQ->rear].bTimingTestStarted = bTimingTestStarted;
//	pQ->base[pQ->rear].bIsDoTimingTest =  bIsDoTimingTest;
//
//	pQ->base[pQ->rear].SnapshotTm.nanoseconds = SnapshotTm->nanoseconds;
//	pQ->base[pQ->rear].SnapshotTm.seconds = SnapshotTm->seconds;

	pQ->base[pQ->rear] = *pElem;

	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // 队列尾部指针增加1

	return Q_OK;
}


int DeQueue ( QType *pQ, QElem *pe )
{
	//算法2－2 出队操作
//	ENTER_CRITICAL_REGION();

	if ( pQ->front == pQ->rear )
	{
//		EXIT_CRITICAL_REGION();
		return Q_ERROR;  // 出队列前判断
	}
	
//	EXIT_CRITICAL_REGION();

	//DEBUG_STATEMENT ( " DeQueue\n\n" );
	pe->bTimingTestStarted =  pQ->base[pQ->front].bTimingTestStarted;
	pe->bIsDoTimingTest = pQ->base[pQ->front].bIsDoTimingTest;
	pe->SnapshotTm.nanoseconds  = pQ->base[pQ->front].SnapshotTm.nanoseconds ;
	pe->SnapshotTm.seconds = pQ->base[pQ->front].SnapshotTm.seconds;
	
	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;

	//DEBUG_STATEMENT ( " exit DeQueue\n\n" );

	return Q_OK;
}


int GetHead ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return Q_ERROR;   // 首尾指针相等则出错
	
	pe->bTimingTestStarted =  pQ->base[pQ->front].bTimingTestStarted;
	pe->bIsDoTimingTest = pQ->base[pQ->front].bIsDoTimingTest;
	pe->SnapshotTm.nanoseconds  = pQ->base[pQ->front].SnapshotTm.nanoseconds ;
	pe->SnapshotTm.seconds = pQ->base[pQ->front].SnapshotTm.seconds;

//
	return Q_OK;
}
int GetLatest ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return Q_ERROR;   // 首尾指针相等则出错

//	pe->bTimingTestStarted =  pQ->base[pQ->front].bTimingTestStarted;
//	pe->bIsDoTimingTest = pQ->base[pQ->front].bIsDoTimingTest;
//	pe->SnapshotTm.nanoseconds  = pQ->base[pQ->front].SnapshotTm.nanoseconds ;
//	pe->SnapshotTm.seconds = pQ->base[pQ->front].SnapshotTm.seconds;
	*pe = pQ->latest_elem;

//
	return Q_OK;
}
//pe是Input and Output
int GetElem ( QType *pQ, QElem *pe )
{
	int i = pQ->front;
	QElem temp;
	
	
	if ( pQ->front == pQ->rear )
	{
		return Q_ERROR;
	}
	
	/*
	while ( i != pQ->rear )       // 遍历整个队列查找ID匹配的时标
	{
	
		if ( pQ->base[i].ID == pe->ID )
		{
			*pe = pQ->base[i];    // structure assignment
	
			if ( i != pQ->front ) // 如果取出的时间戳不是当前front所指
			{
				pQ->base[i] = pQ->base[pQ->front];	// 当前的值赋给取出的位置
			}
	
			pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;
	
			return OK;
		}
	
		i = ( i + 1 ) % QUEUE_BUFFER_SIZE;      // 遍历整个队列
	}
	*/
	return Q_ERROR;
}

void Visit_Q ( QType *pQ )  // 遍历队列
{
	int i = pQ->front;
	//printf ( "\n\t当前队列状态：" );
	
	if ( pQ->front == pQ->rear )  printf ( "Queue is empty \n\n" );
	
	else
	{
		printf ( "->" );
		
		while ( i != pQ->rear )
		{
			//printf ( "%d, %0x; ", pQ->base[i].ID,  pQ->base[i].SnapTime );
			i = ( i + 1 ) % QUEUE_BUFFER_SIZE;
		}
		
		printf ( "\t<-\n\n" );
	}
}

////////////////////////
int Init_DM9000A_INT_EVENT_Queue ( QueueType *pQ )
{
	pQ->front = pQ->rear = 0;
	memset(pQ->base, 0, DM9000A_INT_EVENT_QUEUE_BUFFER_SIZE*sizeof(int) );
	return Q_OK;
}

int En_DM9000A_INT_EVENT_Queue( QueueType *pQ, int Elem)
{
	if ( ( pQ->rear + 1 ) % DM9000A_INT_EVENT_QUEUE_BUFFER_SIZE == pQ->front )
	{
		return Q_ERROR;  // 入队前判断(预留一个存储单元)
	}

	*(pQ->base + pQ->rear ) = Elem;

	pQ->rear = ( pQ->rear + 1 ) % DM9000A_INT_EVENT_QUEUE_BUFFER_SIZE;  // 队列尾部指针增加1

	return Q_OK;
}

int De_DM9000A_INT_EVENT_Queue ( QueueType *pQ, int *pe )
{
	if ( pQ->front == pQ->rear )
	{
		return Q_ERROR;  // 出队列前判断
	}

	*pe = *(pQ->base + pQ->front);

	pQ->front = ( pQ->front + 1 ) % DM9000A_INT_EVENT_QUEUE_BUFFER_SIZE;

	return Q_OK;
}



