
/**************************************
***  Circular Queue
***
**************************************/


#include "stdlib.h"
#include "stdlib_bf.h"
#include "queue_ad_modual.h"
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



QElem* PushQueue( QType *pQ )
{
	if ( ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE == pQ->front )
	{
		DEBUG_PRINT("%s[#%d]:PushQueue overflow...\n\n", __FILE__, __LINE__);
		return NULL;  // 入队前判断(预留一个存储单元)
	}


	QElem* pElem = pQ->base + pQ->rear;

	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // 队列尾部指针增加1

	return pElem;
}


QElem * PopQueue ( QType *pQ )
{

//	ENTER_CRITICAL_REGION();

	if ( pQ->front == pQ->rear )
	{

//		EXIT_CRITICAL_REGION();
		DEBUG_PRINT("%s[#%d]:PopQueue overflow...\n\n", __FILE__, __LINE__);
		return NULL;  // 出队列前判断
	}
	
//	EXIT_CRITICAL_REGION();

	QElem* pElem = pQ->base + pQ->front;
	
	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;


	return pElem;
}


int GetHead ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return Q_ERROR;   // 首尾指针相等则出错
	
	*pe =  pQ->base[pQ->front];

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

int InitFt3FrmQueue ( Ft3FrmQueue *pQ )
{
	pQ->front = pQ->rear = 0;
	return Q_OK;
}

int Ft3FrmQueueLength ( Ft3FrmQueue Q )
{
	return ( Q.rear - Q.front + QUEUE_BUFFER_SIZE ) % QUEUE_BUFFER_SIZE;
}



Ft3FrmItem* PushFt3FrmQueue( Ft3FrmQueue *pQ )
{
	if ( ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE == pQ->front )
	{
		DEBUG_PRINT("%s[#%d]:PushFt3FrmQueue overflow...\n\n", __FILE__, __LINE__);
		return NULL;  // 入队前判断(预留一个存储单元)
	}


	QElem* pElem = pQ->base + pQ->rear;

	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // 队列尾部指针增加1

	return pElem;
}


Ft3FrmItem * PopFt3FrmQueue ( Ft3FrmQueue *pQ )
{

//	ENTER_CRITICAL_REGION();

	if ( pQ->front == pQ->rear )
	{
//		EXIT_CRITICAL_REGION();
		DEBUG_PRINT("%s[#%d]:PopFt3FrmQueue overflow...\n\n", __FILE__, __LINE__);
		return NULL;  // 出队列前判断
	}

//	EXIT_CRITICAL_REGION();

	QElem* pElem = pQ->base + pQ->front;

	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;


	return pElem;
}

