
/**************************************
***  ѭ������ Circular Queue
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
		return Q_ERROR;  // ���ǰ�ж�(Ԥ��һ���洢��Ԫ)
	}


//	pQ->base[pQ->rear].bTimingTestStarted = bTimingTestStarted;
//	pQ->base[pQ->rear].bIsDoTimingTest =  bIsDoTimingTest;
//
//	pQ->base[pQ->rear].SnapshotTm.nanoseconds = SnapshotTm->nanoseconds;
//	pQ->base[pQ->rear].SnapshotTm.seconds = SnapshotTm->seconds;

	pQ->base[pQ->rear] = *pElem;

	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // ����β��ָ������1

	return Q_OK;
}


int DeQueue ( QType *pQ, QElem *pe )
{
	//�㷨2��2 ���Ӳ���
//	ENTER_CRITICAL_REGION();

	if ( pQ->front == pQ->rear )
	{
//		EXIT_CRITICAL_REGION();
		return Q_ERROR;  // ������ǰ�ж�
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
	if ( pQ->front == pQ->rear ) return Q_ERROR;   // ��βָ����������
	
	pe->bTimingTestStarted =  pQ->base[pQ->front].bTimingTestStarted;
	pe->bIsDoTimingTest = pQ->base[pQ->front].bIsDoTimingTest;
	pe->SnapshotTm.nanoseconds  = pQ->base[pQ->front].SnapshotTm.nanoseconds ;
	pe->SnapshotTm.seconds = pQ->base[pQ->front].SnapshotTm.seconds;

//
	return Q_OK;
}
int GetLatest ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return Q_ERROR;   // ��βָ����������

//	pe->bTimingTestStarted =  pQ->base[pQ->front].bTimingTestStarted;
//	pe->bIsDoTimingTest = pQ->base[pQ->front].bIsDoTimingTest;
//	pe->SnapshotTm.nanoseconds  = pQ->base[pQ->front].SnapshotTm.nanoseconds ;
//	pe->SnapshotTm.seconds = pQ->base[pQ->front].SnapshotTm.seconds;
	*pe = pQ->latest_elem;

//
	return Q_OK;
}
//pe��Input and Output
int GetElem ( QType *pQ, QElem *pe )
{
	int i = pQ->front;
	QElem temp;
	
	
	if ( pQ->front == pQ->rear )
	{
		return Q_ERROR;
	}
	
	/*
	while ( i != pQ->rear )       // �����������в���IDƥ���ʱ��
	{
	
		if ( pQ->base[i].ID == pe->ID )
		{
			*pe = pQ->base[i];    // structure assignment
	
			if ( i != pQ->front ) // ���ȡ����ʱ������ǵ�ǰfront��ָ
			{
				pQ->base[i] = pQ->base[pQ->front];	// ��ǰ��ֵ����ȡ����λ��
			}
	
			pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;
	
			return OK;
		}
	
		i = ( i + 1 ) % QUEUE_BUFFER_SIZE;      // ������������
	}
	*/
	return Q_ERROR;
}

void Visit_Q ( QType *pQ )  // ��������
{
	int i = pQ->front;
	//printf ( "\n\t��ǰ����״̬��" );
	
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
		return Q_ERROR;  // ���ǰ�ж�(Ԥ��һ���洢��Ԫ)
	}

	*(pQ->base + pQ->rear ) = Elem;

	pQ->rear = ( pQ->rear + 1 ) % DM9000A_INT_EVENT_QUEUE_BUFFER_SIZE;  // ����β��ָ������1

	return Q_OK;
}

int De_DM9000A_INT_EVENT_Queue ( QueueType *pQ, int *pe )
{
	if ( pQ->front == pQ->rear )
	{
		return Q_ERROR;  // ������ǰ�ж�
	}

	*pe = *(pQ->base + pQ->front);

	pQ->front = ( pQ->front + 1 ) % DM9000A_INT_EVENT_QUEUE_BUFFER_SIZE;

	return Q_OK;
}



