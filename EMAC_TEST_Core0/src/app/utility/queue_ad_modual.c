
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
		return NULL;  // ���ǰ�ж�(Ԥ��һ���洢��Ԫ)
	}


	QElem* pElem = pQ->base + pQ->rear;

	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // ����β��ָ������1

	return pElem;
}


QElem * PopQueue ( QType *pQ )
{

//	ENTER_CRITICAL_REGION();

	if ( pQ->front == pQ->rear )
	{

//		EXIT_CRITICAL_REGION();
		DEBUG_PRINT("%s[#%d]:PopQueue overflow...\n\n", __FILE__, __LINE__);
		return NULL;  // ������ǰ�ж�
	}
	
//	EXIT_CRITICAL_REGION();

	QElem* pElem = pQ->base + pQ->front;
	
	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;


	return pElem;
}


int GetHead ( QType *pQ, QElem *pe )
{
	if ( pQ->front == pQ->rear ) return Q_ERROR;   // ��βָ����������
	
	*pe =  pQ->base[pQ->front];

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
		return NULL;  // ���ǰ�ж�(Ԥ��һ���洢��Ԫ)
	}


	QElem* pElem = pQ->base + pQ->rear;

	pQ->rear = ( pQ->rear + 1 ) % QUEUE_BUFFER_SIZE;  // ����β��ָ������1

	return pElem;
}


Ft3FrmItem * PopFt3FrmQueue ( Ft3FrmQueue *pQ )
{

//	ENTER_CRITICAL_REGION();

	if ( pQ->front == pQ->rear )
	{
//		EXIT_CRITICAL_REGION();
		DEBUG_PRINT("%s[#%d]:PopFt3FrmQueue overflow...\n\n", __FILE__, __LINE__);
		return NULL;  // ������ǰ�ж�
	}

//	EXIT_CRITICAL_REGION();

	QElem* pElem = pQ->base + pQ->front;

	pQ->front = ( pQ->front + 1 ) % QUEUE_BUFFER_SIZE;


	return pElem;
}

