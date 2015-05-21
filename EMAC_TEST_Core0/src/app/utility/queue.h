
#ifndef __QUEUE_H
#define __QUEUE_H

//#include <drivers/ethernet/adi_ether.h>

#include <stdio.h>
#include <string.h>


#include "arith.h"

//////////////////

#define Q_OK 1
#define Q_ERROR 0
#define Q_OVERFLOW -2
#define QUEUE_BUFFER_SIZE (10)




typedef struct
{
	bool bIsDoTimingTest;
	bool bTimingTestStarted;
//	int SnapshotSec;
//	int SnapshotNanosec;
	TimeInternal SnapshotTm;
}TIME_STAMP_TYPE;

typedef TIME_STAMP_TYPE QElem;
typedef struct
{
	QElem base[QUEUE_BUFFER_SIZE];
	QElem latest_elem;
	int front;
	int rear;
} AUXI_SNAPSHOT_TM_QUEUE;


typedef AUXI_SNAPSHOT_TM_QUEUE QType;

int InitQueue ( QType *pQ );
int QueueLength ( QType Q );

int EnQueue( QType *pQ, QElem* pElem);
int DeQueue ( QType *pQ, QElem *pe );

int GetHead ( QType *pQ, QElem *pe );
//pe «Input and Output
int GetElem ( QType *pQ, QElem *pe );
int GetLatest ( QType *pQ, QElem *pe );
void Visit_Q ( QType *pQ );

////////////////////////////////////////////////////////



#endif


