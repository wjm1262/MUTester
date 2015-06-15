
#ifndef __QUEUE_H
#define __QUEUE_H

//#include <drivers/ethernet/adi_ether.h>

#include <stdio.h>
#include <string.h>

#include "comm_pc_protocol.h"

//////////////////

#define Q_OK 1
#define Q_ERROR 0
#define Q_OVERFLOW -2
#define QUEUE_BUFFER_SIZE (8000)


typedef STAND_SAMP_TYPE QElem;
typedef struct
{
	QElem base[QUEUE_BUFFER_SIZE];
	int front;
	int rear;
} AD_STAND_SMPDATA_Q;


typedef AD_STAND_SMPDATA_Q QType;

int InitQueue ( QType *pQ );
int QueueLength ( QType Q );


QElem* PushQueue( QType *pQ );

QElem* PopQueue ( QType *pQ );


int GetHead ( QType *pQ, QElem *pe );

//pe «Input and Output
int GetElem ( QType *pQ, QElem *pe );

void Visit_Q ( QType *pQ );

////////////////////////////////////////////////////////


#define MAX_FT3_FRM_SIZE 512
#define MAX_FT3_FRM_Q_SIZE 4096
typedef struct
{
    unsigned int  FrmLen;
    unsigned char   Ft3FrmData[MAX_FT3_FRM_SIZE];
}  Ft3FrmItem;

typedef struct
{
	Ft3FrmItem    base[MAX_FT3_FRM_Q_SIZE];
    unsigned int    front;    /* the first filled index  */
    unsigned int    rear;    /* the latest filled index */
} Ft3FrmQueue;



int InitFt3FrmQueue ( Ft3FrmQueue *pQ );

int Ft3FrmQueueLength ( Ft3FrmQueue Q );

Ft3FrmItem* PushFt3FrmQueue( Ft3FrmQueue *pQ );

Ft3FrmItem* PopFt3FrmQueue ( Ft3FrmQueue *pQ );





#endif


