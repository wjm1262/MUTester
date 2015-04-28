/*
 * loop_queue_pointer.h
 *
 *  Created on: 2015-4-3
 *      Author: Administrator
 */

#ifndef LOOP_QUEUE_POINTER_H_
#define LOOP_QUEUE_POINTER_H_

#define		CDM_FRM_MAX_SIZE 1518
#define		MAX_QUEUE_ITEM_NUM    500

#define    QUEUE_IS_FULL        1
#define    QUEUE_IS_EMPTY        1


typedef struct {
    unsigned short Size;
    unsigned char   Data[CDM_FRM_MAX_SIZE];
} QueueItem;

typedef struct {
    QueueItem    item[MAX_QUEUE_ITEM_NUM];
    unsigned int    header;    /* the first filled index  */
    unsigned int    tail;    /* the latest filled index */
} Queue;

typedef Queue EXETH_RECV_QUEUE;

int     Queue_init(Queue *q);

int    Queue_getCount(Queue *q);

int    Queue_getHeader(Queue *q);
int    Queue_getTail(Queue *q);

int    Queue_isEmpty(Queue *q);
int    Queue_isFull(Queue *q);

QueueItem*    Queue_pop_unused_buf(Queue *q);
QueueItem*    Queue_pop_unprocesed_buf(Queue *q);

int    Queue_clear(Queue *q);

/////////////////////////////////
#if 0
#define ENTER_RING_BUF_MUTEX_SECTION()
#define EXIT_RING_BUF_MUTEX_SECTION()

#define DATA_SECTION_SIZE 252
typedef struct {
    unsigned short    nUsedItems;
    unsigned short   nDataLen;
    unsigned char    Data[DATA_SECTION_SIZE];
} BufferItem;

typedef struct {
	BufferItem*    pItem; /* used in loop */
    unsigned int 	nItemCapacity;//
    unsigned int 	nItemSize;// the size of item, in bytes.
    unsigned int    nStartIdx;    /* the first filled index  */
    unsigned int    nReadIdx;    /* the read index  */
    unsigned int    nWriteIdx;    /* the write index */
    unsigned int    nStopIdx;    /* the latest filled index */
} RingBufferManager;
#endif


#endif /* LOOP_QUEUE_POINTER_H_ */
