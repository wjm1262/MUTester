/*
 * loop_Queue_pointer.h
 *
 *  Created on: 2015-4-3
 *      Author: wu jm
 */

#ifndef LOOP_Queue_H_
#define LOOP_Queue_H_

#define		CDM_FRM_MAX_SIZE 1518
#define		QUEUE_MAX_ITEM_NUM    512

#define    LoopQueue_IS_FULL        1
#define    LoopQueue_IS_EMPTY        1


typedef struct {
    unsigned int Size;
    unsigned char   Data[CDM_FRM_MAX_SIZE];
} LoopQueueItem;

typedef struct {
    LoopQueueItem    item[QUEUE_MAX_ITEM_NUM];
    unsigned int    header;    /* the first filled index  */
    unsigned int    tail;    /* the latest filled index */
} LoopQueue;



int    LoopQueue_init(LoopQueue *q);

int    LoopQueue_getCount(LoopQueue *q);

int    LoopQueue_getHeader(LoopQueue *q);
int    LoopQueue_getTail(LoopQueue *q);

int    LoopQueue_isEmpty(LoopQueue *q);
int    LoopQueue_isFull(LoopQueue *q);

LoopQueueItem*    LoopQueue_push(LoopQueue *q);
LoopQueueItem*    LoopQueue_pop(LoopQueue *q);

int    LoopQueue_clear(LoopQueue *q);



#endif /* LOOP_Queue_H_ */
