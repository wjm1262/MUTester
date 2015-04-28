/*
 * dev_exEth.h
 *
 *  Created on: 2015-3-30
 *      Author: Administrator
 */

#ifndef DEV_EXETH_H_
#define DEV_EXETH_H_

#include "dri_adi_gemac.h"
#include "dri_dm9000a.h"
#include "loop_queue.h"
#include "queue.h"
/*! Frame queue */
typedef struct EXEMAC_FRAME_Q
{
	ADI_ETHER_BUFFER  *pQueueHead;        /*!< frame queue head */
	ADI_ETHER_BUFFER  *pQueueProcessed;        /*!< frame queue head */
	ADI_ETHER_BUFFER  *pQueueTail;        /*!< frame queue tail */
	int32_t            ElementCount;      /*!< number of buffers in a queue */
	int32_t            ProcessedElementCount;      /*!< number of buffers in a queue */

} EXEMAC_FRAME_Q;

extern EXEMAC_FRAME_Q g_ExEthXmtQueue;
extern EXETH_RECV_QUEUE g_ExEthRecvQueue;
extern DM9000A_INT_EVENT_QUEUE g_Dm9000aIntEventQueue;

int init_exemac_queue ( EXEMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pBuffer );

void RegisterExEthnetModual(void);

#endif /* DEV_EXETH_H_ */
