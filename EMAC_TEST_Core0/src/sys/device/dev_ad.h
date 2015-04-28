/*
 * dev_ad.h
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#ifndef DEV_AD_H_
#define DEV_AD_H_


#include "dri_ad7608.h"

void RegisterADModual(void);

//typedef struct _AD_BUFFER
//{
//    void     *Data;
//    uint32_t  DataCount;                    /*!< Data element count. In bytes         */
//    struct   _AD_BUFFER  *pNext;        /*!< Next buffer pointer.         */
//} AD_BUFFER;
//
//
///*! AD Frame queue */
//typedef struct AD_FRAME_Q
//{
//	AD_BUFFER  *pQueueHead;        /*!< frame queue head */
//	AD_BUFFER  *pQueueTail;        /*!< frame queue tail */
//	int32_t            ElementCount;      /*!< number of buffers in a queue */
//
//} AD_FRAME_Q;
//
//typedef struct ad_cfg_info
//{
//	short                     buffs;
//	short                     tx_buffs;
//	// the maximum data size that each receive and transmit buffer must support
//	short                     buff_datalen;
//	short 					len_align;
//
//
//	// the address and size of the area from which buffers are to be allocated
//	// (must be 32-bit aligned, uncached and accessible by the controller)
//	char                     *buff_area;
//	int                       buff_area_size;
//
//
//	// netif's device driver handle
//	void                     *handle;
//
//
//	AD_FRAME_Q buffers_queue;
//
//	// keeps track if there is a already a post. if its zero stack callback handler will
//	// post the message, else it will be skipped.
//	//
//	int                       txmsg_processed;
//	int                       rxmsg_processed;
//	// queue of received buffers have been completed by EMAC awaiting APP processing/disposal
//	ADI_EMAC_FRAME_Q rx_completed_queue;
//	//ADI_EMAC_FRAME_Q tx_completed_queue;
//
//} ETH_CFG_INFO;

#endif /* DEV_AD_H_ */
