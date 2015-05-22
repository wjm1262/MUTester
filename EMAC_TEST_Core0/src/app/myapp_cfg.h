/*
 * app_cfg.h
 *
 *  Created on: 2015-2-9
 *      Author: Administrator
 */

#ifndef APP_CFG_H_
#define APP_CFG_H_

#include "device_def.h"

#if 0
typedef struct eth_cfg_info
{
	size_t                    buff_overhead;
	short                     buffs;
//	short                     tx_buffs;
	// the maximum data size that each receive and transmit buffer must support
	short                     datalen;
//	short                     tx_buff_datalen;
	short 					  len_align;
//	short					tx_len_align;

	// the address and size of the area from which buffers are to be allocated
	// (must be 32-bit aligned, uncached and accessible by the controller)
	char                     *buff_area;
	int                       buff_area_size;

	// interface's individual (MAC) address
	unsigned char             hwaddr[6];

	// netif's device driver handle
	void                     *handle;


	// keeps track if there is a already a post. if its zero stack callback handler will
	// post the message, else it will be skipped.
	//
	int                       txmsg_processed;
	int                       rxmsg_processed;
	// queue of received buffers have been completed by EMAC awaiting APP processing/disposal
	ADI_EMAC_FRAME_Q rx_completed_queue;
	ADI_EMAC_FRAME_Q tx_completed_queue;

	// lists of received/transmitted buffers awaiting to be submit to Dev'RX/TX Channel
	//ADI_ETHER_BUFFER         *rcv_list;
	//ADI_ETHER_BUFFER*         xmt_list;
	ADI_EMAC_FRAME_Q 	buffers_queue;

} ETH_CFG_INFO;


#endif

typedef struct eth_cfg_info
{
	size_t                    buff_overhead;
	short                     rx_buffs;
	short                     tx_buffs;
	// the maximum data size that each receive and transmit buffer must support
	short                     rx_buff_datalen;
	short                     tx_buff_datalen;
	short 					rx_len_align;
	short					tx_len_align;

	// the address and size of the area from which buffers are to be allocated
	// (must be 32-bit aligned, uncached and accessible by the controller)
	char                     *buff_area;
	int                       buff_area_size;

	// interface's individual (MAC) address
	unsigned char             hwaddr[6];

	// netif's device driver handle
	void                     *handle;

	// lists of received/transmitted buffers awaiting to be submit to Dev'RX/TX Channel
	ADI_ETHER_BUFFER         *rcv_list;
	//ADI_ETHER_BUFFER*         xmt_list;
	ADI_EMAC_FRAME_Q xmt_buffers_queue;

	// keeps track if there is a already a post. if its zero stack callback handler will
	// post the message, else it will be skipped.
	//
	int                       txmsg_processed;
	int                       rxmsg_processed;
	// queue of received buffers have been completed by EMAC awaiting APP processing/disposal
	ADI_EMAC_FRAME_Q rx_completed_queue;
	//ADI_EMAC_FRAME_Q tx_completed_queue;

} ETH_CFG_INFO;

extern ETH_CFG_INFO user_net_config_info[5];
extern unsigned long int user_net_num_ifces;

int Alloc_EthMem(void);

#endif /* APP_CFG_H_ */
