/*
 * Task_eth.c
 *
 *  Created on: 2015-2-9
 *      Author: Wu JM
 */
#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"

#include "myapp_cfg.h"

#include "xl-6004_forward_protocol.h"
#include "mutester_comm_protocol.h"
#include "post_debug.h"
/*
 * Report error if result code is not success
 */



#define CHECK_ETH_RESULT(result, message)           \
   if(result != ADI_ETHER_RESULT_SUCCESS)                       \
   {                                              \
		DEBUG_PRINT("%s failed\n", message);           \
   }

//static void CheckResult(ADI_ETHER_RESULT result, char *message)
//{
//	if (result != ADI_GPIO_SUCCESS)
//	{
//		DEBUG_PRINT("%s failed\n", message);
//		bError = true;
//	}
//}
#define CHECK_OS_RESULT(osErr, message)           \
   if(osErr != OS_ERR_NONE)                       \
   {                                              \
		DEBUG_PRINT("%s failed\n", message);           \
   }


void Task_Eth0_Rx( void *p_arg )
{
	void        *p_msg;
//	OS_ERR       osErr;
//	OS_MSG_SIZE  msg_size;
//	CPU_TS       ts;

	ADI_ETHER_BUFFER *pRecv = NULL,*pSend = NULL, *pCopyRecv = NULL;
	ADI_ETHER_RESULT eResult;

	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;
	uint32_t nanSeconds = 0;
	uint32_t FrmLen = 0;

	MuTesterSystem.Device.Eth0.EnableRxTimeStamped( g_hEthDev[0] );
	eResult = MuTesterSystem.Device.Eth0.Read( g_hEthDev[0], user_net_config_info[0].rcv_list);
	CHECK_ETH_RESULT(eResult, "Eth0.Read");

	MuTesterSystem.Device.Eth0.EnableGemacInt( g_hEthDev[0] );
	MuTesterSystem.Device.Eth0.EnableGemacDMA( g_hEthDev[0], RX );
	MuTesterSystem.Device.Eth0.EnableGemacRx( g_hEthDev[0] );

//	OSTaskDel((OS_TCB*)0, &osErr);
}

void Task_Eth0_Tx( void *p_arg )
{
	void        *p_msg;
//	OS_ERR       osErr;
//	OS_MSG_SIZE  msg_size;
//	CPU_TS       ts;

	ADI_ETHER_BUFFER *pXmt = NULL;
	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;
	ADI_ETHER_FRAME_BUFFER *pPkt = NULL;

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) g_hEthDev[0];

	MuTesterSystem.Device.Eth0.EnableGemacInt( g_hEthDev[0] );
	MuTesterSystem.Device.Eth0.EnableGemacDMA( g_hEthDev[0], TX );
	MuTesterSystem.Device.Eth0.EnableGemacTx( g_hEthDev[0] );

//	OSTaskDel((OS_TCB*)0, &osErr);
}

void Task_Eth1_Rx( void *p_arg )
{
	void        *p_msg;
//	OS_ERR       osErr;
//	OS_MSG_SIZE  msg_size;
//	CPU_TS       ts;

	ADI_ETHER_BUFFER *pRecv = NULL,*pSend = NULL, *pCopyRecv = NULL;
	ADI_ETHER_RESULT eResult;

	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;
	uint32_t nanSeconds = 0;
	uint32_t FrmLen = 0;

	MuTesterSystem.Device.Eth1.EnableRxTimeStamped( g_hEthDev[1] );
	eResult = MuTesterSystem.Device.Eth1.Read( g_hEthDev[1], user_net_config_info[1].rcv_list);
	CHECK_ETH_RESULT(eResult, "Eth1.Read");

	MuTesterSystem.Device.Eth1.EnableGemacInt( g_hEthDev[1] );
	MuTesterSystem.Device.Eth1.EnableGemacDMA( g_hEthDev[1], RX );
	MuTesterSystem.Device.Eth1.EnableGemacRx( g_hEthDev[1] );

//	OSTaskDel((OS_TCB*)0, &osErr);
}

void Task_Eth1_Tx( void *p_arg )
{
	void        *p_msg;
//	OS_ERR       osErr;
//	OS_MSG_SIZE  msg_size;
//	CPU_TS       ts;

	ADI_ETHER_BUFFER *pXmt = NULL;
	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;
	ADI_ETHER_FRAME_BUFFER *pPkt = NULL;

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) g_hEthDev[1];

	MuTesterSystem.Device.Eth1.EnableGemacInt( g_hEthDev[1] );
	MuTesterSystem.Device.Eth1.EnableGemacDMA( g_hEthDev[1], TX );
	MuTesterSystem.Device.Eth1.EnableGemacTx( g_hEthDev[1] );

//	OSTaskDel((OS_TCB*)0, &osErr);
}



void Ethernet0_Callback ( void *pArg1, unsigned int event, void *pArg2 )
{

	ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)g_hEthDev[0];


	ADI_ETHER_BUFFER *pNewBuffer, *pFrmHead, *pFrms = (ADI_ETHER_BUFFER*)pArg2;
	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;

	uint32_t unSecond = 0;
	uint32_t nanoSeconds = 0;
	uint32_t FrmLen = 0;
	int  n =0;

	//int int_sts = cli();
	ENTER_CRITICAL_REGION();

	pFrmHead = pFrms;

	switch ( event )
	{

		case ADI_ETHER_EVENT_FRAME_RCVD:

			while(pFrms)
			{
				//process frame
				//get time stamp
				pTmBuff = ( RX_TX_TIME_STAMP_BUFFER * ) pFrms;

				unSecond = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampHi ) ;
				nanoSeconds = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampLo ) ;

				FrmLen = pFrms->ProcessedElementCount - 6;

//				PackForwardSMVFrame ( nanoSeconds, FrmLen, pFrms );

				PackForwardFrame( TYPE609_CONT_NET_RECV1_DATA,  unSecond, nanoSeconds,
						FrmLen,
						pFrms);


				pFrms = pFrms->pNext;
				n++;
			}
#if 0
			//send by eth1
			MuTesterSystem.Device.Eth1.Write( g_hEthDev[1], pFrmHead);

			// get n buffers
			//
			pNewBuffer = pop_n_queue( &(user_net_config_info[1].xmt_buffers_queue), n );

			// add buffers to eth0
			MuTesterSystem.Device.Eth0.Read( g_hEthDev[0], pNewBuffer );
			//CHECK_ETH_RESULT(eResult, "Eth0.Read");
#else
			//send by exEth
			MuTesterSystem.Device.exEth.PushUnprocessElem(&g_ExEthXmtQueue, pFrmHead);

			// get n buffers
			pNewBuffer = MuTesterSystem.Device.exEth.PopProcessedElem( &g_ExEthXmtQueue, n );

			// Add buffers to eth0
			MuTesterSystem.Device.Eth0.Read( g_hEthDev[0], pNewBuffer );
#endif
			break;

		case ADI_ETHER_EVENT_FRAME_XMIT:

			push_queue( &(user_net_config_info[0].xmt_buffers_queue), pFrmHead );

			break;

		case ADI_ETHER_EVENT_INTERRUPT:
			break;

		case ADI_ETHER_EVENT_PHY_INTERRUPT:
			break;
	}

	//sti(int_sts);
	EXIT_CRITICAL_REGION();

}

void Ethernet1_Callback ( void *pArg1, unsigned int event, void *pArg2 )
{

	ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)g_hEthDev[1];


	ADI_ETHER_BUFFER *pNewBuffer, *pFrmHead, *pFrms = (ADI_ETHER_BUFFER*)pArg2;
	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;

	uint32_t unSecond = 0;
	uint32_t nanoSeconds = 0;
	uint32_t FrmLen = 0;
	int  n =0;

	//int int_sts = cli();
	ENTER_CRITICAL_REGION();
	pFrmHead = pFrms;

	switch ( event )
	{

		case ADI_ETHER_EVENT_FRAME_RCVD:

			while(pFrms)
			{
				//process frame
				//get time stamp
				pTmBuff = ( RX_TX_TIME_STAMP_BUFFER * ) pFrms;

				unSecond = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampHi ) ;
				nanoSeconds = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampLo ) ;

				FrmLen = pFrms->ProcessedElementCount - 6;

//				PackForwardSMVFrame ( nanoSeconds, FrmLen, pFrms );

				PackForwardFrame( TYPE609_CONT_NET_RECV2_DATA, unSecond, nanoSeconds,
						FrmLen,
						pFrms);

				pFrms = pFrms->pNext;
				n++;
			}
#if 0
			//send by eth0
			MuTesterSystem.Device.Eth0.Write ( g_hEthDev[0], pFrmHead);

			// get n buffers
			pNewBuffer = pop_n_queue( &(user_net_config_info[0].xmt_buffers_queue), n );

			// Add buffers to eth1
			MuTesterSystem.Device.Eth1.Read( g_hEthDev[1], pNewBuffer );
#else
			//send by exEth
			MuTesterSystem.Device.exEth.PushUnprocessElem(&g_ExEthXmtQueue, pFrmHead);

			// get n buffers
			pNewBuffer = MuTesterSystem.Device.exEth.PopProcessedElem( &g_ExEthXmtQueue, n );

			// Add buffers to eth1
			MuTesterSystem.Device.Eth1.Read( g_hEthDev[1], pNewBuffer );
#endif
			break;

		case ADI_ETHER_EVENT_FRAME_XMIT:

			push_queue( &(user_net_config_info[1].xmt_buffers_queue), pFrmHead );

			break;

		case ADI_ETHER_EVENT_INTERRUPT:
			break;

		case ADI_ETHER_EVENT_PHY_INTERRUPT:
			break;
	}

	//sti(int_sts);
	EXIT_CRITICAL_REGION();

}

/**************Ex eth********************/



/*
 * DM9000A IRQ ISR
 */
void DM9000A_ISR1(ADI_GPIO_PIN_INTERRUPT const ePinInt,const uint32_t event, void *pArg)
{
	QueueItem* pItem = NULL;
	/* turn off the INT to prevent INIT err */
//	WriteReg(IMR, 0x80);

	if( 0 == PendSmcSem() )
	{
		return;
	}

	uint16_t status = ReadReg( ISR );

	if( status & 0x01 )//接收包中断
	{
		WriteReg( ISR, 0x01 );//清除中断
//		WriteReg( ISR, 0x01 );//清除中断

		adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

		ENTER_CRITICAL_REGION();

		pItem =  Queue_pop_unused_buf( &g_ExEthRecvQueue );

		if( pItem != NULL )
		{
			MDMA2_Param = (void*)pItem->Data;
	//		Process_DM9000A_Recv( pItem->Data);
		}

		EXIT_CRITICAL_REGION();

	}

	if(status & 0x02)//发送包中断
	{
		WriteReg( ISR, 0x02 );//清除中断
	}

	PostSmcSem();

}

void DM9000A_ISR(ADI_GPIO_PIN_INTERRUPT const ePinInt,const uint32_t event, void *pArg)
{

	/* turn off the INT to prevent INIT err */
//	WriteReg(IMR, 0x80);

	//WriteReg( ISR, 0x01 );//清除中断
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

	//Push DM9000a INT Event Queue
//	ENTER_CRITICAL_REGION();
	En_DM9000A_INT_EVENT_Queue( &g_Dm9000aIntEventQueue, 1);
//	EXIT_CRITICAL_REGION();
}




void Task_exEth_Tx_Rx( void *p_arg )
{
	//tx mem
	MuTesterSystem.Device.exEth.InitExEthQueue(&g_ExEthXmtQueue,
			user_net_config_info[2].xmt_buffers_queue.pQueueHead);

	clear_queue(&user_net_config_info[2].xmt_buffers_queue);

	//rx mem
	Queue_init( &g_ExEthRecvQueue );
	Init_DM9000A_INT_EVENT_Queue( &g_Dm9000aIntEventQueue );

	//dev
	MuTesterSystem.Device.exEth.InitExEthnet();
//	MuTesterSystem.Device.exEth.RegisterMACIntCallback(DM9000A_ISR );
//	MuTesterSystem.Device.exEth.EnableMACIntInterrupt(true);

}
