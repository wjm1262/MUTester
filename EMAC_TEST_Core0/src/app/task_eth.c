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

//#include "xl-6004_forward_protocol.h"
#include "mutester_comm_protocol.h"
#include "msg.h"

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


static void IntegrityTest0(uint8_t* pForwardFrm)
{
	static int16_t PreSmpCnt0 = -1;
	int16_t CurSmpCnt = 0;
	uint8_t u0, u1;

	uint8_t c60_pos0 = 0x4e;
	uint8_t c60_pos1 = 0x4f;
//	uint8_t c60_pos0 = 0x54;
//	uint8_t c60_pos1 = 0x55;

	u0 = *(pForwardFrm +c60_pos0);
	u1 = *(pForwardFrm +c60_pos1);

	CurSmpCnt = (u0<<8) + u1;

	if( -1 != PreSmpCnt0 )
	{
		if( (PreSmpCnt0 + 1)%4000 != CurSmpCnt )
		{
			DEBUG_PRINT("eth ca0:%d--%d\n\n" , PreSmpCnt0, CurSmpCnt);
		}
		PreSmpCnt0 = CurSmpCnt;
	}
	else
	{
		PreSmpCnt0 = CurSmpCnt;
	}
}

static void IntegrityTest1(uint8_t* pForwardFrm)
{
	static int16_t PreSmpCnt1 = -1;
	int16_t CurSmpCnt = 0;

	uint8_t c60_pos0 = 0x4e;
	uint8_t c60_pos1 = 0x4f;

//	uint8_t c60_pos0 = 0x54;
//	uint8_t c60_pos1 = 0x55;
	CurSmpCnt = (*(pForwardFrm +c60_pos0)<<8) + (*(pForwardFrm +c60_pos1));

	if( -1 != PreSmpCnt1 )
	{
		if( (PreSmpCnt1 + 1)%4000 != CurSmpCnt )
		{
			DEBUG_PRINT("eth ca1:%d ..%d\n\n" , PreSmpCnt1, CurSmpCnt);
		}
		PreSmpCnt1 = CurSmpCnt;
	}
	else
	{
		PreSmpCnt1 = CurSmpCnt;
	}
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

	int nSeconds0, nNanoSeconds0;
	int nSeconds1, nNanoSeconds1;
	int nSeconds2, nNanoSeconds2;



	//int int_sts = cli();
	ENTER_CRITICAL_REGION();

	uint8_t* pForwardFrm = NULL;

	pFrmHead = pFrms;

	switch ( event )
	{
		case ADI_ETHER_EVENT_FRAME_RCVD:
			if (g_rtParams.U8Parameter[U8PARA_NETIN1_TRANSTOPC] )
			{
				while(pFrms)
				{
					//process frame
					//get time stamp
					pTmBuff = ( RX_TX_TIME_STAMP_BUFFER * ) pFrms;

					unSecond = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampHi ) ;
					nanoSeconds = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampLo ) ;

					FrmLen = pFrms->ProcessedElementCount - 6;
#if 0
					pForwardFrm = (uint8_t*)pFrms->Data +2;
					if( (*(pForwardFrm +0x2e) == 0x40 ) && (*(pForwardFrm +0x2f) == 0x01) )
					{
						IntegrityTest0( pForwardFrm);
					}
					else if( (*(pForwardFrm +0x2e) == 0x40 ) && (*(pForwardFrm +0x2f) == 0x02) )
					{
						IntegrityTest1( pForwardFrm);
					}
					else
					{
						DEBUG_STATEMENT("net0 ERROR AppID\n\n");
					}
#endif

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
				//MuTesterSystem.Device.Eth0.Read( g_hEthDev[0], pNewBuffer );
				adi_ether_GemacRead( g_hEthDev[0], pNewBuffer );
#endif
			}
			else
			{
				// return buffers to eth0
				MuTesterSystem.Device.Eth0.Read( g_hEthDev[0], pFrmHead );
			}
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


//	DEBUG_PRINT("rx %d, %d, %d\n\n", nNanoSeconds1-nNanoSeconds0, nNanoSeconds2-nNanoSeconds1, nNanoSeconds2-nNanoSeconds0);
//	DEBUG_PRINT("rx s:%d, r:%d \n\n", nNanoSeconds1-nNanoSeconds0, nNanoSeconds2-nNanoSeconds1);
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

	uint8_t* pForwardFrm = NULL;

	switch ( event )
	{
		case ADI_ETHER_EVENT_FRAME_RCVD:
			if (g_rtParams.U8Parameter[U8PARA_NETIN2_TRANSTOPC] )
			{
				while(pFrms)
				{
					//process frame
					//get time stamp
					pTmBuff = ( RX_TX_TIME_STAMP_BUFFER * ) pFrms;

					unSecond = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampHi ) ;
					nanoSeconds = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampLo ) ;

					FrmLen = pFrms->ProcessedElementCount - 6;

#if 0
					pForwardFrm = (uint8_t*)pFrms->Data +2;
					if( (*(pForwardFrm +0x2e) == 0x40 ) && (*(pForwardFrm +0x2f) == 0x01) )
					{
						IntegrityTest0( pForwardFrm);
					}
					else if( (*(pForwardFrm +0x2e) == 0x40 ) && (*(pForwardFrm +0x2f) == 0x02) )
					{
						IntegrityTest1( pForwardFrm);
					}
					else
					{
						DEBUG_STATEMENT("net1 ERROR AppID\n\n");
					}
#endif
					PackForwardFrame( TYPE609_CONT_NET_RECV2_DATA, unSecond, nanoSeconds,
										FrmLen,
										pFrms);

					pFrms = pFrms->pNext;
					n++;
				}//while

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

			}
			else
			{
				// return buffers to eth1
				MuTesterSystem.Device.Eth1.Read( g_hEthDev[1], pFrmHead );
			}

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

void Task_exEth_Tx_Rx( void *p_arg )
{
	//tx mem
	MuTesterSystem.Device.exEth.InitExEthQueue(&g_ExEthXmtQueue,
			user_net_config_info[2].xmt_buffers_queue.pQueueHead);

	clear_queue(&user_net_config_info[2].xmt_buffers_queue);

	//dev
	MuTesterSystem.Device.exEth.InitExEthnet(user_net_config_info[2].hwaddr);

//	MuTesterSystem.Device.exEth.EnableMACIntInterrupt(true);

}
