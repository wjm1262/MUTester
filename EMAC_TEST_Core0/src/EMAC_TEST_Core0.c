/*****************************************************************************
  * EMAC_TEST_Core0.c
 *****************************************************************************/

#define ADI_DEBUG  (1)

#include "EMAC_TEST_Core0.h"
//#include "xl-6004_forward_protocol.h"
#include "comm_pc_protocol.h"
#include "msg.h"
#include "comm_cmd_process.h"

#include <stdio.h>
#include <stddef.h>
#include <adi_types.h>

#include <ccblkfn.h>


#include "post_debug.h"

#include "myapp_cfg.h"
#include "sys.h"
#include "task.h"
#include "adi_initialize.h"

/*
 * Init FT3 test
 * user only need invoke this function in INIT code, then the FT3 send process is running in Timer6.
 */
void Init_FT3_Test(unsigned char FT3_heap_id);


char VersionString[128] = "Version 1.0.0. ";
char VerDescripString[64] = "for BeiJing SiFang. ";




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
			DEBUG_PRINT("eth0:%d--%d\n\n" , PreSmpCnt0, CurSmpCnt);
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
			DEBUG_PRINT("eth1:%d ..%d\n\n" , PreSmpCnt1, CurSmpCnt);
		}
		PreSmpCnt1 = CurSmpCnt;
	}
	else
	{
		PreSmpCnt1 = CurSmpCnt;
	}
}
void Timer5_Event_Handler(void *pCBParam, uint32_t Event, void *pArg)
{


//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
}


int main(void)
{
	int32_t result;
	ADI_ETHER_BUFFER* pXmtBuf = NULL;

#if USE_OS
	OS_ERR  osErr;
#endif

	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */
	adi_initComponents();
	
	/**
	 * The default startup code does not include any functionality to allow
	 * core 0 to enable core 1. A convenient way to enable core 1 is to use the
	 * 'adi_core_1_enable' function. 
	 */
//	adi_core_1_enable();

	/* Begin adding your custom code here */

	//global initialization
	MuTesterSystem.Initialize();
	g_rtParams.U8Parameter[U8PARA_NETIN1_TRANSTOPC] = 1;
	g_rtParams.U8Parameter[U8PARA_NETIN2_TRANSTOPC] = 1;
	g_rtParams.U8Parameter[U8PARA_FT3_SEND2] =1;


	// print info
	int i;

	strcat(VersionString, VerDescripString );
	i = strlen(VersionString);
	snprintf(VersionString + i, 100, " Built on %s, at %s, line:%d ", __DATE__, __TIME__, __LINE__);
	DEBUG_PRINT ( " %s. \n\n" , VersionString);


	//get eth buffer
	Alloc_EthMem();


	//Gemac
	g_hEthDev[0] = MuTesterSystem.Device.Eth0.Open();
	g_hEthDev[1] = MuTesterSystem.Device.Eth1.Open();

	MuTesterSystem.Device.Eth0.Configure( g_hEthDev[0] );
	MuTesterSystem.Device.Eth0.SetCallbakFn( g_hEthDev[0], (void*)Ethernet0_Callback );

	MuTesterSystem.Device.Eth1.Configure( g_hEthDev[1] );
	MuTesterSystem.Device.Eth1.SetCallbakFn( g_hEthDev[1], (void*)Ethernet1_Callback );

	MuTesterSystem.Device.Eth0.hDev = g_hEthDev[0];
	MuTesterSystem.Device.Eth1.hDev = g_hEthDev[1];


	Task_exEth_Tx_Rx(NULL);

	Task_Eth0_Tx(NULL);
	Task_Eth1_Tx(NULL);
	Task_Eth0_Rx(NULL);
	Task_Eth1_Rx(NULL);

	//
//	Init_FT3_Test(1);


	Task_SystemTime0(NULL);
	Task_SystemTime1(NULL);



//	Task_AD7608( NULL );

	///
	Init_GP_Timer5();

//	SetTimerEventHandler(Timer5_Event_Handler);


	LoopQueueItem* pRecvItem;
	uint8_t* pForwardFrm = NULL;
	uint8_t* pRecvFrm = NULL;
	uint16_t NoBytes;

	while(1)
	{

		pRecvItem = (LoopQueueItem*)MuTesterSystem.Device.exEth.EthRecv();
		if(pRecvItem)
		{
			NoBytes  = pRecvItem->Size;
			pRecvFrm = pRecvItem->Data;

			Comm_processCmd( pRecvFrm, NoBytes );

		}

		//
		pXmtBuf = MuTesterSystem.Device.exEth.PopUnprocessElem(&g_ExEthXmtQueueEth0 );
		if(pXmtBuf)
		{
			pForwardFrm = (uint8_t*)pXmtBuf->Data +2;
			MuTesterSystem.Device.exEth.EthSend ( pForwardFrm, pXmtBuf->ElementCount - 2);
		}

		pXmtBuf = MuTesterSystem.Device.exEth.PopUnprocessElem(&g_ExEthXmtQueueEth1 );
		if(pXmtBuf)
		{
			pForwardFrm = (uint8_t*)pXmtBuf->Data +2;
			MuTesterSystem.Device.exEth.EthSend ( pForwardFrm, pXmtBuf->ElementCount - 2);
		}

		pXmtBuf = MuTesterSystem.Device.exEth.PopUnprocessElem(&g_ExEthXmtQueueAD );
		if(pXmtBuf)
		{
			pForwardFrm = (uint8_t*)pXmtBuf->Data +2;
			MuTesterSystem.Device.exEth.EthSend ( pForwardFrm, pXmtBuf->ElementCount - 2);
		}

//		StandardSmpDataFormatConverter();

	}//while


	return 0;
}

