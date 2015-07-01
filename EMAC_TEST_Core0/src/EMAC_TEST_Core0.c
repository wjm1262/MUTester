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
char VerDescripString[64] = "timing + forward. ";

//define STACK SIZE
#define APPLICATION_TASK_STACK_SIZE (2048)

#define APP_START_TASK_STACK_SIZE (2048)
#define Eth0_Rx_TASK_STACK_SIZE (2048)
#define Eth0_Tx_TASK_STACK_SIZE (2048)
#define Eth1_Rx_TASK_STACK_SIZE (2048)
#define Eth1_Tx_TASK_STACK_SIZE (2048)
#define SYSTIME_TASK_STACK_SIZE (2048)
#define AD_TASK_STACK_SIZE (2048)

//define PRIO
#define APP_START_TASK_PRIO  (6)
#define Eth0_Rx_TASK_PRIO  (6)
#define Eth0_Tx_TASK_PRIO  (6)
#define Eth1_Rx_TASK_PRIO  (6)
#define Eth1_Tx_TASK_PRIO  (6)
#define SysTime_TASK_PRIO  (6)
#define AD_TASK_PRIO  (6)
///

///



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
	Init_FT3_Test(1);


	Task_SystemTime0(NULL);
	Task_SystemTime1(NULL);



	//
//	Init_IEC_9_2();

	Task_AD7608( NULL );

	///
//	Init_GP_Timer5();

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
			if( ( NoBytes == 512 )|| ( NoBytes == 1541 ))
			{
				//DEBUG_PRINT ( " L:%d,%0X\n\n" , NoBytes, pRecvItem->Size );
				DEBUG_PRINT ( " L:%d,\n\n" , NoBytes );
			}

//			DEBUG_STATEMENT("recv ok.\n\n");
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

#if USE_OS
	//
	result = adi_OS_Init();
	if (result != 0)
	{
		printf("Failed to initialize operating system. \n\n");
		while(1){ ; }
	}

	/* create Application task */
//	OSTaskCreate (
//		&App_Satrt_TaskTCB,                /* Pointer to TCB */
//		"App Start Task",                 /* The Task Name */
//		Task_App_Start,                    /* Function Pointer */
//		(void*) NULL,                       /* Function Argument */
//		APP_START_TASK_PRIO,              /* Application Task Priority */
//		App_Satrt_TaskStack,               /* Stack base Address */
//		APPLICATION_TASK_STACK_SIZE - 1,    /* Stack Limit */
//		APPLICATION_TASK_STACK_SIZE,        /* Stack Size */
//		NULL,                               /* Message Queue Size */
//		1,                                  /* Time Quota */
//		NULL,                               /* External Pointer */
//		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
//		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
//		&osErr                              /* Pointer to error variable */
//		);
//
//	if(osErr != OS_ERR_NONE)
//	{
//	   printf("Error creating application task \n\n");
//	   while(1){ ; }
//	}

#if 1
	/* create Application task */
	OSTaskCreate (
		&Eth1_Tx_TaskTCB,                /* Pointer to TCB */
		"Eth1 Sender Task",                 /* The Task Name */
		Task_Eth1_Tx,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		Eth1_Tx_TASK_PRIO,              /* Application Task Priority */
		Eth1_Tx_TaskStack,               /* Stack base Address */
		Eth1_Tx_TASK_STACK_SIZE-1,    /* Stack Limit */
		Eth1_Tx_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

	if(osErr != OS_ERR_NONE)
	{
	   printf("Error creating application task. \n\n");
	   while(1){ ; }
	}

	/* create Application task */
	OSTaskCreate (
		&Eth0_Rx_TaskTCB,                /* Pointer to TCB */
		"Eth0 Receiver Task",                 /* The Task Name */
		Task_Eth0_Rx,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		Eth0_Rx_TASK_PRIO,              /* Application Task Priority */
		Eth0_Rx_TaskStack,               /* Stack base Address */
		Eth0_Rx_TASK_STACK_SIZE - 1,    /* Stack Limit */
		Eth0_Rx_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

	if(osErr != OS_ERR_NONE)
	{
	   printf("Error creating application task. \n\n");
	   while(1){ ; }
	}

#else

	/* create Application task */
	OSTaskCreate (
		&Eth0_Tx_TaskTCB,                /* Pointer to TCB */
		"Eth0 Sender Task",                 /* The Task Name */
		Task_Eth0_Tx,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		Eth0_Tx_TASK_PRIO,              /* Application Task Priority */
		Eth0_Tx_TaskStack,               /* Stack base Address */
		Eth0_Tx_TASK_STACK_SIZE-1,    /* Stack Limit */
		Eth0_Tx_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

		if(osErr != OS_ERR_NONE)
		{
		   printf("Error creating application task. \n\n");
		   while(1){ ; }
		}

	/* create Application task */
	OSTaskCreate (
		&Eth1_Rx_TaskTCB,                /* Pointer to TCB */
		"Eth1 Receiver Task",                 /* The Task Name */
		Task_Eth1_Rx,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		Eth1_Rx_TASK_PRIO,              /* Application Task Priority */
		Eth1_Rx_TaskStack,               /* Stack base Address */
		Eth1_Rx_TASK_STACK_SIZE - 1,    /* Stack Limit */
		Eth1_Rx_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

	if(osErr != OS_ERR_NONE)
	{
	   printf("Error creating application task. \n\n");
	   while(1){ ; }
	}


#endif




#if 1
	/* create Application task */
	OSTaskCreate (
		&SysTime0_TaskTCB,                /* Pointer to TCB */
		"SysTime0 Task",                 /* The Task Name */
		Task_SystemTime0,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		SysTime_TASK_PRIO,              /* Application Task Priority */
		SysTime0_TaskStack,               /* Stack base Address */
		SYSTIME_TASK_STACK_SIZE - 1,    /* Stack Limit */
		SYSTIME_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

	if(osErr != OS_ERR_NONE)
	{
	   printf("Error creating application task. \n\n");
	   while(1){ ; }
	}


//	/* create Application task */
	OSTaskCreate (
		&SysTime1_TaskTCB,                /* Pointer to TCB */
		"SysTime1 Task",                 /* The Task Name */
		Task_SystemTime1,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		SysTime_TASK_PRIO,              /* Application Task Priority */
		SysTime1_TaskStack,               /* Stack base Address */
		SYSTIME_TASK_STACK_SIZE - 1,    /* Stack Limit */
		SYSTIME_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

	if(osErr != OS_ERR_NONE)
	{
	   printf("Error creating application task. \n\n");
	   while(1){ ; }
	}

#endif

#if 0
	//
	/* create Application task */
	OSTaskCreate (
		&AD7608_TaskTCB,                /* Pointer to TCB */
		"AD7608 Task",                 /* The Task Name */
		Task_AD7608,                    /* Function Pointer */
		(void*) NULL,                       /* Function Argument */
		AD_TASK_PRIO,              /* Application Task Priority */
		AD7608_TaskStack,               /* Stack base Address */
		AD_TASK_STACK_SIZE - 1,    /* Stack Limit */
		AD_TASK_STACK_SIZE,        /* Stack Size */
		NULL,                               /* Message Queue Size */
		1,                                  /* Time Quota */
		NULL,                               /* External Pointer */
		OS_OPT_TASK_STK_CHK |               /* Allow Stack Check */
		OS_OPT_TASK_STK_CLR,                /* Clear the stack */
		&osErr                              /* Pointer to error variable */
		);

	if(osErr != OS_ERR_NONE)
	{
	   printf("Error creating application task. \n\n");
	   while(1){ ; }
	}

#endif

	/* start the OS */
	OSStart(&osErr);

	if(osErr != OS_ERR_NONE)
	{
		printf("failed to start OS. \n\n");
		while(1){ ; }
	}

#endif

	return 0;
}

