/*****************************************************************************
  * EMAC_TEST_Core0.c
 *****************************************************************************/

#define ADI_DEBUG  (1)

#include "EMAC_TEST_Core0.h"
#include "xl-6004_forward_protocol.h"
#include "mutester_comm_protocol.h"
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

char VersionString[128] = "Version 1.0.0. ";
char VerDescripString[64] = "with Forward SMV frame. ";

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
#if USE_OS
OS_TCB  App_Satrt_TaskTCB;
static  CPU_STK App_Satrt_TaskStack[APP_START_TASK_STACK_SIZE];

OS_TCB  Eth0_Tx_TaskTCB;
static  CPU_STK Eth0_Tx_TaskStack[Eth0_Tx_TASK_STACK_SIZE];

OS_TCB  Eth0_Rx_TaskTCB;
static  CPU_STK Eth0_Rx_TaskStack[Eth0_Rx_TASK_STACK_SIZE];

OS_TCB  Eth1_Tx_TaskTCB;
static  CPU_STK Eth1_Tx_TaskStack[Eth1_Tx_TASK_STACK_SIZE];

OS_TCB  Eth1_Rx_TaskTCB;
static  CPU_STK Eth1_Rx_TaskStack[Eth1_Rx_TASK_STACK_SIZE];

OS_TCB  SysTime0_TaskTCB;
static  CPU_STK SysTime0_TaskStack[SYSTIME_TASK_STACK_SIZE];

OS_TCB  SysTime1_TaskTCB;
static  CPU_STK SysTime1_TaskStack[SYSTIME_TASK_STACK_SIZE];

OS_TCB  AD7608_TaskTCB;
static  CPU_STK AD7608_TaskStack[AD_TASK_STACK_SIZE];

///////////////////////////////
extern int32_t adi_OS_Init(void);

#define  SystemDatasBroadcast_PRIO            12 // 统计任务优先级最低，我这里是12，已经低于其他任务的优先级了
#define  SystemDatasBroadcast_STK_SIZE       128 // 任务的堆栈大小，做统计一般够了，统计结果出来后不够再加..
OS_TCB  SystemDatasBroadcast_TCB;		 // 定义统计任务的TCB
CPU_STK SystemDatasBroadcast_STK [SystemDatasBroadcast_STK_SIZE];// 开辟数组作为任务栈给任务使用

#endif

//void  SystemDatasBroadcast (void *p_arg)
//{
//  OS_ERR err;
//  CPU_STK_SIZE free,used;
//  (void)p_arg;
//  while(DEF_TRUE)
//  {
//	OSTaskStkChk (&App_Satrt_TaskTCB, &free, &used, &err);//  把统计任务本身的堆栈使用量也打印出来
//								  // 然后从实验结果看看我们设置100字节给它是不是真的合适
//	DEBUG_PRINT("App_Satrt  used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
//
//	OSTaskStkChk (&Eth0_Rx_TaskTCB,&free,&used,&err);
//	DEBUG_PRINT("Eth0_Rx             used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
//
//	OSTaskStkChk (&Eth1_Tx_TaskTCB,&free,&used,&err);
//	DEBUG_PRINT("Eth1_Tx             used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
//
//	OSTaskStkChk (&SysTime0_TaskTCB,&free,&used,&err);
//	DEBUG_PRINT("SysTime0_Task       used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
//
//	OSTaskStkChk (&SysTime1_TaskTCB,&free,&used,&err);
//	DEBUG_PRINT("SysTime1_Task       used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
//
////	OSTaskStkChk (&Calibrate_Process_TCB,&free,&used,&err);
////	DEBUG_PRINT("Calibrate             used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
////
////
////	OSTaskStkChk (&Data_Process_TCB,&free,&used,&err);
////	DEBUG_PRINT("Data_Process          used/free:%d/%d  usage:%%%d\n\n",used,free,(used*100)/(used+free));
//
//	OSTimeDlyHMSM(0,0,5,0,(OS_OPT)OS_OPT_TIME_DLY,(OS_ERR*)&err);
//   }
//}
#if USE_OS
void Task_App_Start(void*p_arg)
{
	OS_ERR  osErr;
//
//#if	OS_CFG_STAT_TASK_EN > 0
//
//	OSStatTaskCPUUsageInit(&osErr);
//
//#endif

//	OSTaskCreate( (OS_TCB     *)&SystemDatasBroadcast_TCB,
//				(CPU_CHAR   *)"SystemDatasBroadcast",
//				(OS_TASK_PTR ) SystemDatasBroadcast,
//				(void       *) 0,
//				(OS_PRIO     ) SystemDatasBroadcast_PRIO,
//				(CPU_STK    *)&SystemDatasBroadcast_STK[0],
//				(CPU_STK_SIZE) SystemDatasBroadcast_STK_SIZE/10,/*栈溢出临界值我设置在栈大小的90%处*/
//				(CPU_STK_SIZE) SystemDatasBroadcast_STK_SIZE,
//				(OS_MSG_QTY  ) 0,
//				(OS_TICK     ) 0,
//				(void       *) 0,
//				(OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//				(OS_ERR     *) &osErr);
//	if(osErr != OS_ERR_NONE)
//	{
//		printf("Error creating application task /n");
//		while(1){ ; }
//	}





//	while(1)
//	{
//		OSTimeDlyHMSM(0,0,1,0,(OS_OPT)OS_OPT_TIME_DLY,(OS_ERR*)&osErr);
//	}

}

#endif

void IntegrityTest0(uint8_t* pForwardFrm)
{
	static int16_t PreSmpCnt0 = -1;
	int16_t CurSmpCnt = 0;

	uint8_t c60_pos0 = 0x4e;
	uint8_t c60_pos1 = 0x4f;


	CurSmpCnt = (*(pForwardFrm +c60_pos0)<<8) + (*(pForwardFrm +c60_pos1));

	if( -1 != PreSmpCnt0 )
	{
		if( (PreSmpCnt0 + 1)%4000 != CurSmpCnt )
		{
			DEBUG_PRINT("%d ... %d.\n\n" , PreSmpCnt0, CurSmpCnt);
		}
		PreSmpCnt0 = CurSmpCnt;
	}
	else
	{
		PreSmpCnt0 = CurSmpCnt;
	}
}

void IntegrityTest1(uint8_t* pForwardFrm)
{
	static int16_t PreSmpCnt1 = -1;
	int16_t CurSmpCnt = 0;

	uint8_t c60_pos0 = 0x4e;
	uint8_t c60_pos1 = 0x4f;


	CurSmpCnt = (*(pForwardFrm +c60_pos0)<<8) + (*(pForwardFrm +c60_pos1));

	if( -1 != PreSmpCnt1 )
	{
		if( (PreSmpCnt1 + 1)%4000 != CurSmpCnt )
		{
			DEBUG_PRINT("%d ... %d.\n\n" , PreSmpCnt1, CurSmpCnt);
		}
		PreSmpCnt1 = CurSmpCnt;
	}
	else
	{
		PreSmpCnt1 = CurSmpCnt;
	}
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
	Task_Eth1_Tx(NULL);
	Task_Eth0_Rx(NULL);
	Task_Eth1_Rx(NULL);


	Task_SystemTime0(NULL);
	Task_SystemTime1(NULL);

	//
//	Init_IEC_9_2();

//	Task_AD7608( NULL );

	uint8_t* pForwardFrm = NULL;
	uint8_t* pRecvFrm = NULL;

	while(1)
	{
		//
		pXmtBuf = MuTesterSystem.Device.exEth.PopUnprocessElem(&g_ExEthXmtQueue );
		if(pXmtBuf)
		{
			pForwardFrm = (uint8_t*)pXmtBuf->Data +2;
#if 0
			if( (*(pForwardFrm +0x32) == 0x40 ) && (*(pForwardFrm +0x33) == 0x01) )
			{
				IntegrityTest0( pForwardFrm);
			}
			else if( (*(pForwardFrm +0x32) == 0x40 ) && (*(pForwardFrm +0x33) == 0x02) )
			{
				IntegrityTest1( pForwardFrm);
			}
#endif
			MuTesterSystem.Device.exEth.EthSend ( pForwardFrm, pXmtBuf->ElementCount - 2);

//			DEBUG_STATEMENT("send ok\n\n");

		}

		pRecvFrm = (uint8_t*)MuTesterSystem.Device.exEth.EthRecv();
		if(pRecvFrm)
		{
			FORWARD_ETHER_FRAME *pEtheCMD_Frame = ( FORWARD_ETHER_FRAME * )pRecvFrm;

//			Comm_processCmd( (uint8_t*)pEtheCMD_Frame +2, pEtheCMD_Frame->NoBytes );

			//DEBUG_PRINT ( " len:%d,   \n\n" , pEtheCMD_Frame->NoBytes );
			DEBUG_STATEMENT("recv ok.\n\n");

		}

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


//	Task_SystemTime0( NULL );
//	Task_SystemTime1( NULL );

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

