/*
 * task_systime.c
 *
 *  Created on: 2015-3-11
 *      Author: Administrator
 */

#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"

#include "queue.h"
#include "arith.h"
#include "servo.h"

#include "IEC61850_9_2.h"


AUXI_SNAPSHOT_TM_QUEUE  g_Eth0AuxiTMQueue;
AUXI_SNAPSHOT_TM_QUEUE  g_Eth1AuxiTMQueue;

typedef struct sTASK_SYSTIME_PARA
{
	bool bIsFirstInterrupt;
	bool bTimingTestStarted;
	bool bIsDoTimingTest;

	bool bSysTmIsSynced;
	bool bSysTmSyncChanged;
	bool bSetFirstAlarm; // set the  first target trigger time

	bool bPPSIsRunning; //PPS是否输出

	TimeInternal AlarmStartTime;

	int nAlarmIdx; //
	int  nErrorTiming;  // 系统时钟对时误差
	int  nResultTimingTest ;// 对时测试的结果
}TASK_SYSTIME_PARA;

TASK_SYSTIME_PARA g_TaskSysTimeParas[MAX_NETWORK_IF] = {
		{true, false, false, false, false, false, false,{0}, 0, 0, 0},
		{true, false, false, false, false, false, false,{0}, 0, 0, 0},
};

#if 0

static int handle_systime_calibration(	ADI_ETHER_HANDLE phDevice,
		const TimeInternal *pAuxiTimeStamp,
		bool bTimingTestStarted
		)
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	TimeInternal tmInternal = {0,0};
	double dt,deltaNanoSec = 0;
	unsigned int addend = 0;
	int adjAddend = 0;

	tmInternal.nanoseconds = pAuxiTimeStamp->nanoseconds ;


	if( tmInternal.nanoseconds > (NANO_SECOND_UNIT/2) )
	{
		tmInternal.nanoseconds =  tmInternal.nanoseconds - NANO_SECOND_UNIT ;
	}

	//Fine Correction
	deltaNanoSec = runPIservo ( &PI, tmInternal.nanoseconds );
	dt = PI.dt;
	if(bTimingTestStarted)
	{
		dt= 2*PI.dt;
	}
	//
	adjAddend = calcAdjAddend ( ADDEND_VAL, deltaNanoSec, dt );//
	//
	addend = ADDEND_VAL + adjAddend;

	pEmacRegs->EMAC_TM_ADDEND =  addend;

	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSADDREG;


//	DEBUG_PRINT("aux:< %10d.%-9d > int:(%10d %-9d) adj: %d \n\n", pAuxiTimeStamp->seconds, pAuxiTimeStamp->nanoseconds, tmInternal.seconds, tmInternal.nanoseconds, adjAddend);


	return tmInternal.nanoseconds;
}
#endif

//
int handle_systime_calibration(	ADI_ETHER_HANDLE phDevice,
		const TimeInternal *pAuxiTimeStamp,
		TASK_SYSTIME_PARA* pTaskPara)
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	TimeInternal tmInternal = {0,0};

	double dt,deltaNanoSec = 0;

	unsigned int addend = 0;

	int adjAddend = 0;


	tmInternal.nanoseconds = pAuxiTimeStamp->nanoseconds ;

	/* the First time for external trigger PPS arrived,
	 * reInit the system time and output system PPS
	*/

//	if(pTaskPara->bIsFirstInterrupt )
//	{
//		/* reset the system time should be done before the SetFlexiblePPSOutput,
//		 * for the current time may exceed the startTime,
//		 * then cause a Time Stamp Target Time Programming Error.
//		 * And this situation is testified in our TESTS.
//			by wjm@2014-8-16 AM9:30
//		 * */
//		//reset the system time
//		ResetSysTime( phDevice );
//
////		DEBUG_PRINT("aux:< %10d.%-9d >  sys:(reset)\n\n",pAuxiTimeStamp->seconds,  pAuxiTimeStamp->nanoseconds );
//
//		SetPtpPPSOut(phDevice, pAuxiTimeStamp->seconds+2, 0);
//
//		pTaskPara->bIsFirstInterrupt = false;
//
//	}
//	else
//	{
		//

		if( tmInternal.nanoseconds > (NANO_SECOND_UNIT/2) )
		{
			tmInternal.nanoseconds =  tmInternal.nanoseconds - NANO_SECOND_UNIT ;
		}

		//Fine Correction
		deltaNanoSec = runPIservo ( &PI, tmInternal.nanoseconds );
		dt = PI.dt;
		if(pTaskPara->bTimingTestStarted)
		{
			dt= 2*PI.dt;
		}
		//
		adjAddend = calcAdjAddend ( ADDEND_VAL, deltaNanoSec, dt );//
		//
		addend = ADDEND_VAL + adjAddend;

		pEmacRegs->EMAC_TM_ADDEND =  addend;

		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSADDREG;

//		DEBUG_PRINT("aux:< %10d.%-9d > int:(%10d %-9d) adj: %d \n\n", pAuxiTimeStamp->seconds, pAuxiTimeStamp->nanoseconds, tmInternal.seconds, tmInternal.nanoseconds, adjAddend);
//	}//

	return tmInternal.nanoseconds;
}

int handle_timing_test(const TimeInternal *pAuxiTimeStamp)
{
	TimeInternal tmInternal = {0,0};

	tmInternal.nanoseconds = pAuxiTimeStamp->nanoseconds ;

	if( tmInternal.nanoseconds > (NANO_SECOND_UNIT/2) )
	{
		tmInternal.nanoseconds =  tmInternal.nanoseconds - NANO_SECOND_UNIT ;
	}

	return tmInternal.nanoseconds;
}

static bool SysTmIsSynchronizted(int nanoseconds)
{
	if( nanoseconds < 100 && nanoseconds > -100 )
		return true;
	return false;
}

static int handle_auxiliary_tm_interrupt(	void*pArg1, void* pArg2)
{
	ADI_ETHER_HANDLE phDevice 			= pArg1;
	ADI_EMAC_DEVICE    *const  pDev     = ( ADI_EMAC_DEVICE * ) pArg1;
	ADI_EMAC_REGISTERS *const pEmacRegs = pDev->pEMAC_REGS;
	TimeInternal *pAuxiTimeStamp = (TimeInternal *)pArg2;// time stamp
	TASK_SYSTIME_PARA* pTaskPara = NULL;

	bool bRet;

	if( phDevice == g_hEthDev[0] )
	{
		pTaskPara = &g_TaskSysTimeParas[0];
		pIEC61850_9_2->pASDU_info[0].SmpCnt = 3999;
	}

	else if( phDevice == g_hEthDev[1] )
	{
		pTaskPara = &g_TaskSysTimeParas[1];
	}
	else
	{
		return 0;
	}


	if(pTaskPara->bIsDoTimingTest == false)
	{
		pTaskPara->nErrorTiming = handle_systime_calibration(phDevice,
				pAuxiTimeStamp,
				pTaskPara);
#if 1
		bRet = SysTmIsSynchronizted( pTaskPara->nErrorTiming );

		if(pTaskPara->bSysTmIsSynced != bRet)
		{
			pTaskPara->bSysTmSyncChanged = true;
			pTaskPara->bSysTmIsSynced = bRet;
			pTaskPara->bSetFirstAlarm = false;
		}
#endif

#if 1
		if(!pTaskPara->bPPSIsRunning)
		{
			SetPtpPPSOut(phDevice, pAuxiTimeStamp->seconds+2, 0);
			pTaskPara->bPPSIsRunning = true;
		}
#endif
		// set the  first Trigger time
#if 0

		if ( (!pTaskPara->bSetFirstAlarm ) && (pTaskPara->bSysTmIsSynced) )
		{
			pTaskPara->bSetFirstAlarm = true;

			pTaskPara->AlarmStartTime.nanoseconds = pAuxiTimeStamp->nanoseconds;
			pTaskPara->AlarmStartTime.seconds     = pAuxiTimeStamp->seconds;

			if( pTaskPara->AlarmStartTime.nanoseconds < (NANO_SECOND_UNIT/2) )
			{
				pTaskPara->AlarmStartTime.seconds += 1;
				pTaskPara->AlarmStartTime.nanoseconds = 0;
			}
			else
			{
				pTaskPara->AlarmStartTime.seconds += 2;
				pTaskPara->AlarmStartTime.nanoseconds = 0;
			}

			SetTrigerTimeofAuxiInCtrlPPS( g_hDev[0], &(pTaskPara->AlarmStartTime) );

		}
#endif
	}
	else
	{
		pTaskPara->nResultTimingTest = handle_timing_test( pAuxiTimeStamp );

	}


	return 1;
}



//static void Eth0AuxiTMTriggerHandler(void* pArg1)
//{
//	if( EnQueue( &g_Eth0AuxiTMQueue, pArg1 ) != Q_OK )
//	{
//		DEBUG_STATEMENT("EnQueue: Eth0 AuxiSnapshotTMQueue Failed!\n\n ");
//	}
//}
//
//static void Eth1AuxiTMTriggerHandler(void* pArg1)
//{
//	if( EnQueue( &g_Eth1AuxiTMQueue, pArg1 ) != Q_OK )
//	{
//		DEBUG_STATEMENT("EnQueue: Eth1 AuxiSnapshotTMQueue Failed!\n\n ");
//	}
//}

// 这个函数 会被 两个网口调用，注意 不能有静态变量
void TargetTimeTriggerInterruptHandler(	ADI_ETHER_HANDLE phDevice )
{
#if 1
//	static bool bFirstTriggered = true;
//	static int idx = 0;
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;

	TASK_SYSTIME_PARA* pTaskPara = NULL;

	bool bRet;

	if( phDevice == g_hEthDev[0] )
	{
		pTaskPara = &g_TaskSysTimeParas[0];
	}
	else if( phDevice == g_hEthDev[1] )
	{
		pTaskPara = &g_TaskSysTimeParas[1];
	}
	else
	{
		return ;
	}

	StartTransmit( phDevice );

	pTaskPara->nAlarmIdx++;

	if(pTaskPara->nAlarmIdx % 4000 != 0)
	{
		pTaskPara->AlarmStartTime.nanoseconds = pTaskPara->nAlarmIdx *250000;
	}
	else
	{
		pTaskPara->nAlarmIdx = 0;
		pTaskPara->AlarmStartTime.seconds += 1;
		pTaskPara->AlarmStartTime.nanoseconds = 0;
	}


	SetTrigerTimeofAuxiInCtrlPPS(phDevice, &pTaskPara->AlarmStartTime);

#else
	EnableCoreTimer(true);
#endif

}


void Task_SystemTime0( void* p_arg )
{
//	OS_ERR osErr;

	int QRet;
	int nRet;
	bool bRet = false;
	TIME_STAMP_TYPE SnapshotTMType;

	InitQueue( &g_Eth0AuxiTMQueue );

	MuTesterSystem.Device.Eth0.EnableAuxiTimeStamped( g_hEthDev[0] );

	MuTesterSystem.Device.SysTime0.InitSystemTime( g_hEthDev[0] );
	MuTesterSystem.Device.SysTime0.SetAuxiTMTriggerHandler( g_hEthDev[0], handle_auxiliary_tm_interrupt );
//	MuTesterSystem.Device.SysTime0.SetTargetTMTriggerHandler( g_hEthDev[0], TargetTimeTriggerInterruptHandler );
	MuTesterSystem.Device.SysTime0.EnableTimeStampAuxinInterrupt( g_hEthDev[0] );

//	while(1)
//	{
//		QRet = DeQueue ( &g_Eth0AuxiTMQueue, &SnapshotTMType );
//
//		if(Q_OK == QRet)
//		{
//			if(SnapshotTMType.bIsDoTimingTest == false)
//			{
//
//				nRet = handle_systime_calibration(g_hEthDev[0],
//						&SnapshotTMType.SnapshotTm,
//						SnapshotTMType.bTimingTestStarted);
//
////				bRet = SysTmIsSynchronizted(nRet);
//			}
//		}
//		else
//		{
//			OSTimeDly(10, OS_OPT_TIME_DLY, &osErr);
//			if(osErr != OS_ERR_NONE)
//			{
//				DEBUG_PRINT("Task_SystemTime0 : OSTimeDly Error:%d \n\n", osErr);
//
//			}
//		}
//
//	}

//	OSTaskDel((OS_TCB*)0, &osErr);

}

void Task_SystemTime1( void* p_arg )
{
//	OS_ERR osErr;

	int QRet;
	int nRet;
	bool bRet = false;
	TIME_STAMP_TYPE SnapshotTMType;

	InitQueue( &g_Eth1AuxiTMQueue );

	MuTesterSystem.Device.Eth1.EnableAuxiTimeStamped( g_hEthDev[1] );

	MuTesterSystem.Device.SysTime1.InitSystemTime( g_hEthDev[1] );
	MuTesterSystem.Device.SysTime1.SetAuxiTMTriggerHandler( g_hEthDev[1], handle_auxiliary_tm_interrupt );
	MuTesterSystem.Device.SysTime1.EnableTimeStampAuxinInterrupt( g_hEthDev[1] );

//	while(1)
//	{
//		QRet = DeQueue ( &g_Eth1AuxiTMQueue, &SnapshotTMType );
//
//		if(Q_OK == QRet)
//		{
//			if(SnapshotTMType.bIsDoTimingTest == false)
//			{
//
//				nRet = handle_systime_calibration(g_hEthDev[1],
//						&SnapshotTMType.SnapshotTm,
//						SnapshotTMType.bTimingTestStarted);
//
////				bRet = SysTmIsSynchronizted(nRet);
//			}
//		}
//		else
//		{
//			OSTimeDly(10, OS_OPT_TIME_DLY, &osErr);
//			if(osErr != OS_ERR_NONE)
//			{
//				DEBUG_PRINT("Task_SystemTime1 : OSTimeDly Error:%d \n\n", osErr);
//
//			}
//		}
//	}

//	OSTaskDel((OS_TCB*)0, &osErr);
}
