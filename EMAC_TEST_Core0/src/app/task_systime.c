/*
 * task_systime.c
 *
 *  Created on: 2015-3-11
 *      Author: Administrator
 */

#include "EMAC_TEST_Core0.h"

#include "task.h"
#include "sys.h"

#include "arith.h"
#include "servo.h"


#include "comm_pc_protocol.h"
#include "msg.h"

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

extern int OutputStandardADFrm( const STAND_SAMP_TYPE*  pStandSmpData);

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

	}
	else if( phDevice == g_hEthDev[1] )
	{
		pTaskPara = &g_TaskSysTimeParas[1];
	}
	else
	{
		return 0;
	}



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
	// set the  first Trigger time
	// 因为PPS输出也需要设置target time regs，为了避免 Time Stamp Target Time Programming Error，在输出PPS后才开始第一次定时。

	if ( (pTaskPara->bPPSIsRunning)&&(!pTaskPara->bSetFirstAlarm ) && (pTaskPara->bSysTmIsSynced) )
	{
		pTaskPara->bSetFirstAlarm = true;

		pTaskPara->AlarmStartTime.nanoseconds = 0;
		pTaskPara->AlarmStartTime.seconds     = pAuxiTimeStamp->seconds + 3;


		SetAlarmTrigerTime( g_hEthDev[0], &(pTaskPara->AlarmStartTime) );

	}
#endif

#if 1
	if(!pTaskPara->bPPSIsRunning && pTaskPara->bSysTmIsSynced )
	{

		//
		ProgrammingSysTimeFineCorrection( phDevice );//注意：如果调用了本函数，即使没有调用SetFixedPPSOutput，也会输出PPS，
													//PPS脉冲宽度等于SCLKx的周期，因为只要Enable PTP Module + 系统时间在运行，
												//就相当于SetFixedPPSOutput。

		SetPtpPPSOut(phDevice, pAuxiTimeStamp->seconds + 2, 0);
		pTaskPara->bPPSIsRunning = true;


	}
#endif


	return 1;
}



// 这个函数 会被 两个网口调用，注意 不能有静态变量
void TargetTimeTriggerInterruptHandler(	ADI_ETHER_HANDLE phDevice )
{
#if 1

	STAND_SAMP_TYPE StandardSmpData={0};

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

//	StartTransmit( phDevice );

	//标准数据组帧发送
	StandardSmpData.sampCnt = pTaskPara->nAlarmIdx;
	OutputStandardADFrm( &StandardSmpData );

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


	SetAlarmTrigerTime(phDevice, &pTaskPara->AlarmStartTime);

#else
	EnableCoreTimer(true);
#endif

}


void Task_SystemTime0( void* p_arg )
{
//	OS_ERR osErr;

	int nRet=0;


	MuTesterSystem.Device.Eth0.EnableAuxiTimeStamped( g_hEthDev[0] );

	nRet = MuTesterSystem.Device.SysTime0.InitSystemTime( g_hEthDev[0] );
	if(nRet)
	{
		g_TaskSysTimeParas[0].bPPSIsRunning = true;
	}

	MuTesterSystem.Device.SysTime0.SetAuxiTMTriggerHandler( g_hEthDev[0], handle_auxiliary_tm_interrupt );
	MuTesterSystem.Device.SysTime0.SetTargetTMTriggerHandler( g_hEthDev[0], TargetTimeTriggerInterruptHandler );
	MuTesterSystem.Device.SysTime0.EnableTimeStampAuxinInterrupt( g_hEthDev[0] );


//	OSTaskDel((OS_TCB*)0, &osErr);

}

void Task_SystemTime1( void* p_arg )
{
//	OS_ERR osErr;

	int nRet=0;


	MuTesterSystem.Device.Eth1.EnableAuxiTimeStamped( g_hEthDev[1] );

	nRet = MuTesterSystem.Device.SysTime1.InitSystemTime( g_hEthDev[1] );
	if(nRet)
	{
		g_TaskSysTimeParas[1].bPPSIsRunning = true;
	}

	MuTesterSystem.Device.SysTime1.SetAuxiTMTriggerHandler( g_hEthDev[1], handle_auxiliary_tm_interrupt );
	MuTesterSystem.Device.SysTime1.EnableTimeStampAuxinInterrupt( g_hEthDev[1] );

//	OSTaskDel((OS_TCB*)0, &osErr);
}
