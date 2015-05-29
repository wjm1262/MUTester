/*
 * dev_ptp_engine.c
 *
 *  Created on: 2015-3-11
 *      Author: Administrator
 */


#include "sys.h"
#include "dev_ptp_engine.h"


static int InitSystemTime (void* hEthDev)
{
	ADI_EMAC_DEVICE* pDev = ( ADI_EMAC_DEVICE * ) hEthDev;

	int ret = 0;

	//init sys time
	InitSysTimeRegs( hEthDev );

	if ( pDev->TxTimeStamped || pDev->RxTimeStamped )
	{
		ProgrammingTimestamp(hEthDev);
	}

//
//	ProgrammingSysTimeFineCorrection( hEthDev );//注意：如果调用了本函数，即使没有调用SetFixedPPSOutput，也会输出PPS，
//												//PPS脉冲宽度等于SCLKx的周期，因为只要Enable PTP Module + 系统时间在运行，
//											//就相当于SetFixedPPSOutput。
//	//
//	ret = SetPtpPPSOut(hEthDev, 2, 0);//输出FlexiblePPS

//	SetFixedPPSOutput(hEthDev);

	return ret;

}

static void EnableTimeStampAuxinInterrupt(void* hEthDev)
{
	Enable_Time_Stamp_Auxin_Interrupt( hEthDev);
}

static void SetAuxiTMTriggerHandler(void* hEthDev, void* handler )
{
	set_auxiTM_trigger_handler(hEthDev, handler);
}

static void SetTargetTMTriggerHandler(void* hEthDev, void* handler )
{
	set_targetTM_trigger_handler(hEthDev, handler);
}

void RegisterSysTimeModual( void )
{
	MuTesterSystem.Device.SysTime0.InitSystemTime = InitSystemTime;
	MuTesterSystem.Device.SysTime0.EnableTimeStampAuxinInterrupt = EnableTimeStampAuxinInterrupt;
	MuTesterSystem.Device.SysTime0.SetAuxiTMTriggerHandler = SetAuxiTMTriggerHandler;
	MuTesterSystem.Device.SysTime0.SetTargetTMTriggerHandler = SetTargetTMTriggerHandler;


	MuTesterSystem.Device.SysTime1.InitSystemTime = InitSystemTime;
	MuTesterSystem.Device.SysTime1.EnableTimeStampAuxinInterrupt = EnableTimeStampAuxinInterrupt;
	MuTesterSystem.Device.SysTime1.SetAuxiTMTriggerHandler = SetAuxiTMTriggerHandler;

}

