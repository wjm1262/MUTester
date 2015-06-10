/*
 * ptp_engine.h
 *
 *  Created on: 2015-3-11
 *      Author: Administrator
 */

#ifndef PTP_ENGINE_H_
#define PTP_ENGINE_H_

#include <string.h>
#include <stdio.h>
#include <adi_osal.h>
#include <ccblkfn.h>
#include <cdefbf609.h>

#include "dri_ether_header.h"

#include <services/int/adi_int.h>
#include "dri_adi_gemac.h"

#define NANO_SECOND_UNIT (1000000000)
#define PPS_WIDTH_100M  (0x2FAF080)
#define PPS_INTERVAL_100M (0x5F5E100)

#define SCLK0_100M
//#define SCLK0_163840K
//#define SCLK0_80M

#if  defined(SCLK0_100M)

	#define PTP_REF_CLK (50000000)

	#define ADDEND_50M_100M (0x80000000)

	#define ADDEND_50M ADDEND_50M_100M
	#define PPS_WIDTH_50M (PPS_WIDTH_100M*0.5)
	#define PPS_INTERVAL_50M (PPS_INTERVAL_100M*0.5)

	#define ADDEND_VAL (ADDEND_50M)
	#define PPS_WIDTH_VAL PPS_WIDTH_50M
	#define PPS_INTERVAL_VAL PPS_INTERVAL_50M

#elif  defined(SCLK0_163840K)

	#define PTP_REF_CLK (50000000)

	#define ADDEND_100M_163840K (0X9C400000)

	#define ADDEND_100M ADDEND_100M_163840K

	#define ADDEND_50M (ADDEND_100M*0.5)
	#define PPS_WIDTH_50M ( PPS_WIDTH_100M *0.5 )
	#define PPS_INTERVAL_50M  (PPS_INTERVAL_100M*0.5 )

	#define ADDEND_VAL (ADDEND_50M)
	#define PPS_WIDTH_VAL PPS_WIDTH_50M
	#define PPS_INTERVAL_VAL PPS_INTERVAL_50M


#elif  defined(SCLK0_80M)

	#define PTP_REF_CLK (40000000)

	#define ADDEND_40M_80M (0x80000000)

	#define ADDEND_40M ADDEND_40M_80M
	#define PPS_WIDTH_40M (PPS_WIDTH_100M*0.4)
	#define PPS_INTERVAL_40M (PPS_INTERVAL_100M*0.4)

	#define ADDEND_VAL (ADDEND_40M)
	#define PPS_WIDTH_VAL PPS_WIDTH_40M
	#define PPS_INTERVAL_VAL PPS_INTERVAL_40M

#endif

#define _4K_PPS_WIDTH_VAL (PPS_WIDTH_VAL)/4000
#define _4K_PPS_INTERVAL_VAL (PPS_INTERVAL_VAL)/4000

typedef enum PPS_TYPE
{
	PULSE_SINGLE = 0x1, //0001
	PULSE_TRAIN = 0x2, //0010,start pulse train
}PPS_TYPE;

typedef struct
{
	int seconds;
	int nanoseconds;
} TimeInternal;


/*  */
void Init_PTPAuxin(void);

void InitSysTimeRegs(ADI_ETHER_HANDLE phDevice );

void set_auxiTM_trigger_handler(void* hDev, void* pfHandler );

void set_targetTM_trigger_handler(void* hDev, void* pfHandler );

void ProgrammingTimestamp(ADI_ETHER_HANDLE phDevice);

void Enable_Time_Stamp_Auxin_Interrupt(ADI_ETHER_HANDLE phDevice);

void SetEth0FlexiblePPSOutput( ADI_ETHER_HANDLE phDevice,
		PPS_TYPE ePPSType,
		int tmStartSec,
		int tmStartNanoSec,
		unsigned int uPPSInterval,
		unsigned int uPPSWidth
		);

void SetEth1FlexiblePPSOutput( ADI_ETHER_HANDLE phDevice,
		PPS_TYPE ePPSType,
		int tmStartSec,
		int tmStartNanoSec,
		unsigned int uPPSInterval,
		unsigned int uPPSWidth
		);

//
void GetSysTime(ADI_ETHER_HANDLE phDevice, int *pnSeconds, int *pnNanoSeconds);

void AdjSysTimeSecond(ADI_ETHER_HANDLE phDevice, time_t deltaSecond);

void ResetSysTime(ADI_ETHER_HANDLE phDevice);

void ProgrammingSysTimeFineCorrection(ADI_ETHER_HANDLE phDevice );

//void SetTrigerTimeofAuxiInCtrlPPS(ADI_ETHER_HANDLE phDevice, const TimeInternal *pAuxiTimeStamp);
//
//int handle_systime_calibration(	ADI_ETHER_HANDLE phDevice, const TimeInternal *pAuxiTimeStamp,
//							bool bTimingTestStarted);

int SetPtpPPSOut(void* hDev, int tmStartSec, int tmStartNanoSec );

int SetFixedPPSOutput(ADI_ETHER_HANDLE phDevice);

void SetTrigerTimeofAuxiInCtrlPPS(ADI_ETHER_HANDLE phDevice, const TimeInternal *pAuxiTimeStamp);

void TimeStampStatusInterruptHandler ( 	ADI_ETHER_HANDLE phDevice, uint32_t tm_status );


#endif /* PTP_ENGINE_H_ */
