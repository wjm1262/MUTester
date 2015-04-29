/*
 * ptp_engine.c
 *
 *  Created on: 2015-3-11
 *      Author: Administrator
 */

#include "dri_ptp_engine.h"

#include "queue.h"
#include "arith.h"
#include "servo.h"




#define ETH_PTPAUXIN_PORTC_MUX  ((uint32_t) ((uint32_t) 2<<22))
#define ETH_PTPAUXIN_PORTC_FER  ((uint32_t) ((uint32_t) 1<<11))

// define Roll Over Nanosecond Mode
#define RO_SUBSEC_RES (0x7FFFFFFF +1)
#define RO_NANO_RES 	(0x3B9AC9FF+1)

void Init_PTPAuxin(void)
{
	 *pREG_PORTC_MUX |= ETH_PTPAUXIN_PORTC_MUX ;
	 ssync();

	  /* PORTx_FER registers */
	 *pREG_PORTC_FER |= ETH_PTPAUXIN_PORTC_FER ;
	 ssync();

	 // disable Time Stamp Interrupt
	 *pREG_EMAC0_IMSK  =  BITM_EMAC_IMSK_TS;
	 *pREG_EMAC1_IMSK  =  BITM_EMAC_IMSK_TS;

	 ssync();

}

void Enable_Time_Stamp_Auxin_Interrupt(ADI_ETHER_HANDLE phDevice)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	if( pDev->AuxiTMEnabled )
	{
		pEmacRegs->EMAC_IMSK =  0x0;
	}

	ssync();
}

void set_auxiTM_trigger_handler(void* hDev, void* pfHandler )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hDev;

	pDev->pAuxiTMTriggerHandler = (AUXITM_TRIGGER_HANDLER_FN) pfHandler;
}

void set_targetTM_trigger_handler(void* hDev, void* pfHandler )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hDev;

	pDev->pTmTriggerIntHandler = (TARGETTIME_TRIGGER_INTERRUPT_FN) pfHandler;
}

void InitSysTimeRegs(ADI_ETHER_HANDLE phDevice )
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	pEmacRegs->EMAC_TM_CTL = BITM_EMAC_TM_CTL_TSENA ; //ENUM_EMAC_TM_CTL_TS;//TSENA: Enable PTP Module

	//System Time Initialization
	//(a)
	pEmacRegs->EMAC_TM_SECUPDT = 0;
	pEmacRegs->EMAC_TM_NSECUPDT = 0;

	//(b)
	while( (pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSINIT) != 0 )
	{
		;
	}
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSINIT;//ENUM_EMAC_TM_CTL_EN_TS_INIT;//system time init

	//(c)
	pEmacRegs->EMAC_TM_CTL |= ENUM_EMAC_TM_CTL_RO_NANO_RES;

	//When EMAC_TM_CTL.TSCTRLSSR is clear, the EMAC_TM_NSEC register has a resolution of ~0.465ns.
	//Binary rollover mode, When reset, the roll over value of EMAC_TM_NSEC register is 0x7FFF_FFFF.

//	pEmacRegs->EMAC_TM_ADDEND = ADDEND_VAL ;
//
//	pEmacRegs->EMAC_TM_SUBSEC = s_NanoSecondUnit/PTP_REF_CLK; //0x14;//20ns, PTP Refer CLK = 50M
//
//	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSADDREG |ENUM_EMAC_TM_CTL_EN_FINE_UPDT;

	asm ( "SSYNC;" );
}

//void SetTrigerTimeofAuxiInCtrlPPS(ADI_ETHER_HANDLE phDevice, const TimeInternal *pAuxiTimeStamp)
//{
//	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
//	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
//	int tm_status;
//
//	tm_status = pEmacRegs->EMAC_TM_STMPSTAT;
//	pEmacRegs->EMAC_TM_PPSCTL &=  ~BITM_EMAC_TM_PPSCTL_TRGTMODSEL;
//
//	/* b.4 Program the start time value when the PPS output should start using the EMAC_TM_TGTM and EMAC_TM_
//	NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
//	time registers again.
//	*/
//	while(pEmacRegs->EMAC_TM_NTGTM & BITM_EMAC_TM_NTGTM_TSTRBUSY )
//	{
//		;
//	}
//	pEmacRegs->EMAC_TM_TGTM = pAuxiTimeStamp->seconds;
//	pEmacRegs->EMAC_TM_NTGTM = pAuxiTimeStamp->nanoseconds;
//
//	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSTRIG;
//
//}


//System Time Fine Correction
void ProgrammingSysTimeFineCorrection(ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	//select clk source
	*pREG_PADS0_EMAC_PTP_CLKSEL = 0x5;//SELECT SCLK0/SCLK1, EMAC0:SCLK0(0x1) AND EMAC1:SCLK0(0x4) ，SCLK(0X01)
	//*pREG_PADS0_EMAC_PTP_CLKSEL = 0x0;//EMAC0_RMII CLK

	pEmacRegs->EMAC_TM_ADDEND = ADDEND_VAL ;//changed as the PTP_REF_CLK,  clk source

	if((pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSCTRLSSR) == ENUM_EMAC_TM_CTL_RO_NANO_RES)
	{
		pEmacRegs->EMAC_TM_SUBSEC = RO_NANO_RES/PTP_REF_CLK; //0x14;//20ns, PTP Refer CLK = 50M
//		s_NanoSecondUnit = RO_NANO_RES;
	}
	else
	{
//		pEmacRegs->EMAC_TM_SUBSEC = RO_SUBSEC_RES/PTP_REF_CLK; //0x14;//20ns, PTP Refer CLK = 50M
//		s_NanoSecondUnit = RO_NANO_RES;
	}

	while( ( pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSADDREG ) != 0 )
	{
		;
	}
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSADDREG |ENUM_EMAC_TM_CTL_EN_FINE_UPDT;

	asm ( "SSYNC;" );
}



void ProgrammingSysTimeCoarseCorrection(ADI_ETHER_HANDLE phDevice, time_t SysSecondsError)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	//update the system time
	if(SysSecondsError > 0)
	{
		pEmacRegs->EMAC_TM_SECUPDT = SysSecondsError; //
		pEmacRegs->EMAC_TM_NSECUPDT = 0x80000000;//when set, subtracts the time
											//value with the contents of the update registers.
	}
	else
	{
		pEmacRegs->EMAC_TM_SECUPDT = -SysSecondsError; //
		pEmacRegs->EMAC_TM_NSECUPDT = 0x0;//
	}


	while( (pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSUPDT) != 0)
	{
		;
	}

//	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSUPDT;////
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSUPDT|ENUM_EMAC_TM_CTL_EN_COARSE_UPDT;////
}

//Programming The PTP for Frame Detection and Timestamping
void ProgrammingTimestamp(ADI_ETHER_HANDLE phDevice)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	//pEmacRegs->EMAC_TM_CTL = 0;

	// init steps for TASK:
	// a. Capture the ETHER FRAMES Time Stamp
	// c. Programming for Auxiliary Timestamps

	//a.1 set the TTSE bit in the TDES0 register of the corresponding frame
	// this is done in 'set_descriptor', if tx frame time stamp is enable

	//a.2  Extend the descriptor word length from 4 words to 8 words by setting the EMAC_DMA_BUSMODE.ATDS bit
	// this is the default settings in adi_gmac_EnableMAC

	//a.3 detects and/or timestamps only specific types of received frames
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENALL; //ENUM_EMAC_TM_CTL_E_TSALL_FRAMES;

	//a.4 select clk source
	//*pREG_PADS0_EMAC_PTP_CLKSEL = 0x5;//SELECT SCLK0/SCLK1, EMAC0:SCLK0(0x1) AND EMAC1:SCLK0(0x4) ，SCLK(0X01)

	//c.1 Set the EMAC_IMSK.TS bit to enable PTP interrupts. THIS is done after ENABLE the EMAC.
//
//		*pREG_EMAC0_IMSK  =  TM_AUXI_INT_EN;//BITM_EMAC_IMSK_TS;
//		asm ( "SSYNC;" );

	//a.5 Enable the PTP module
	//c.2 Set the EMAC_TM_CTL.TSENA bit to enable the PTP module
	//pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ; //ENUM_EMAC_TM_CTL_TS;//TSENA: Enable PTP Module


	//a.6 init system time
		//c.3 init system time, all initiations are over for TASK C.
	//InitSysTime( pEmacRegs );


	/*a.7 Verify the RDES4 register for the status of the received frame and the RDES6 and RDES7 registers for
		timestamp nanoseconds and seconds value.
	 */
	asm ( "SSYNC;" );
}



void GetSysTime(ADI_ETHER_HANDLE phDevice, int* pnSeconds, int *pnNanoSeconds)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	*pnNanoSeconds = pEmacRegs->EMAC_TM_NSEC;
	*pnSeconds = pEmacRegs->EMAC_TM_SEC;
}

void ResetSysTime(ADI_ETHER_HANDLE phDevice)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	//reset the system time
	pEmacRegs->EMAC_TM_SECUPDT = 0; //
	pEmacRegs->EMAC_TM_NSECUPDT = 0;//

	//add by wjm 2014-12-08
	while((pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSINIT) != 0 )
	{
		;
	}
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSINIT;
	asm ( "SSYNC;" );
}

void AdjSysTimeSecond(ADI_ETHER_HANDLE phDevice, time_t deltaSecond)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	//First, configure Coarse Correction to initialize the Second Reg...
	ProgrammingSysTimeCoarseCorrection(phDevice, deltaSecond);

	while( (pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSUPDT) == ENUM_EMAC_TM_CTL_EN_UPDATE)
	{
		;
	}

	//then, configure Fine Correction to adjust the Nanosecond Reg...
	ProgrammingSysTimeFineCorrection(phDevice);
}

void SetFixedPPSOutput(ADI_ETHER_HANDLE phDevice)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;


	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ;
	asm ( "SSYNC;" );



	pEmacRegs->EMAC_TM_PPSCTL = 0x1;
	asm ( "SSYNC;" );

	//0000,The default
//	value for these bits is 0000, which configures a 1 Hz signal with a pulse width equal to the period of the PTP
//	clock

}

void StopFlexiblePPSOutput( ADI_ETHER_HANDLE phDevice, PPS_TYPE ePPSType )
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	/*
	 * The PPS pulse train is free-running unless stopped by a STOP pulse train at
		time command (EMAC_TM_PPSCTL.PPSCTL = 0100) or STOP pulse train immediately command EMAC_
		TM_PPSCTL.PPSCTL = 0101).)
	 * */
	while( pEmacRegs->EMAC_TM_PPSCTL & 0xF != 0)
	{
		;
	}

	if( ePPSType == PULSE_TRAIN )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x5;//0101,stop pulse train immediately
	}

}


void SetEth0FlexiblePPSOutput( ADI_ETHER_HANDLE phDevice,
		PPS_TYPE ePPSType,
		int tmStartSec,
		int tmStartNanoSec,
		unsigned int uPPSInterval,
		unsigned int uPPSWidth
		)
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	// b. Programming Flexible Pulse-Per-Second Output
	//b.1 Enable the PTP module by setting the EMAC_TM_CTL.TSENA
	if( (pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSENA) != ENUM_EMAC_TM_CTL_TS )
	{
		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ;
		asm ( "SSYNC;" );
	}


	// b.2 EMAC_TM_PPSCTL.PPSEN bit to enable flexible PPS output,// b.3 TRGTMODSEL bits with 11
	pEmacRegs->EMAC_TM_PPSCTL = BITM_EMAC_TM_PPSCTL_PPSEN | BITM_EMAC_TM_PPSCTL_TRGTMODSEL;
	asm ( "SSYNC;" );

	/* b.4 Program the start time value when the PPS output should start using the EMAC_TM_TGTM and EMAC_TM_
	NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
	time registers again.
	*/
	while(pEmacRegs->EMAC_TM_NTGTM & BITM_EMAC_TM_NTGTM_TSTRBUSY )
	{
		;
	}
	pEmacRegs->EMAC_TM_TGTM = tmStartSec;
	pEmacRegs->EMAC_TM_NTGTM = tmStartNanoSec;
	asm ( "SSYNC;" );

	/* b.5 Program the period of the PPS signal output using the EMAC_TM_PPSINTVL register for pulse train
		output, and the width of the PPS signal output in the EMAC_TM_PPSWIDTH register for single pulse or
		pulse train output.
	*/

	*pREG_EMAC0_TM_PPSINTVL = uPPSInterval;//100M //0x2FAF080; //50M
	asm ( "SSYNC;" );
	*pREG_EMAC0_TM_PPSWIDTH = uPPSWidth;//0x2625A0;// 50ms
	asm ( "SSYNC;" );

	/* b.6 Ensure that the EMAC_TM_PPSCTL.PPSCTL bits are cleared and then program the bits to 0001 to start
		single pulse, or to 0010 to start pulse train at programmed start time (Step 4).
	  (ADDITIONAL INFORMATION: The PPS pulse train is free-running unless stopped by a STOP pulse train at
		time command (EMAC_TM_PPSCTL.PPSCTL = 0100) or STOP pulse train immediately command EMAC_
		TM_PPSCTL.PPSCTL = 0101).)
	*/
	while( pEmacRegs->EMAC_TM_PPSCTL & 0xF != 0)
	{
		;
	}

	if( ePPSType == PULSE_SINGLE )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x1;//0001,start SINGLE pulse
	}
	else if( ePPSType == PULSE_TRAIN )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x2;//0010,start pulse train
	}

	asm ( "SSYNC;" );

	/* b.7 The start of pulse generation can be cancelled by giving the cancel start command (EMAC_TM_PPSCTL.
		PPSCTL = 0011) before the programmed start time (Step 4) elapses
	*/

	/* b.8 Program the stop time value when the PPS output should stop using the EMAC_TM_TGTM and EMAC_TM_
		NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
		time registers again.
	*/


	/* b.9 Ensure that the EMAC_TM_PPSCTL.PPSCTL bits are cleared and then program them to 0100. This stops
		the train of pulses on PPS signal output after the programmed stop time (Step 8) elapses.
	*    (ADDITIONAL INFORMATION: The pulse train can be stopped immediately by giving the STOP pulse train
		immediately command (EMAC_TM_PPSCTL.PPSCTL = 0101). Similarly, the stop pulse train command
		(given in Step 9) can be cancelled by programming the EMAC_TM_PPSCTL.PPSCTL bits to 0110 before
		the programmed stop time (Step 8) elapses.)
	*/

}

void SetEth1FlexiblePPSOutput( ADI_ETHER_HANDLE phDevice,
		PPS_TYPE ePPSType,
		int tmStartSec,
		int tmStartNanoSec,
		unsigned int uPPSInterval,
		unsigned int uPPSWidth
		)
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	// b. Programming Flexible Pulse-Per-Second Output
	//b.1 Enable the PTP module by setting the EMAC_TM_CTL.TSENA
	if( (pEmacRegs->EMAC_TM_CTL & BITM_EMAC_TM_CTL_TSENA) != ENUM_EMAC_TM_CTL_TS )
	{
		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ;
	}


	// b.2 EMAC_TM_PPSCTL.PPSEN bit to enable flexible PPS output,// b.3 TRGTMODSEL bits with 11
	pEmacRegs->EMAC_TM_PPSCTL = BITM_EMAC_TM_PPSCTL_PPSEN | BITM_EMAC_TM_PPSCTL_TRGTMODSEL;

	/* b.4 Program the start time value when the PPS output should start using the EMAC_TM_TGTM and EMAC_TM_
	NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
	time registers again.
	*/
	while(pEmacRegs->EMAC_TM_NTGTM & BITM_EMAC_TM_NTGTM_TSTRBUSY )
	{
		;
	}
	pEmacRegs->EMAC_TM_TGTM = tmStartSec;
	pEmacRegs->EMAC_TM_NTGTM = tmStartNanoSec;

	/* b.5 Program the period of the PPS signal output using the EMAC_TM_PPSINTVL register for pulse train
		output, and the width of the PPS signal output in the EMAC_TM_PPSWIDTH register for single pulse or
		pulse train output.
	*/

	*pREG_EMAC1_TM_PPSINTVL = uPPSInterval;//100M //0x2FAF080; //50M
	*pREG_EMAC1_TM_PPSWIDTH = uPPSWidth;//0x2625A0;// 50ms



	/* b.6 Ensure that the EMAC_TM_PPSCTL.PPSCTL bits are cleared and then program the bits to 0001 to start
		single pulse, or to 0010 to start pulse train at programmed start time (Step 4).
	  (ADDITIONAL INFORMATION: The PPS pulse train is free-running unless stopped by a STOP pulse train at
		time command (EMAC_TM_PPSCTL.PPSCTL = 0100) or STOP pulse train immediately command EMAC_
		TM_PPSCTL.PPSCTL = 0101).)
	*/
	while( pEmacRegs->EMAC_TM_PPSCTL & 0xF != 0)
	{
		;
	}

	if( ePPSType == PULSE_SINGLE )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x1;//0001,start SINGLE pulse
	}
	else if( ePPSType == PULSE_TRAIN )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x2;//0010,start pulse train
	}


	/* b.7 The start of pulse generation can be cancelled by giving the cancel start command (EMAC_TM_PPSCTL.
		PPSCTL = 0011) before the programmed start time (Step 4) elapses
	*/

	/* b.8 Program the stop time value when the PPS output should stop using the EMAC_TM_TGTM and EMAC_TM_
		NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
		time registers again.
	*/


	/* b.9 Ensure that the EMAC_TM_PPSCTL.PPSCTL bits are cleared and then program them to 0100. This stops
		the train of pulses on PPS signal output after the programmed stop time (Step 8) elapses.
	*    (ADDITIONAL INFORMATION: The pulse train can be stopped immediately by giving the STOP pulse train
		immediately command (EMAC_TM_PPSCTL.PPSCTL = 0101). Similarly, the stop pulse train command
		(given in Step 9) can be cancelled by programming the EMAC_TM_PPSCTL.PPSCTL bits to 0110 before
		the programmed stop time (Step 8) elapses.)
	*/

}

void SetPtpPPSOut(void* hDev, int tmStartSec, int tmStartNanoSec )
{

	if(hDev == g_hEthDev[0])
	{
		SetEth0FlexiblePPSOutput( hDev, PULSE_TRAIN,
				tmStartSec, tmStartNanoSec,
				_4K_PPS_INTERVAL_VAL, _4K_PPS_WIDTH_VAL);
//				PPS_INTERVAL_VAL-1, PPS_WIDTH_VAL-1);
	}
	else
	{
		SetEth1FlexiblePPSOutput( hDev, PULSE_TRAIN,
				tmStartSec, tmStartNanoSec,
				PPS_INTERVAL_VAL-1, PPS_WIDTH_VAL-1);
	}
}

void SetTrigerTimeofAuxiInCtrlPPS(ADI_ETHER_HANDLE phDevice, const TimeInternal *pAuxiTimeStamp)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	int tm_status;

	tm_status = pEmacRegs->EMAC_TM_STMPSTAT;
	pEmacRegs->EMAC_TM_PPSCTL &=  ~BITM_EMAC_TM_PPSCTL_TRGTMODSEL;//Interrupt Only

	/* b.4 Program the start time value when the PPS output should start using the EMAC_TM_TGTM and EMAC_TM_
	NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
	time registers again.
	*/
	while(pEmacRegs->EMAC_TM_NTGTM & BITM_EMAC_TM_NTGTM_TSTRBUSY )
	{
		;
	}
	pEmacRegs->EMAC_TM_TGTM = pAuxiTimeStamp->seconds;
	pEmacRegs->EMAC_TM_NTGTM = pAuxiTimeStamp->nanoseconds;

	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSTRIG;

}

//
//void TargetTimeTriggerInterruptHandler(	ADI_ETHER_HANDLE phDevice )
//{
//#if 1
//	static bool bFirstTriggered = true;
//	static int idx = 0;
//	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
//
//	if(bFirstTriggered)
//	{
//		bFirstTriggered = false;
//		g_SystemRunningState.bIsTimeOpenEmac = true;
//	}
//
//	StartTransmit( phDevice );
//
//	idx++;
//
//	if(idx % 4000 != 0)
//	{
//		g_StartTxTime.nanoseconds = idx *250000;
//	}
//	else
//	{
//		idx = 0;
//		g_StartTxTime.seconds += 1;
//		g_StartTxTime.nanoseconds = 0;
//	}
//
//
//	SetTrigerTimeofAuxiInCtrlPPS(phDevice, &g_StartTxTime);
//
//#else
//	EnableCoreTimer(true);
//#endif
//
//}

void TimeStampStatusInterruptHandler ( 	ADI_ETHER_HANDLE phDevice, uint32_t tm_status )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	int i = 0;

	static TimeInternal AuxiTimeStamps[4] = {0};

	TIME_STAMP_TYPE tmTypeElem;

	uint32_t uCurAuxiFIFOCounter = (tm_status & BITM_EMAC_TM_STMPSTAT_ATSNS) >> BITP_EMAC_TM_STMPSTAT_ATSNS;

	for( i = 0; i < uCurAuxiFIFOCounter; i++)
	{
		AuxiTimeStamps[i].nanoseconds = pEmacRegs->EMAC_TM_AUXSTMP_NSEC;
		AuxiTimeStamps[i].seconds = pEmacRegs->EMAC_TM_AUXSTMP_SEC ;
	}

	/* Auxiliary Time Stamp Snapshot Trigger Missed */
	//RC/NW
	if( tm_status & BITM_EMAC_TM_STMPSTAT_ATSSTM )
	{
		DEBUG_STATEMENT("Auxiliary Time Stamp Snapshot Trigger Missed \n\n ");

 		//Set the EMAC_TM_CTL.ATSFC bit to clear the FIFO
		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_ATSFC;

		asm ( "SSYNC;" );

		return;
	}

	 /* Time Stamp Target Time Programming Error */
	/*The EMAC_TM_STMPSTAT.TSTRGTERR bit is set when the target time,
			which is being programmed in the EMAC_TM_SEC and EMAC_TM_NSEC
			registers, has already elapsed. This bit is cleared when read by the
			application.
	 * */
	//R/W
	if( tm_status & BITM_EMAC_TM_STMPSTAT_TSTRGTERR )
	{
		DEBUG_STATEMENT("Time Stamp Target Time Programming Error \n\n ");

		return;
	}

	 /* Auxiliary Time Stamp Trigger Snapshot */
	//RC/NW
	if( tm_status & BITM_EMAC_TM_STMPSTAT_ATSTS)
	{
//		if( EnQueue(&g_AuxiSnapshotTMQueue, &AuxiTimeStamps[i-1], false, false ) != Q_OK )
//		{
//			DEBUG_STATEMENT("EnQueue: AuxiSnapshotTMQueue Failed!\n\n ");
//		}

//		tmTypeElem.SnapshotTm 			= AuxiTimeStamps[i-1];
//		tmTypeElem.bIsDoTimingTest 		= false;
//		tmTypeElem.bTimingTestStarted 	= false;
//		pDev->pAuxiTMTriggerHandler( &tmTypeElem );

//		handle_auxiliary_tm_interrupt(	phDevice, &AuxiTimeStamps[i-1], false);
		if(pDev->pAuxiTMTriggerHandler)
		{
			pDev->pAuxiTMTriggerHandler( phDevice,  &AuxiTimeStamps[i-1] );
		}

	}

	 /* Time Stamp Target Time Reached */
	//RC/NW
	if( tm_status & BITM_EMAC_TM_STMPSTAT_TSTARGT )
	{
//		DEBUG_STATEMENT("Time Stamp Target Time Reached \n\n ");
//		TargetTimeTriggerInterruptHandler(phDevice);
		if(pDev->pTmTriggerIntHandler)
		{
			pDev->pTmTriggerIntHandler(phDevice);
		}
	}

	 /* Time Stamp Seconds Overflow */
	if( tm_status & BITM_EMAC_TM_STMPSTAT_TSSOVF )
	{
		DEBUG_STATEMENT("Time Stamp Seconds Overflow\n\n ");
	}

}
