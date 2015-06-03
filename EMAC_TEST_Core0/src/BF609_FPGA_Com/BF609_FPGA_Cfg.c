/*
 * BF609_FPGA_Comm_Protocol.c
 *
 *  Created on: 2015-4-13
 *      Author: ChiliWang
 *  Note:
 *  	 FPGA parameter setup function.
 */
#include "BF609_FPGA_Comm_Protocol.h"
#include "BF609_FPGA_Cfg.h"
#include <stdio.h>
/*
 *  DSP的SPORT2口映射
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ：   BF609->FPGA data pointer.
 *  			  Sport2b_Device  :   ADS1278 , AD7608_1((通道9-16对应的AD7608))
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgSPORT2B_Device(BF609_FPAG_CFG *pBF609_FPGA_Cfg, SPORT2B_DEVICE Sport2b_Device)
{
	bool bAD7608_1 = Sport2b_Device;
	if(bAD7608_1)
		pBF609_FPGA_Cfg->usAdc_cfg |=  (1u << 15);		/*  D15: 1   AD7608   */
	else
		pBF609_FPGA_Cfg->usAdc_cfg &= ~(1u << 15);		/*  D15: 0   ADS1278 */

	return BF609_FPGA_SUCCESS;
}

/*
 *  选择同步信号的时钟
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg ： BF609->FPGA data pointer.
 *  			  bIRIG_B     [D5]   : 0： PPS      1： IGIR-B （Only available in IN1-IN4）
 *  			  bReverse    [D4]   : 0: normal   1: reverse
 *  			  ucSyncInSrc [D3:D0]:
 *  			  					 0-0000: 内建PPS
 *  			  					 1-0001: GPS模块的100M脉冲
 *  			  					 2-0010: IN1
 *  			  					 3-0011: IN2
 *  			  					 4-0100: IN3
 *  			  					 5-0101: IN4
 *  			  					 6-0110: ETH0_PTP_PPS
 *  			  					 7-0111: ETH1_PTP_PPS
 *  			  					 8-1000: 过0输出的PPS
 *  			  					 others: 内建PPS
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgRefInSrc(BF609_FPAG_CFG *pBF609_FPGA_Cfg, SYNC_IN_CFG SyncInCfg_s)
{
	 bool bIRIG_B  		 = SyncInCfg_s.Type;
	 bool bReverse		 = SyncInCfg_s.Logic;
	 UINT8 ucSyncInSrc = SyncInCfg_s.Type;

	/* if reverse the src */
	if(bReverse)
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel |=  (1u << 4); /*  D4: 1  reverse */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel &= ~(1u << 4); /*  D4: 0  normal  */

	/* SYNC type select */
	if(bIRIG_B)
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel |=  (1u << 5); /*  D5: 1  IRIG-B */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel &= ~(1u << 5); /*  D5: 0  PPS    */

	/* reference SYNC input source select */
	pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel &= 0xFFF0;   /* Clear the legacy setup */
	if(ucSyncInSrc <= 0x0F)  /*check if SYNC type is legal */
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel |= ucSyncInSrc; /*  0000-1000: - */
	else
		return BF609_FPGA_PARA_ERR;

	return BF609_FPGA_SUCCESS;
}
/*
 *  选择被检信号
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg ： BF609->FPGA data pointer.
 *  			  bIRIG_B     [D5]   : 0： PPS      1： IGIR-B （Only available in IN1-IN4）
 *  			  bReverse    [D4]   : 0: normal   1: reverse
 *  			  ucSyncInSrc [D3:D0]:
 *  								 0-0000: 内建PPS
 *  			  					 1-0001: GPS模块的100M脉冲
 *  			  					 2-0010: IN1
 *  			  					 3-0011: IN2
 *  			  					 4-0100: IN3
 *  			  					 5-0101: IN4
 *  			  					 6-0110: ETH0_PTP_PPS
 *  			  					 7-0111: ETH1_PTP_PPS
 *  			  					 8-1000: 过0输出的PPS
 *  			  					 others: IN1
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgMeterInSrc(BF609_FPAG_CFG *pBF609_FPGA_Cfg, SYNC_IN_CFG MeterInCfg_s)
{
	 bool bIRIG_B  		     = MeterInCfg_s.Type;
	 bool bReverse		 	 = MeterInCfg_s.Logic;
	 UINT8 ucMeterInPort   = MeterInCfg_s.Port;
	/* if reverse the src */
	if(bReverse)
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel |=  (1u << 4); /*  D4: 1  reverse */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel &= ~(1u << 4); /*  D4: 0  normal  */

	/*
	 * meter SYNC input source select
	 */
	pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel &= 0xFFF0;   /* Clear the legacy setup */
	if(ucMeterInPort <= 0x0F)  /*check if meter SYNC type is legal */
		pBF609_FPGA_Cfg->SyncCfg_s.usRefSyncInSel |= ucMeterInPort; /*  0000-0110: - */
	else
		return BF609_FPGA_PARA_ERR;

	return BF609_FPGA_SUCCESS;
}
/*
 *  配置同步输出超前还是滞后，仅适用于out2-4
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ：   BF609->FPGA data pointer.
 *  			  Outx			   :  2-4
 *  			  bAhead  [D31]    :  1 : ahead  0: lag
 *  			  ucTime           :  ahead or lag  time, unit : 25ns.
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
inline BF609_FPGA_RESULT CfgSyncOutDelay(BF609_FPAG_CFG *pBF609_FPGA_Cfg, UINT8 Outx, DELAY_MODE DelayMode, UINT32 uiNum_of_25ns)
{
	switch (Outx)
	{
	case 2:
		if(DelayMode == BEFORE)  /* before */
			pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut2Delay |=  (1u << 31);
		else  /* after */
			pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut2Delay &= ~(1u << 31);
		pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut2Delay      = uiNum_of_25ns;
		break;

	case 3:
		if(DelayMode == BEFORE)	/* before */
			pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut3Delay |= (1u << 31);
		else  /* after */
			pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut3Delay &= ~(1u << 31);
		pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut3Delay      = uiNum_of_25ns;
		break;

	case 4:
		if(DelayMode == BEFORE)	/* before */
			pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut4Delay |= (1u << 31);
		else  /* after */
			pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut4Delay &= ~(1u << 31);
		pBF609_FPGA_Cfg->SyncCfg_s.uiSyncOut4Delay      = uiNum_of_25ns;

		break;

	default:
		break;
	}

	return BF609_FPGA_SUCCESS;
}
/*
 *  配置4路同步输出参数
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *  			  SyncOutCfg_s			:
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgSyncOutx(BF609_FPAG_CFG *pBF609_FPGA_Cfg, SYNC_OUT_CFG SyncOutCfg_s)
{
	UINT8   Outx 		  = SyncOutCfg_s.Port;
	UINT32  uiNum_of_25ns = SyncOutCfg_s.Delay_Per_25ns;
	/*
	 * OUTx configuration
	 */
	if(SyncOutCfg_s.Logic == REVERSE) /* if reverse? */
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg |=  (1u << (15-Outx));  /* Reverse    */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg &= ~(1u << (15-Outx));  /* normal     */

	if(SyncOutCfg_s.BcodeCheck == EVEN_CHECK) /* Check type */
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg |=  (1u << (11-Outx));  /* even check */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg &= ~(1u << (11-Outx));  /* odd check  */

	if(SyncOutCfg_s.Type == IRIG_B) /* output type */
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg |=  (1u << (7-Outx));  /*   IRIG_B    */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg &= ~(1u << (7-Outx));  /*   PPS       */

	if(SyncOutCfg_s.Status == STOP) /* stop ? */
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg |=  (1u << (3-Outx));  /*   stop      */
	else
		pBF609_FPGA_Cfg->SyncCfg_s.usSyncOutCfg &= ~(1u << (3-Outx));  /*   start     */
	/*
	 * Out Delay CFG
	 */
	CfgSyncOutDelay(pBF609_FPGA_Cfg, Outx+1, SyncOutCfg_s.DelayMode, uiNum_of_25ns);
	return BF609_FPGA_SUCCESS;
}

/*
 *  配置B码时间信息
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *				  BcodeTime_s
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgBCodeTime(BF609_FPAG_CFG *pBF609_FPGA_Cfg, BCODE_TIME BcodeTime_s)
{
	pBF609_FPGA_Cfg->SyncCfg_s.usBcodeYear = BcodeTime_s.ucBcodeYear;
	pBF609_FPGA_Cfg->SyncCfg_s.usBcodeDay  = BcodeTime_s.ucBcodeDay;
	pBF609_FPGA_Cfg->SyncCfg_s.usBcodeHM   = (BcodeTime_s.ucBcodeHour << 8) + BcodeTime_s.ucBcodeMin;
	pBF609_FPGA_Cfg->SyncCfg_s.usBcodeSec  = BcodeTime_s.ucBcodeSec;

	return BF609_FPGA_SUCCESS;
}
/*
 *  UTC time CFG
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *  			  UTC               : standard UTC second time
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgUTC_Time(BF609_FPAG_CFG *pBF609_FPGA_Cfg, UINT32 uiUTC)
{
	pBF609_FPGA_Cfg->SyncCfg_s.uiUTC = uiUTC;

	return BF609_FPGA_SUCCESS;
}
/*
 *  SOE 的遥信（YX）输入的配置
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *  			  ucEdge [D7:D6]   :  00: rising  edge
 *  			                      01: falling edge
 *  			                      10: both    edge
 *  			                      11: rising  edge
 *  			  ucFiltTime [D5:D0]    : software filter time 0-63ms
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgSOE_YX_EdgeFiltTime(BF609_FPAG_CFG *pBF609_FPGA_Cfg, TRIG_MODE TriggerMode, UINT8 ucFiltTime_ms)
{
	pBF609_FPGA_Cfg->SOE_Cfg_s.ucSOE_YX_EdgeFiltTime     = 0;    /* clear legacy cfg */

	/*
	 * Edge CFG
	 */
	pBF609_FPGA_Cfg->SOE_Cfg_s.ucSOE_YX_EdgeFiltTime |= (TriggerMode << 6);

	/*
	 * Filter time CFG
	 */
	if(ucFiltTime_ms <= 0x3F) /* check the input parameter */
		pBF609_FPGA_Cfg->SOE_Cfg_s.ucSOE_YX_EdgeFiltTime |= (ucFiltTime_ms);
	else
		return BF609_FPGA_PARA_ERR;

	return BF609_FPGA_SUCCESS;
}
/*
 *  SOE 的遥信（YX）输入的配置
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *  			  ucYK_Status      ：	0：断开
										1：闭合
* --------------------------------------------------------------------
 * | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1  | Bit0 |
 * | YK7  | YK6  | YK5  | YK4  | YK3  | YK2  | YK1   | YK0  |
 *---------------------------------------------------------------------
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgSOE_YK_SetStatus(BF609_FPAG_CFG *pBF609_FPGA_Cfg, UINT8 ucYK_Status)
{
	pBF609_FPGA_Cfg->SOE_Cfg_s.ucSOE_YK_Status  = ucYK_Status;    /* clear legacy cfg */
	return BF609_FPGA_SUCCESS;
}
/*
 *  SOE的遥控（YKx）触发时间配置
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *  			  YK_ID             : 1-8: YK1-YK8
 *  			  uiUTC             : UTC time
 *  			  usUs              : us time
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgSOE_YKxTrigTime(BF609_FPAG_CFG *pBF609_FPGA_Cfg, UINT8 YK_ID, UINT32 uiUTC, UINT16 usUs)
{
	YK_ID--;
	pBF609_FPGA_Cfg->SOE_Cfg_s.SOE_YKx_Time_sa[YK_ID].uiSOE_YKx_UTC = uiUTC;
	pBF609_FPGA_Cfg->SOE_Cfg_s.SOE_YKx_Time_sa[YK_ID].usSOE_YKx_Us  = usUs;

	return BF609_FPGA_SUCCESS;
}

/*
 *  SOE 的YKx风暴配置
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg  ： BF609->FPGA data pointer.
 *  			  uiYK_StormUTC     : YK storm UTC time (second)
 *  			  usYK_StormUs      : YK storm us time
 *  			  usYK_StormNum     : YK storm number
 *  			  usYK_StormDelay   : 1,3,5,7 output in time we set.
 *  			  					  2,4,6,8 output when delay time is expired.
 *  			  					  unit: ms, maximum 65535ms
 *  			  usYK_StormPeriod  : unit: ms
 *  			  usYK_StormHiTime  : unit: ms
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgSOE_YKxStorm(BF609_FPAG_CFG *pBF609_FPGA_Cfg, SOE_YK_STORM_CFG SOE_YK_Storm_Cfg)
{
	pBF609_FPGA_Cfg->SOE_Cfg_s.uiSOE_YK_StormUTC = SOE_YK_Storm_Cfg.uiYK_StormUTC;
	pBF609_FPGA_Cfg->SOE_Cfg_s.usSOE_YK_StormUs  = SOE_YK_Storm_Cfg.usYK_StormUs;
	pBF609_FPGA_Cfg->SOE_Cfg_s.usSOE_YK_StormNum = SOE_YK_Storm_Cfg.usYK_StormNum;

	pBF609_FPGA_Cfg->SOE_Cfg_s.usSOE_YK_StormDelay   = SOE_YK_Storm_Cfg.usYK_StormDelay;
	pBF609_FPGA_Cfg->SOE_Cfg_s.usSOE_YK_StormPeriod  = SOE_YK_Storm_Cfg.usYK_StormPeriod;
	pBF609_FPGA_Cfg->SOE_Cfg_s.usSOE_YK_StormHiTime  = SOE_YK_Storm_Cfg.usYK_StormHiTime;

	return BF609_FPGA_SUCCESS;
}
/*
 *  FT3 Tx data CFG
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg    ： BF609->FPGA data pointer.
 *  			  FT3_Tx_Para_Cfg	:
 *
 *  			  ucFT3_Port   : 1-6: FT3 channel
 *  			  usFT3_TxHead  : FT3 channel frame head
 *  			  ucFrameLen [D15:D8]: 0-255
 *  			  bNegative  [D7]    : 0: fiber bright in 1
 *  			  					   1: reverse
 *  			  bAsyncMode [D6]    : 0: Sync  FT3 mode
 *  			  					   1: Async FT3 mode
 *  			  ucBaudRate [D5:D3] : 000: 2M
 *  			  					   001: 2.5M
 *  			  					   010: 4M
 *  			  					   011: 5M
 *  			  					   100: 6M
 *  			  					   101: 8M
 *  			  					   110: 10M
 *  			  					   others: 2.5M
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgFT3_TxPort(BF609_FPAG_CFG *pBF609_FPGA_Cfg, FT3_TX_PARA_CFG FT3_Tx_Para_Cfg)
{

	UINT8  ucFT3_Port 	   = FT3_Tx_Para_Cfg.FT3_TxPort;
	UINT16 usFT3_TxHead    = FT3_Tx_Para_Cfg.usFT3_TxHead;
	UINT8  ucFrameLen	   = FT3_Tx_Para_Cfg.ucFT3_FrameLen;
	bool   bNegative	   = FT3_Tx_Para_Cfg.FT3_Data_Logic;
	bool   bAsyncMode	   = FT3_Tx_Para_Cfg.FT3_Mode;
	UINT8  ucBaudRate	   = FT3_Tx_Para_Cfg.FT3_BaudRate;

	pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].usFT3_TxHead = usFT3_TxHead;

	/*
	 * FT3 channel CFG
	 */
	pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg  = 0; /* Clear the legacy CFG*/
	pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg |=  (ucFrameLen << 8);

	if(bNegative) /* Negative Code ? */
		pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg |=  (1u << 7);
	else
		pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg &= ~(1u << 7);

	if(bAsyncMode) /* Async data mode ? */
		pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg |=  (1u << 6);
	else
		pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg &= ~(1u << 6);

	if(ucBaudRate <= 0x07) /* Paremeter legal ? */
		pBF609_FPGA_Cfg->FT3_Cfg_s.FT3_TxPortCfg_sa[ucFT3_Port].uiFT3_TxCfg |= (ucBaudRate << 3);
	else
		return BF609_FPGA_PARA_ERR;

	return BF609_FPGA_SUCCESS;
}
/*
 *  FT3 Rx data CFG
 *  Input  Value：
 *  			  pBF609_FPGA_Cfg    ： BF609->FPGA data pointer.
 *  			  FT3_Rx_Para_Cfg:
 *
 *  			  ucFT3_Port   : 1-2: FT3 channel
 *  			  usFT3_TxHead  : FT3 channel frame head
 *  			  bNegative  [D7]    : 0: fiber bright in 1
 *  			  					   1: reverse
 *  			  bAsyncMode [D6]    : 0: Sync  FT3 mode
 *  			  					   1: Async FT3 mode
 *
 *  Output Value： pBF609_FPGA_Cfg
 *  Return Value: BF609_FPGA_RESULT
 */
BF609_FPGA_RESULT CfgFT3_RxPort(BF609_FPAG_CFG *pBF609_FPGA_Cfg, FT3_RX_PARA_CFG FT3_Rx_Para_Cfg)
{

	/*
	 * FT3 channel CFG
	 */
	if(FT3_RX_PORT_1 == FT3_Rx_Para_Cfg.FT3_RxPort)
	{
		pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx1Head = FT3_Rx_Para_Cfg.usFT3_RxHead;

		/* FT3 data mode */
		if(FT3_ASYNC == FT3_Rx_Para_Cfg.FT3_Mode)
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx1Cfg |= (1u << 14);
		else
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx1Cfg &= ~(1u << 14);

		/* FT3 data logic */
		if(FT3_NEGATIVE == FT3_Rx_Para_Cfg.FT3_Data_Logic)
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx1Cfg |= (1u << 13);
		else
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx1Cfg &= ~(1u << 13);
	}
	else if(FT3_RX_PORT_2 == FT3_Rx_Para_Cfg.FT3_RxPort)
	{
		pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx2Head = FT3_Rx_Para_Cfg.usFT3_RxHead;

		/* FT3 data mode */
		if(FT3_ASYNC == FT3_Rx_Para_Cfg.FT3_Mode)
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx2Cfg |= (1u << 14);
		else
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx2Cfg &= ~(1u << 14);

		/* FT3 data logic */
		if(FT3_NEGATIVE == FT3_Rx_Para_Cfg.FT3_Data_Logic)
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx2Cfg |= (1u << 13);
		else
			pBF609_FPGA_Cfg->FT3_Cfg_s.usFT3_Rx2Cfg &= ~(1u << 13);
	}


	return BF609_FPGA_SUCCESS;
}
