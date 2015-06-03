/*
 * BF609_FPGA_Comm_Protocol.h
 *
 *  Created on: 2015-4-3
 *      Author: ChiliWang
 */

#ifndef BF609_FPGA_COMM_PROTOCOL_H_
#define BF609_FPGA_COMM_PROTOCOL_H_

typedef unsigned char  UINT8;
typedef char           INT8;
typedef unsigned short UINT16;
typedef short          INT16;
typedef unsigned int   UINT32;
typedef int            INT32;
/***********************************************************************************************
 * misc definition.
 * including return value definition and SPORT2-B map definition.
 *************************************************************************************************/
/*
 * Return value
 */
typedef enum BF609_FPGA_RESULT
{
	BF609_FPGA_SUCCESS = 0,
	BF609_FPGA_PARA_ERR,
	BF609_FPGA_ERR
}BF609_FPGA_RESULT;
/*
 * SPORT2B link to device
 */
typedef enum SPORT2B_Device
{
	ADS1278 = 0,
	AD7608_1
}SPORT2B_DEVICE;

/***********************************************************************************************
 * B-Code and SOE definition.
 *
 *************************************************************************************************/
/*
 * B码时间信息结构体
 */
typedef struct BcodeTime
{
	UINT8 ucBcodeYear;
    UINT8 ucBcodeDay;
    UINT8 ucBcodeHour;
	UINT8 ucBcodeMin;
	UINT8 ucBcodeSec;
}BCODE_TIME;
/*
 *  trigger definition
 */
typedef enum TriggerMode{
	RISING_EDGE = 0,	// rising  edge
	FALLING_EDGE,		// 01: falling edge
	BOTH_EDGE,			// 10: both    edge
	DEFAULT_RISING     //  11: rising  edge
}TRIG_MODE;
/*
 *SOE的遥控（YK）风暴配置
 */
typedef struct SOE_YK_Storm_Cfg
{
	 UINT32 uiYK_StormUTC;		//YK storm UTC time (second)
	 UINT32 usYK_StormUs ;		//YK storm us time
	 UINT16 usYK_StormNum;		//YK storm number of times

	 /*
	  *   1,3,5,7 output in time we set.
	  *   2,4,6,8 output when delay time is expired.
	  *   unit: ms, maximum 65535ms
	  */
	 UINT16 usYK_StormDelay;
	 UINT16 usYK_StormPeriod;	// unit: ms
	 UINT16 usYK_StormHiTime;	// unit: ms
}SOE_YK_STORM_CFG;
/***********************************************************************************************
 * FT3 parameter definition.
 *
 *************************************************************************************************/
/*
 * FT3 port
 *
 */
typedef enum FT3_Tx_Port{
	FT3_TX_PORT_1 = 0,
	FT3_TX_PORT_2,
	FT3_TX_PORT_3,
	FT3_TX_PORT_4,
	FT3_TX_PORT_5,
	FT3_TX_PORT_6
}FT3_TX_PORT;
typedef enum FT3_Rx_Port{
	FT3_RX_PORT_1 = 0,
	FT3_RX_PORT_2,
}FT3_RX_PORT;
/*
 * FT3 baud rate
 */
typedef enum FT3_Baud_Rate{
	FT3_2_5M_BITS = 1,
	FT3_4M_BITS,
	FT3_5M_BITS,
	FT3_6M_BITS,
	FT3_8M_BITS,
	FT3_10M_BITS,
	DEFAULT_2_5M_BITS  //2.5M
}FT3_BAUD_RATE;
/*
 * FT3 mode
 */
typedef enum FT3_Mode
{
	FT3_SYNC,
	FT3_ASYNC
}FT3_MODE;
/*
 * FT3 DATA logic
 * 正反码选择
 * （0：正码（逻辑‘1’定义为光纤亮）
 * 1：反码（逻辑‘1’定义为光纤灭）
 */
typedef enum FT3_Data_Logic
{
	FT3_NEGATIVE = 0,
	FT3_ACTIVE
}FT3_DATA_LOGIC;
/*
 * all FT3 CFG items.
 */
typedef struct FT3_Tx_Para_Cfg
{
	FT3_TX_PORT    FT3_TxPort; 		     //  ucFT3_Channel_ID   : 1-6: FT3 channel
	UINT16         usFT3_TxHead; 		 //  FT3 channel frame head
	UINT8    	   ucFT3_FrameLen;		 // [D15:D8]: 0-255
	FT3_DATA_LOGIC FT3_Data_Logic;
	FT3_MODE 	   FT3_Mode;
	FT3_BAUD_RATE  FT3_BaudRate;

}FT3_TX_PARA_CFG;
typedef struct FT3_Rx_Para_Cfg
{
	FT3_RX_PORT    FT3_RxPort; 		     //  ucFT3_Channel_ID   : 1-6: FT3 channel
	UINT16         usFT3_RxHead; 		 //  FT3 channel frame head
	FT3_DATA_LOGIC FT3_Data_Logic;
	FT3_MODE 	   FT3_Mode;
}FT3_RX_PARA_CFG;
/***********************************************************************************************
 * Sync input and output definition.
 *
 *************************************************************************************************/
/*
 * sync in port
*   			0-0000: 内建PPS
*  			  	1-0001: GPS模块的100M脉冲
*  			  	2-0010: IN1
*  			  	3-0011: IN2
*  			  	4-0100: IN3
*  			  	5-0101: IN4
*  			  	6-0110: ETH0_PTP_PPS
*  			  	7-0111: ETH1_PTP_PPS
*  			  	8-1000: 过0输出的PPS
*  			  	others: 内建PPS
 */
typedef enum Sync_In_Port{
	 INTERNAL_PPS = 0,
	 GPS_100M_IMPULSE,
	 IN1,
	 IN2,
	 IN3,
	 IN4,
	 ETH0_PTP_PPS,
	 ETH1_PTP_PPS,
	 DEFAULT_INTERNAL_PPS
}SYNC_IN_PORT;
/*
 * sync out port
 */
typedef enum Sync_Out_Port{
	OUT1 = 0,
	OUT2,
	OUT3,
	OUT4
}SYNC_OUT_PORT;
/*
 * sync type
 */
typedef enum Sync_Type{
	 PPS = 0,
	 IRIG_B
}SYNC_TYPE;
/*
 *
 */
typedef enum Sync_Logic{
	 NORMAL = 0,
	 REVERSE
}SYNC_LOGIC;
/*
 *
 */
typedef enum SYNC_OUT_STATUS{
	 RUN = 0,
	 STOP
}SYNC_OUT_STATUS;
/*
 * B码校验
 */
typedef enum Bcode_Check{
	 ODD_CHECK = 0,
	 EVEN_CHECK
}BCODE_CHECK;
/*
 * Output before or after
 */
typedef enum Sync_Out_Delay_Mode
{
	BEFORE = 0,
	AFTER
}DELAY_MODE;
/*
 * sync input cfg
 */
typedef struct Sync_In_Cfg{
	SYNC_IN_PORT  Port;
	SYNC_TYPE     Type;
	SYNC_LOGIC    Logic;
}SYNC_IN_CFG;
/*
 * sync output cfg
 */
typedef struct Sync_Out_Cfg{
	SYNC_OUT_PORT   Port;
	SYNC_TYPE       Type;
	SYNC_LOGIC     	Logic;

	BCODE_CHECK     BcodeCheck;

	/* Delay only available in OUT2-4*/
	DELAY_MODE      DelayMode;
	UINT32          Delay_Per_25ns;  /* one value means 25ns */
	SYNC_OUT_STATUS Status;
}SYNC_OUT_CFG;

/***********************************************************************************************
 * protocol frame definition.
 * We should disable memory align.
 *
 *************************************************************************************************/
#pragma pack(1)
/*
 * sync parameter cfg， 32-Bytes
 */
typedef struct SYNC_CFG{

	UINT16 usRefSyncInSel;  	/* SYNC reference input choose */
	UINT16 usMeterSyncInSel;    /* external Meter SYNC input choose  */
	UINT16 usSyncOutCfg;

	UINT32 uiSyncOut2Delay;
	UINT32 uiSyncOut3Delay;
	UINT32 uiSyncOut4Delay;

	/* B code time */
	UINT16 usBcodeYear;
	UINT16 usBcodeDay;
	UINT16 usBcodeHM;    /* 时分 */
	UINT16 usBcodeSec;
	UINT16 usBcodeCtl;

	UINT32 uiUTC;

}SYNC_CFG;
/*
 * 8路遥控触发时间信息, 0-7
 */
typedef struct SOE_YKx_TIME{
	UINT32 uiSOE_YKx_UTC;	    /* SOE的YK1秒触发时间	 */
	UINT16 usSOE_YKx_Us;        /* SOE的YK1微秒触发时间   */
}SOE_YKx_TIME;
/*
 * SOE CFG definition. 64-Bytes
 */
typedef struct SOE_CFG{
	UINT8 ucSOE_YX_EdgeFiltTime; 		 /* SOE的遥信防抖时间 	 */
	UINT8 ucSOE_YK_Status; 		     /* SOE的YK通道输出对应状态 	 */

	SOE_YKx_TIME SOE_YKx_Time_sa[8]; /*  8路遥控触发时间信息 */

	UINT32 uiSOE_YK_StormUTC;   	/* 遥控风暴秒触发时间设置      */
	UINT16 usSOE_YK_StormUs;   		/* 遥控风暴微秒触发时间设置  */
	UINT16 usSOE_YK_StormNum;   	/* 遥控风暴循环输出的总个数  */

	UINT16 usSOE_YK_StormDelay;		/* 遥控风暴延时输出时间 	  */
	UINT16 usSOE_YK_StormPeriod;    /* 遥控风暴单个脉冲的周期 	  */
	UINT16 usSOE_YK_StormHiTime;	/* 遥控风暴单个脉冲高电平时间*/

}SOE_CFG;
/*
 * FT3 channel data CFG
 */
typedef struct FT3_Tx_CHANNEL_CFG
{
	UINT16 usFT3_TxHead;			/* FT3 发送帧头   */
	UINT32 uiFT3_TxCfg;			/* FT3 发送帧配置 */
}FT3_TX_PORT_CFG;
/*
 * FT3 CFG definition. 44-Bytes
 */
typedef struct FT3_CFG{

	FT3_TX_PORT_CFG FT3_TxPortCfg_sa[6];
	UINT16 usFT3_Rx1Head;
	UINT16 usFT3_Rx1Cfg;

	UINT16 usFT3_Rx2Head;
	UINT16 usFT3_Rx2Cfg;
}FT3_CFG;
/*
 * FPGA To BF609 data, 65-Bytes
 */
typedef struct FPAG_BF609_DATA{

	UINT8  RecvBcodeData_a[16];		  /* B码抓包数据 13-Bytes, other 3-Bytes is padding */

	UINT32 uiCapturePPS_Time_ns;  	  /* 被检PPS的捕获ns时间 , 4-Bytes */

	SOE_YKx_TIME SOE_YKx_Time_sa[8];  /* 8路遥控触发时间信息  48-Bytes   */


}FPAG_BF609_DATA;

/*
 * BF609 To FPGA CFG, 144-Bytes
 */
typedef struct BF609_FPAG_CFG{
	UINT16   usAdc_cfg;        /* AD configuration, 2-Bytes	    */

	SYNC_CFG SyncCfg_s; 	   /* Sync configuration, 32-Bytes	*/

	SOE_CFG  SOE_Cfg_s;  	   /* SOE configuration, 64-Bytes	*/

	FT3_CFG  FT3_Cfg_s;   	   /* FT3 configuration, 44-Bytes	*/

}BF609_FPAG_CFG;


#pragma pack()

#endif /* BF609_FPGA_COMM_PROTOCOL_H_ */
