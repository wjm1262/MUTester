/*
 * msg.h
 *
 *  Created on: 2015-3-24
 *      Author: Wu JM
 */

#ifndef MSG_H_
#define MSG_H_

#include "comm_pc_protocol.h"
#include <stdlib.h>
#include <string.h>
#include <drivers/ethernet/adi_ether.h>

#include "VerUpgrade.h"

/*******************************
start：共享数据
*******************************/

#define MAX_FT3_OUTPUT_NUM (6)
typedef struct sRUNTIME_PARAMS
{

	SMV_PROTOCOL_PARA NetSend1SmvPara;		//点对点1发送参数
	SMV_PROTOCOL_PARA NetSend2SmvPara;		//点对点2发送参数

	FT3_PROTOCOL_PARA FT3SendPara[MAX_FT3_OUTPUT_NUM];


	VIRTUAL_DATA_TYPE virtualData;					//虚拟发生器缓存,数组大小为VIRTUAL_DATA_SIZE

	UINT32 U32Parameter[U32_PARAMETER_COUNT];				//U32参数缓存，数组大小见U32_PARAMETER_COUNT
	UINT8 U32ParaChange[U32_PARAMETER_COUNT];
	UINT8 U8Parameter[U8_PARAMETER_COUNT];				//
	UINT8 U8ParaChange[U8_PARAMETER_COUNT];

	KZ_PARAMETER_TYPE kzPara;

	LOSE_TYPE NetSendLosePara;
	LOSE_TYPE FT3SendLosePara;

	GOOSE_PARA_TYPE NetSend1GoosePara;
	GOOSE_PARA_TYPE NetSend2GoosePara;
} RUNTIME_PARAMS;

extern RUNTIME_PARAMS g_rtParams;

extern UINT8 PC_MAC[6] ;
extern UINT8 g_ControlPackSendBuf[MAX_COMM_PACK_LEN];			//发送控制帧的缓冲区
/*******************************
end：共享数据
*******************************/

/*******************************
start：导出接口
*******************************/


void msgUnpackHeader ( UINT8 *InBuf, MUTestMsgHeader *header );

int msgPackHeader ( UINT8 *OutBuf, UINT8 MsgType, UINT16 NetType,
									UINT16 CmdCode, UINT16 DataLeng );


UINT8 msgUnPackSoftWareVersionUpdate(UINT8 *netData,UINT16 netDataSize);

INT32 PackSoftWareVersionUpdateAckFrm( bool bRight, VER_UPDATE_ACK_CODE errCode);

INT32 msgPackSoftWareVersionRead(UINT8 *netData, UINT16 netDataSize );

int msgPackDefaultReply(UINT8 isRight, UINT16 order, UINT16 errorCode, char *info );

UINT8 msgUnpackSmvFormatWrite(UINT8 *netData,UINT16 netDataSize);
int msgPackSmvFormatRead(UINT8 *netData, UINT16 netDataSize );

UINT8 msgUnpackU8ParaWrite(UINT8 *netData,UINT16 netDataSize );
INT32 msgPackU8ParaRead(UINT8 *netData,UINT16 netDataSize );

UINT8 msgUnpackU32ParaWrite(UINT8 *netData,UINT16 netDataSize );
INT32 msgPackU32ParaRead(UINT8 *netData, UINT16 netDataSize );

UINT8 msgUnpackGooseFormatWrite(UINT8 *netData,UINT16 netDataSize );
INT32 msgPackGooseFormatRead(UINT8 *netData,UINT16 netDataSize  );
UINT8 msgUnpackGooseDataWrite(UINT8 *netData,UINT16 netDataSize  );

UINT8 msgUnpackFT3FormatWrite(UINT8 *netData,UINT16 netDataSize );
INT32 msgPackFT3FormatRead(UINT8 *netData,UINT16 netDataSize );

//////////////////////////

int msgPackStandADFrm(UINT8* pOutBuf,  UINT8 asduNum);
int PackSmvFrm( UINT8* OutBuf, const STAND_SAMP_TYPE* pStandADData, UINT8 Port );
int PackFT3Frm(UINT8 *OutBuf, const STAND_SAMP_TYPE* pSmpData);
ADI_ETHER_BUFFER *PackForwardFrame( UINT16 CmdCode,
		uint32_t unSecond,
		uint32_t unNanoSecond,
		uint32_t unRxCnt,
		uint16_t FrmLen,
		ADI_ETHER_BUFFER *pXmtBuf);

/*******************************
end：导出接口
*******************************/


#endif
