
/*
 * msg.c
 *
 *  Created on: 2015-3-24
 *      Author: Wu JM
 */
#include "msg.h"

#include "myapp_cfg.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "crc_calculator.h"

/*
 * The following two intrinsics are available for
 * changing data from big-endian to little-endian, or vice versa.

#include <ccblkfn.h>
int byteswap4(int);
short byteswap2(short);
For example, byteswap2(0x1234) returns 0x3412.
Blackfin processors use a little-endian architecture.
 *
 */


UINT8 PC_MAC[6] = {0x00, 0x02, 0x03, 0x04, 0x05, 0x81};

RUNTIME_PARAMS g_rtParams ={0};

/*start：局部变量*/

UINT8 g_ControlPackSendBuf[MAX_COMM_PACK_LEN];			//发送控制帧的缓冲区

UINT8 g_FT3Recv1DataSendBuf[MAX_COMM_PACK_LEN];			//FT3转发帧的缓冲区
UINT8 g_FT3Recv2DataSendBuf[MAX_COMM_PACK_LEN];			//FT3转发帧的缓冲区
UINT8 g_netGoose1SendBuf[MAX_COMM_PACK_LEN];			//抄送点对点GOOSE1发送信息缓冲区
UINT8 g_netGoose2SendBuf[MAX_COMM_PACK_LEN];			//抄送点对点GOOSE2发送信息缓冲区


UINT8 g_timeErrorSendBuf[MAX_COMM_PACK_LEN];			//对时误差上报缓冲区
UINT8 g_DIEventSendBuf[MAX_COMM_PACK_LEN];				//开关量输入转发缓冲区
UINT8 g_DOEventSendBuf[MAX_COMM_PACK_LEN];				//开关量输出转发缓冲区

/*end：局部变量*/





inline UINT16 netHostChangeS(UINT16 netshort)
{
	UINT16 retValue;
	*((UINT8*)&retValue) = *(((UINT8*)&netshort)+1);
	*(((UINT8*)&retValue)+1) = *((UINT8*)&netshort);
	return retValue;
}


inline UINT32 netHostChangeL(UINT32 netlong)
{
	UINT32 retValue;
	*(((UINT8*)&retValue)+0) = *(((UINT8*)&netlong)+3);
	*(((UINT8*)&retValue)+1) = *(((UINT8*)&netlong)+2);
	*(((UINT8*)&retValue)+2) = *(((UINT8*)&netlong)+1);
	*(((UINT8*)&retValue)+3) = *(((UINT8*)&netlong)+0);
	return retValue;
}

/*Unpack Header from IN buffer to msgTmpHeader field */
void msgUnpackHeader ( UINT8 *InBuf, MUTestMsgHeader *header )
{
	MUTestMsgHeader *tmpHeader = (MUTestMsgHeader *)InBuf;

	header->code 	 = tmpHeader->code;
	header->netType  = netHostChangeS ( tmpHeader->netType );
	header->dataLeng = tmpHeader->dataLeng;
}


/*Pack header message into OUT buffer of ptpClock, send to PC*/
int msgPackHeader ( UINT8 *OutBuf, UINT8 MsgType, UINT16 NetType, UINT16 CmdCode, UINT16 DataLeng )
{
	UINT16 Len = MSG_HEADER_LEN + DataLeng;
	MUTestMsgHeader *header = (MUTestMsgHeader *)OutBuf;

	memcpy(	header->sourMac, user_net_config_info[2].hwaddr, 6 );
	memcpy( header->descMac, PC_MAC, 6 );
	header->descMac[5] = MsgType;		//控制命令:0x81; 转发帧:0x80

	header->netType    = netHostChangeS( NetType );

	header->code 	   = CmdCode;

	header->unDef 	   = 0x0000;
	header->dataLeng   = DataLeng;

	return Len;
}

/*Pack header message into OUT buffer of ptpClock, send to PC*/
// forward smv, goose, ft3 frames to PC.
int msgPackForwardFrm ( UINT8 *OutBuf, UINT16 CmdCode, UINT16 DataLeng, UINT32 sec, UINT32 nanoSec )
{
	UINT8 MsgType;
	UINT16 NetType;
	UINT16 FrmLen;

	UINT8 *dataStart = OutBuf + sizeof(MUTestMsgHeader);

	UINT8 *tempPoint = dataStart;

	*(UINT32*)tempPoint = sec;
	tempPoint += sizeof(UINT32);

	*(UINT32*)tempPoint = nanoSec;
	tempPoint += sizeof(UINT32);

	MsgType = MSG_FORWARD_FRM_TYPE; //转发帧
	NetType = NET_609_TRANSMIT ;
	FrmLen  = MSG_FORWARD_FRM_HEADER_LEN + DataLeng;

	return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, FrmLen );
}

int msgPackStandADFrm(UINT8* pOutBuf,  UINT8 asduNum)
{
	UINT8  MsgType;
	UINT16 NetType;
	UINT16 FrmLen;

	int i = 0;

	UINT32 ADDataLen = asduNum*sizeof(STAND_SAMP_TYPE);


	//pack 采样单元数目
	UINT8 *dataStart = pOutBuf + sizeof(MUTestMsgHeader) ;
	*dataStart = asduNum;
	dataStart += 4;


	//pack eth header
	MsgType = MSG_FORWARD_FRM_TYPE; //转发帧

	NetType = NET_609_TRANSMIT;

	FrmLen  =  4 + ADDataLen;

	return msgPackHeader ( pOutBuf, MsgType, NetType, TYPE609_CONT_STANDARD_DATA, FrmLen );

}


int msgPackDefaultReply(UINT8 isRight, UINT16 order, UINT16 errorCode, char *info )
{
	UINT8 *OutBuf = g_ControlPackSendBuf;
	UINT8 MsgType = MSG_CONTROL_FRM_TYPE;
	UINT16 CmdCode;

	if(isRight)
		CmdCode = TYPE609_CONT_RIGHT_REPLY;
	else
		CmdCode = TYPE609_CONT_ERROR_REPLY;

	UINT8* StartPos ;
	UINT8* tempPoint = OutBuf + sizeof(MUTestMsgHeader);
	StartPos = tempPoint;

	//应答命令
	*(UINT16*)tempPoint = order;
	tempPoint += sizeof(UINT16);

	//错误编码
	*(UINT16*)tempPoint = errorCode;
	tempPoint += sizeof(UINT16);

	//保留
	*(UINT16*)tempPoint = 0;
	tempPoint += sizeof(UINT16);

	//保留
	if(info)
	{
		UINT16 infoLen = strlen(info);
		*(UINT16*)tempPoint = infoLen;
		tempPoint += sizeof(UINT16);

		strcpy((char*)tempPoint,info);
		tempPoint += infoLen + 1;
	}
	else
	{
		*(UINT16*)tempPoint = 0;
		tempPoint += sizeof(UINT16);
	}

	UINT16 DataLeng = tempPoint - StartPos;

	UINT16 NetType  = NET_609_CONCROL;

	return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLeng );

}


UINT8 msgUnpackSmvFormatWrite(UINT8 *netData,UINT16 netDataSize)
{
	INT32 MsgLen = 0;

	PTR_SMV_PROTOCOL_PARA_HEAD temp = (PTR_SMV_PROTOCOL_PARA_HEAD)netData;
	SMV_PROTOCOL_PARA *smvPara = 0;

	if(temp->port == 1)
	{
		smvPara = &g_rtParams.NetSend1SmvPara;
	}
	else if(temp->port == 2)
	{
		smvPara = &g_rtParams.NetSend2SmvPara;
	}

	if(smvPara)
	{
		smvPara->asduNum = temp->asduNum;
		smvPara->chNum = temp->chNum;
		smvPara->proType = temp->proType;
		smvPara->sampCount = temp->sampCount;

		UINT8 *tempPoint = &netData[sizeof(SMV_PROTOCOL_PARA_HEAD)];
		UINT16 tempSize;

		tempSize = sizeof(UINT16) * smvPara->asduNum;
		memcpy(smvPara->cntIndex,tempPoint,tempSize);
		tempPoint += tempSize;

		tempSize = sizeof(UINT16) * smvPara->asduNum;
		memcpy(smvPara->firstValueIndex,tempPoint,tempSize);
		tempPoint += tempSize;

		tempSize = sizeof(UINT16);
		smvPara->frameLen = *((UINT16*)tempPoint);
		tempPoint += tempSize;

		tempSize = smvPara->chNum;
		memcpy(smvPara->chType,tempPoint,tempSize);
		tempPoint += tempSize;

		tempSize = smvPara->frameLen;
		memcpy(smvPara->frame,tempPoint,tempSize);
		tempPoint += tempSize;

		return 1;
	}
	else
	{
		return 0;
	}

}

INT32 msgPackSmvFormatRead(UINT8 *netData, UINT16 netDataSize )
{
	UINT8 *OutBuf = g_ControlPackSendBuf;
	UINT8  MsgType = MSG_CONTROL_FRM_TYPE;
	UINT16 NetType;
	UINT16 CmdCode;
	UINT16 DataLen;

	UINT8* tempPoint;
	PTR_SMV_PROTOCOL_PARA_HEAD temp ;

	PTR_SMV_PROTOCOL_PARA smvPara = 0;

	UINT8* StartPos ;

	UINT8 port = netData[0];

	if(port == 1)
	{
		smvPara = &g_rtParams.NetSend1SmvPara;
	}
	else if(port == 2)
	{
		smvPara = &g_rtParams.NetSend2SmvPara;
	}

	if(smvPara)
	{
		StartPos = tempPoint = OutBuf +  sizeof(MUTestMsgHeader);

		temp = (PTR_SMV_PROTOCOL_PARA_HEAD)tempPoint;

		temp->port = port;
		temp->asduNum = smvPara->asduNum;
		temp->chNum = smvPara->chNum;
		temp->proType = smvPara->proType;
		temp->sampCount = smvPara->sampCount;
		temp->undef2 = 0;

		tempPoint += sizeof(SMV_PROTOCOL_PARA_HEAD);
		UINT16 tempSize;

		tempSize = sizeof(UINT16) * smvPara->asduNum;
		memcpy(tempPoint,smvPara->cntIndex,tempSize);
		tempPoint += tempSize;

		tempSize = sizeof(UINT16) * smvPara->asduNum;
		memcpy(tempPoint,smvPara->firstValueIndex,tempSize);
		tempPoint += tempSize;

		tempSize = sizeof(UINT16);
		*((UINT16*)tempPoint) = smvPara->frameLen;
		tempPoint += tempSize;

		tempSize = smvPara->chNum;
		memcpy(tempPoint,smvPara->chType,tempSize);
		tempPoint += tempSize;

		tempSize = smvPara->frameLen;
		memcpy(tempPoint,smvPara->frame,tempSize);
		tempPoint += tempSize;


		NetType  = NET_609_CONCROL;
		CmdCode = TYPE609_CONT_SMV_FORMAT_READ;
		DataLen = tempPoint - StartPos;

		return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );
	}
	else
	{
		return msgPackDefaultReply(0,TYPE609_CONT_SMV_FORMAT_READ,COMM_ACK_ERROR, NULL);
	}
}

UINT8 msgUnpackFT3FormatWrite(UINT8 *netData,UINT16 netDataSize )
{
	int i = 0, len =0;
	UINT8 retValue = 0;

	UINT8 *tempPoint = netData;

	UINT16 sampCount = *(UINT16*)tempPoint;

	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		g_rtParams.FT3SendPara[i].sampCount = sampCount;
	}

	tempPoint += sizeof(UINT16);

	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		g_rtParams.FT3SendPara[i].mapCount = *tempPoint;
		tempPoint += sizeof(UINT8);
	}


	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		if(g_rtParams.FT3SendPara[i].mapCount)
		{
			len = g_rtParams.FT3SendPara[i].mapCount * sizeof(FT3_CH_Map_TYPE);
			memcpy(g_rtParams.FT3SendPara[i].chMap, tempPoint, len);
			tempPoint += len;
		}

	}

	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		g_rtParams.FT3SendPara[i].frameLen = *tempPoint;
		tempPoint += sizeof(UINT8);
	}


	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		len = g_rtParams.FT3SendPara[i].frameLen;
		if(len)
		{
			memcpy(g_rtParams.FT3SendPara[i].frame, tempPoint,len);
			tempPoint += len;
		}
	}

	return 1;
}

INT32 msgPackFT3FormatRead(UINT8 *netData,UINT16 netDataSize )
{
	UINT8 *OutBuf = g_ControlPackSendBuf;
	UINT8  MsgType = MSG_CONTROL_FRM_TYPE;
	UINT16 NetType;
	UINT16 CmdCode;
	UINT16 DataLen;

	UINT8* StartPos ;

	int i = 0, len = 0;
	UINT8 retValue = 0;

	UINT8 *tempPoint ;

	StartPos = tempPoint = OutBuf + sizeof(MUTestMsgHeader);

	*(UINT16*)tempPoint = g_rtParams.FT3SendPara[0].sampCount;
	tempPoint += sizeof(UINT16);

	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		*tempPoint = g_rtParams.FT3SendPara[i].mapCount;
		tempPoint += sizeof(UINT8);
	}

	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		len = g_rtParams.FT3SendPara[i].mapCount * sizeof(FT3_CH_Map_TYPE);
		memcpy(tempPoint,g_rtParams.FT3SendPara[i].chMap, len);
		tempPoint += len;
	}

	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		*tempPoint = g_rtParams.FT3SendPara[i].frameLen;
		tempPoint += sizeof(UINT8);
	}


	for(i =0; i < MAX_FT3_OUTPUT_NUM; i++)
	{
		len = g_rtParams.FT3SendPara[i].frameLen;
		memcpy(tempPoint,g_rtParams.FT3SendPara[i].frame, len);
		tempPoint += len;
	}


	NetType  = NET_609_CONCROL;
	CmdCode = TYPE609_CONT_FT3_FORMAT_READ;
	DataLen = tempPoint - StartPos;

	return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );
}

UINT8 COMM_isVirtualWrite(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16* sendSize)
{
//	UINT8 retValue = 0;
//
//	UINT8 *tempPoint = netData;
//
//	pVIRTUAL_DATA_TYPE temp = (pVIRTUAL_DATA_TYPE)netData;
//	if(temp->paraSize + 2 <=  netDataSize && netDataSize <= sizeof(VIRTUAL_DATA_TYPE))
//	{
//		memcpy(virtualData,netData,netDataSize);
//		retValue = msgPackDefaultReply(1,TYPE609_CONT_VIRTUAL_WIRTE,COMM_ACK_RIGHT, NULL);
//	}else
//	{//长度错误
//		retValue = msgPackDefaultReply(0,TYPE609_CONT_VIRTUAL_WIRTE,COMM_ACK_ERROR, NULL);
//	}
//
//	if(retValue)
//		return COMM_returnData(sendData,sendSize);
//	else
		return 0;
}

UINT8 COMM_isVirtualRead(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16*sendSize )
{
	UINT8 retValue = 0;

//	UINT8 *tempPoint = comSendPackDataStart;
//
//	memcpy(tempPoint,virtualData,sizeof(VIRTUAL_DATA_TYPE));
//	tempPoint += sizeof(VIRTUAL_DATA_TYPE);
//
//	comSendPackHead->code = TYPE609_CONT_VIRTUAL_READ;
//	comSendPackHead->dataLeng = tempPoint - comSendPackDataStart;
//	retValue = COMM_returnData(sendData,sendSize);

	return retValue;
}

UINT8 msgUnpackU32ParaWrite(UINT8 *netData,UINT16 netDataSize )
{
	UINT8 retValue = 0;

	UINT8 *tempPoint = netData;

	tempPoint += 2;		//保留

	UINT16 arraySize = *(UINT16*)tempPoint;
	tempPoint += sizeof(UINT16);

	UINT8 idx = 0;
	if(netDataSize >= arraySize * (sizeof(UINT8) + sizeof(UINT32)) + 4)
	{
		UINT8 *tempIndexPoint = tempPoint;
		UINT32 *tempValuePoint = (UINT32*)( tempPoint + sizeof(UINT8) * arraySize );

		for(int i = 0 ; i < arraySize; i ++)
		{
			idx = tempIndexPoint[i];
			if(idx)
			{
				g_rtParams.U32Parameter[idx] = tempValuePoint[i];
				g_rtParams.U32ParaChange[idx] = 1;		//标识参数变化
			}
		}
		return 1;
	}
	else
	{
		//长度错误
		return 0;
	}
}

INT32 msgPackU32ParaRead(UINT8 *netData, UINT16 netDataSize )
{

	UINT8 *OutBuf = g_ControlPackSendBuf;
	UINT8  MsgType = MSG_CONTROL_FRM_TYPE;
	UINT16 NetType;
	UINT16 CmdCode;
	UINT16 DataLen;


	UINT8 *tempPoint = g_ControlPackSendBuf + sizeof(MUTestMsgHeader);

	UINT8 * StartPos = tempPoint;

	tempPoint += 2;		//保留

	*(UINT16*)tempPoint = U32_PARAMETER_COUNT;
	tempPoint += sizeof(UINT16);

	memcpy(tempPoint, g_rtParams.U32Parameter, U32_PARAMETER_COUNT * sizeof(UINT32));
	tempPoint += (U32_PARAMETER_COUNT * sizeof(UINT32));


	NetType = NET_609_CONCROL;
	CmdCode = TYPE609_CONT_UINT32_PAR_READ;
	DataLen = tempPoint - StartPos;

	return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );
}


UINT8 msgUnpackU8ParaWrite(UINT8 *netData,UINT16 netDataSize )
{
	UINT8 retValue = 0;

	UINT8 *tempPoint = netData;

	tempPoint += 2;		//保留

	UINT16 arraySize = *(UINT16*)tempPoint;
	tempPoint += sizeof(UINT16);
	UINT8 Index;
	UINT8 *tempIndexPoint;
	UINT8 *tempValuePoint;

	if(netDataSize >= arraySize * (sizeof(UINT8) + sizeof(UINT8)) + 4)
	{
		tempIndexPoint = tempPoint;
		tempValuePoint = tempPoint + sizeof(UINT8) * arraySize;

		for(int i = 0 ; i < arraySize; i ++)
		{
			Index = tempIndexPoint[i];
			if(Index)
			{
				g_rtParams.U8Parameter[Index] = tempValuePoint[i];
				g_rtParams.U8ParaChange[Index] = 1;		//标识参数变化
			}
		}

		return 1;

	}
	else
	{//长度错误
		return 0;
	}

}

INT32 msgPackU8ParaRead(UINT8 *netData,UINT16 netDataSize )
{
	UINT8 *OutBuf = g_ControlPackSendBuf;
	UINT8  MsgType = MSG_CONTROL_FRM_TYPE;
	UINT16 NetType;
	UINT16 CmdCode;
	UINT16 DataLen;

	UINT8 retValue = 0;

	UINT8 *tempPoint = g_ControlPackSendBuf + sizeof(MUTestMsgHeader);

	UINT8 * StartPos = tempPoint;
	tempPoint += 2;		//保留

	*(UINT16*)tempPoint = U8_PARAMETER_COUNT;
	tempPoint += sizeof(UINT16);

	memcpy(tempPoint, g_rtParams.U8Parameter, U8_PARAMETER_COUNT * sizeof(UINT8));
	tempPoint += (U8_PARAMETER_COUNT * sizeof(UINT8));

	NetType = NET_609_CONCROL;
	CmdCode = TYPE609_CONT_UINT8_PAR_READ;
	DataLen = tempPoint - StartPos;

	return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );

}

UINT8 COMM_isYcCode(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16 *sendSize )
{
//	UINT8 retValue = 0;
//
//	UINT8 *tempPoint = netData;
//
//	tempPoint += 2;		//保留
//
//	KZ_PARAMETER_TYPE *ttt = kzPara;
//
//	UINT16 kzSize = *((UINT16*)tempPoint);
//	tempPoint += sizeof(UINT16);
//
//	tempPoint += 4;		//保留
//
//	if(kzSize <= MAX_KZ_CACHE_COUNT && netDataSize <= sizeof(KZ_PARAMETER_TYPE))
//	{
//		memcpy(kzPara,netData,netDataSize);
//		retValue = msgPackDefaultReply(1,TYPE609_CONT_YC_CODE,COMM_ACK_RIGHT, NULL);
//	}else
//	{//长度错误
//		retValue = msgPackDefaultReply(0,TYPE609_CONT_YC_CODE,COMM_ACK_ERROR, NULL);
//	}
//	if(retValue)
//		return COMM_returnData(sendData,sendSize);
//	else
		return 0;
}

UINT8 COMM_isInterityWrite(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16* sendSize )
{
//	UINT8 retValue = 0;
//
//	UINT8 *tempPoint = netData;
//
//	tempPoint += 2;		//保留
//	tempPoint += 1;		//保留
//
//	UINT8 port = *tempPoint;
//	tempPoint += 1;
//
//	LOSE_TYPE *losePara = 0;
//
//	if(port == 0x01)
//	{
//		losePara = netSendLosePara;
//	}else if(port == 0x02)
//	{
//		losePara = FT3SendLosePara;
//	}
//
//	if(losePara && netDataSize >= sizeof(LOSE_TYPE) + 4)
//	{
//		memcpy(losePara,tempPoint,sizeof(LOSE_TYPE));
//		retValue = msgPackDefaultReply(1,TYPE609_CONT_INTEGRITY_WRITE,COMM_ACK_RIGHT, NULL);
//	}else
//	{//长度错误
//		retValue = msgPackDefaultReply(0,TYPE609_CONT_INTEGRITY_WRITE,COMM_ACK_ERROR, NULL);
//	}
//
//	if(retValue)
//		return COMM_returnData(sendData,sendSize);
//	else
		return 0;
}

UINT8 COMM_isInterityRead(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16 *sendSize)
{
	UINT8 retValue = 0;

	UINT8 port = netData[3];

	pLOSE_TYPE losePara = 0;
	if(port == 0x01)
	{
		losePara = &g_rtParams.NetSendLosePara;
	}
	else if(port == 0x02)
	{
		losePara = &g_rtParams.FT3SendLosePara;
	}

//	if(port)
//	{
//		UINT8 *tempPoint = comSendPackDataStart;
//
//		tempPoint[0] = 0;
//		tempPoint[1] = 0;
//		tempPoint[2] = 0;
//		tempPoint[3] = port;
//		tempPoint += sizeof(UINT32);
//
//		memcpy(tempPoint,losePara,sizeof(LOSE_TYPE));
//		tempPoint += sizeof(LOSE_TYPE);
//
//		comSendPackHead->code = TYPE609_CONT_INTEGRITY_READ;
//		comSendPackHead->dataLeng = tempPoint - comSendPackDataStart;
//	}else
//	{//端口错误
//		retValue = msgPackDefaultReply(0,TYPE609_CONT_INTEGRITY_READ,COMM_ACK_ERROR, NULL);
//	}
//	retValue = COMM_returnData(sendData,sendSize);

	return retValue;
}

UINT8 msgUnpackGooseFormatWrite(UINT8 *netData,UINT16 netDataSize )
{
	UINT8 retValue = 0;

	UINT8 *tempPoint = netData;

	tempPoint += 2;		//保留
	tempPoint += 1;		//保留

	UINT8 port = *tempPoint;
	tempPoint += 1;

	GOOSE_PARA_TYPE *goosePara = 0;

	if(port == 0x01)
	{//点对点1
		goosePara = &g_rtParams.NetSend1GoosePara;
	}
	else if(port == 0x02)
	{//点对点2
		goosePara = &g_rtParams.NetSend2GoosePara;
	}

	if(goosePara && netDataSize - 4 <= sizeof(GOOSE_PARA_TYPE))
	{
		memcpy(goosePara, tempPoint, netDataSize - 4);
		return 1;
	}
	else
	{//长度错误
		return 0;
	}

}

INT32 msgPackGooseFormatRead(UINT8 *netData,UINT16 netDataSize  )
{
	UINT8 *OutBuf = g_ControlPackSendBuf;
	UINT8  MsgType = MSG_CONTROL_FRM_TYPE;
	UINT16 NetType;
	UINT16 CmdCode;
	UINT16 DataLen;

	UINT8 retValue = 0;

	UINT8 port = netData[3];


	UINT8 *tempPoint,*StartPos;
	PTR_GOOSE_PARA_TYPE goosePara = 0;
	if(port == 0x01)
	{//点对点1
		goosePara = &g_rtParams.NetSend1GoosePara;
	}
	else if(port == 0x02)
	{//点对点2
		goosePara = &g_rtParams.NetSend2GoosePara;
	}

	if(goosePara)
	{
		tempPoint = OutBuf + sizeof(MUTestMsgHeader);
		StartPos = tempPoint;

		tempPoint[0] = 0;
		tempPoint[1] = 0;
		tempPoint[2] = 0;
		tempPoint[3] = port;
		tempPoint += sizeof(UINT32);

		memcpy(tempPoint,goosePara,sizeof(GOOSE_PARA_TYPE));
		tempPoint += sizeof(GOOSE_PARA_TYPE);

//		comSendPackHead->code = TYPE609_CONT_GOOSE_FORMAT_READ;
//		comSendPackHead->dataLeng = tempPoint - comSendPackDataStart;

		NetType  = NET_609_CONCROL;
		CmdCode = TYPE609_CONT_GOOSE_FORMAT_READ;
		DataLen = tempPoint - StartPos;

		return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );

	}
	else
	{//端口错误
		return msgPackDefaultReply(0,TYPE609_CONT_GOOSE_FORMAT_READ,COMM_ACK_ERROR, NULL);
	}

}

UINT8 msgUnpackGooseDataWrite(UINT8 *netData,UINT16 netDataSize  )
{
	UINT8 retValue = 0;

	UINT8 *tempPoint = netData;

	tempPoint += sizeof(UINT8);		//保留

	UINT8 port = *tempPoint;
	tempPoint += sizeof(UINT8);

	UINT16 tempLen = *(UINT16*)tempPoint;
	tempPoint += sizeof(UINT16);

	PTR_GOOSE_PARA_TYPE goosePara = 0;
	if(port == 0x01)
	{//点对点1
		goosePara = &g_rtParams.NetSend1GoosePara;
	}
	else if(port == 0x02)
	{//点对点2
		goosePara = &g_rtParams.NetSend2GoosePara;
	}

	if(goosePara)
	{
		if(tempLen == (goosePara->frameLen - goosePara->allDataIndex)  && netDataSize >= tempLen + sizeof(UINT32))
		{
			memcpy(&(goosePara->frame[goosePara->allDataIndex]),tempPoint,tempLen);

			retValue = 1;

		}
		else
		{
			//数据长度错误
			retValue = 2;
		}
	}
	else
	{//端口错误
		retValue = 3;
	}

	return retValue;

}




enum
{
	MUA=0,MUB,MUC,MIA,MIB,MIC,PUA,PUB,PUC,PIA,PIB,PIC,ZERO_VAL,DELAY,CNT
};

//
int PackSmvFrm( UINT8* OutBuf, const STAND_SAMP_TYPE* pSmpData, UINT8 Port )
{
	SMV_PROTOCOL_PARA *pSmvPara = 0;
	UINT8 uChNo = 0;
	UINT8 uAsduNo = 0;
	unsigned int SmpCnt = pSmpData->sampCnt;

	if(Port == 1)
	{
		pSmvPara = &g_rtParams.NetSend1SmvPara;
	}
	else if(Port == 2)
	{
		pSmvPara = &g_rtParams.NetSend2SmvPara;
	}

	if(!pSmvPara )
	{
		return 0;
	}

	UINT8 *smvBuf = OutBuf ;

	// 换成MDMA
	memcpy(smvBuf, pSmvPara->frame,  pSmvPara->frameLen);

	//
	UINT16 cntIndex ;		//采样序号的位置

	UINT16 ch1Index;//第一个通道的在帧中的位置
	UINT8 chType;

	UINT32 MURadio, MIRadio, PURadio, PIRadio;

	UINT32 U32MapValueArray[15];// 需要转成 大端
	UINT16 U16MapValueArray[15];//注意：FT3的只还需要根据 有效值、量化因子量化，并转成 大端
	if( 0 == pSmvPara->proType)
	{
		//9-2
		MURadio = g_rtParams.U32Parameter[U32PARA_METER_VOL_RADIO];
		MIRadio = g_rtParams.U32Parameter[U32PARA_METER_CUR_RADIO];
		PURadio = g_rtParams.U32Parameter[U32PARA_PRO_VOL_RADIO];
		PIRadio = g_rtParams.U32Parameter[U32PARA_PRO_CUR_RADIO];

		U32MapValueArray[MUA]		= netHostChangeL( (UINT32) (pSmpData->UaSampData * MURadio) );
		U32MapValueArray[MUB]		= netHostChangeL( (UINT32) (pSmpData->UbSampData * MURadio) );
		U32MapValueArray[MUC]		= netHostChangeL( (UINT32) (pSmpData->UcSampData * MURadio) );

		U32MapValueArray[MIA]		= netHostChangeL( (UINT32) (pSmpData->IaSampData * MIRadio) );
		U32MapValueArray[MIB]		= netHostChangeL( (UINT32) (pSmpData->IbSampData * MIRadio) );
		U32MapValueArray[MIC]		= netHostChangeL( (UINT32) (pSmpData->IcSampData * MIRadio) );

		U32MapValueArray[PUA]		= netHostChangeL( (UINT32) (pSmpData->UaSampData * PURadio) );
		U32MapValueArray[PUB]		= netHostChangeL( (UINT32) (pSmpData->UbSampData * PURadio) );
		U32MapValueArray[PUC]		= netHostChangeL( (UINT32) (pSmpData->UcSampData * PURadio) );

		U32MapValueArray[PIA]		= netHostChangeL( (UINT32) (pSmpData->IaSampData * PIRadio) );
		U32MapValueArray[PIB]		= netHostChangeL( (UINT32) (pSmpData->IbSampData * PIRadio) );
		U32MapValueArray[PIC]		= netHostChangeL( (UINT32) (pSmpData->IcSampData * PIRadio) );

		U32MapValueArray[ZERO_VAL]	= 0;
		U32MapValueArray[DELAY]		= netHostChangeL( (UINT32) (pSmpData->netSendTMark) );
		U32MapValueArray[CNT]		= netHostChangeL( (UINT32) (pSmpData->sampCnt) );
	}
	else
	{
		//9-1

		U16MapValueArray[MUA]		= netHostChangeS( (UINT16) (pSmpData->UaSampData * 11585) );
		U16MapValueArray[MUB]		= netHostChangeS( (UINT16) (pSmpData->UbSampData * 11585) );
		U16MapValueArray[MUC]		= netHostChangeS( (UINT16) (pSmpData->UcSampData * 11585) );

		U16MapValueArray[MIA]		= netHostChangeS( (UINT16) (pSmpData->IaSampData * 11585) );
		U16MapValueArray[MIB]		= netHostChangeS( (UINT16) (pSmpData->IbSampData * 11585) );
		U16MapValueArray[MIC]		= netHostChangeS( (UINT16) (pSmpData->IcSampData * 11585) );

		U16MapValueArray[PUA]		= netHostChangeS( (UINT16) (pSmpData->UaSampData * 463) );
		U16MapValueArray[PUB]		= netHostChangeS( (UINT16) (pSmpData->UbSampData * 463) );
		U16MapValueArray[PUC]		= netHostChangeS( (UINT16) (pSmpData->UcSampData * 463) );

		U16MapValueArray[PIA]		= netHostChangeS( (UINT16) (pSmpData->IaSampData * 463) );
		U16MapValueArray[PIB]		= netHostChangeS( (UINT16) (pSmpData->IbSampData * 463) );
		U16MapValueArray[PIC]		= netHostChangeS( (UINT16) (pSmpData->IcSampData * 463) );

		U16MapValueArray[ZERO_VAL]	= 0;
		U16MapValueArray[DELAY]		= netHostChangeS( (UINT16) (pSmpData->netSendTMark) );
		U16MapValueArray[CNT]		= netHostChangeS( (UINT16) (pSmpData->sampCnt) );
	}

	if( 0 == pSmvPara->proType )
	{
		SMV_92_CH_TYPE *chValuePoint = NULL;

		for(uAsduNo = 0; uAsduNo < pSmvPara->asduNum; uAsduNo++)
		{
			//采样序号的位置
			cntIndex = pSmvPara->cntIndex[uAsduNo];

			*(smvBuf + cntIndex) = (SmpCnt + uAsduNo)>>8 ;
			*(smvBuf + cntIndex+1) = (SmpCnt + uAsduNo)&0x00ff;

			//the first channel
			ch1Index = pSmvPara->firstValueIndex[uAsduNo];
			chValuePoint = (SMV_92_CH_TYPE*) (smvBuf+ ch1Index);

			for( uChNo = 0; uChNo < pSmvPara->chNum; uChNo++, chValuePoint++)
			{
				chType = pSmvPara->chType[uChNo];

				chValuePoint->value = U32MapValueArray[chType - NEW609_CHTYPE_BASE];

			}//for each channel
		}
	}
	else
	{
		//9-1
		UINT16 *chValuePoint = NULL;
		for(uAsduNo = 0; uAsduNo < pSmvPara->asduNum; uAsduNo++)
		{
			//采样序号的位置
			cntIndex = pSmvPara->cntIndex[uAsduNo];

			*(smvBuf + cntIndex) = (SmpCnt + uAsduNo)>>8 ;
			*(smvBuf + cntIndex+1) = (SmpCnt + uAsduNo)&0x00ff;

			//the first channel
			ch1Index = pSmvPara->firstValueIndex[uAsduNo];
			chValuePoint = (UINT16*) (smvBuf+ ch1Index);

			for( uChNo = 0; uChNo < pSmvPara->chNum; uChNo++, chValuePoint++)
			{
				chType = pSmvPara->chType[uChNo];

				*chValuePoint = U16MapValueArray[chType - NEW609_CHTYPE_BASE];

			}//for each channel
		}
	}

	return 1;
}

int PackFT3Frm(UINT8 *OutBuf, const STAND_SAMP_TYPE* pSmpData)
{
	uint8_t *pDestFrm = OutBuf;
	uint8_t *pOldDestFrm = pDestFrm;
	FT3_PROTOCOL_PARA *pFT3_Protocol_Para;
	uint8_t *pOriginFrm        ;
	uint8_t ucChannelNum       ;
	FT3_CH_Map_TYPE *pMapArray ;
	uint8_t ucFrmLen           ;
	int32_t totalLen=0;

	uint8_t index = 0;
	uint8_t chType;
	uint16_t* pChData;

	uint8_t  CRC_CheckLen;
	uint8_t  CRC_Start;
	uint16_t CRC ;

//	UINT32 MURadio = g_rtParams.U32Parameter[U32PARA_METER_VOL_RADIO];
//	UINT32 MIRadio = g_rtParams.U32Parameter[U32PARA_METER_CUR_RADIO];
//	UINT32 PURadio = g_rtParams.U32Parameter[U32PARA_PRO_VOL_RADIO];
//	UINT32 PIRadio = g_rtParams.U32Parameter[U32PARA_PRO_CUR_RADIO];

	UINT16 U16MapValueArray[15];//注意：FT3的只还需要根据 有效值、量化因子量化，并转成 大端
	U16MapValueArray[MUA]		= netHostChangeS( (UINT16) (pSmpData->UaSampData * 11585) );
	U16MapValueArray[MUB]		= netHostChangeS( (UINT16) (pSmpData->UbSampData * 11585) );
	U16MapValueArray[MUC]		= netHostChangeS( (UINT16) (pSmpData->UcSampData * 11585) );

	U16MapValueArray[MIA]		= netHostChangeS( (UINT16) (pSmpData->IaSampData * 11585) );
	U16MapValueArray[MIB]		= netHostChangeS( (UINT16) (pSmpData->IbSampData * 11585) );
	U16MapValueArray[MIC]		= netHostChangeS( (UINT16) (pSmpData->IcSampData * 11585) );

	U16MapValueArray[PUA]		= netHostChangeS( (UINT16) (pSmpData->UaSampData * 463) );
	U16MapValueArray[PUB]		= netHostChangeS( (UINT16) (pSmpData->UbSampData * 463) );
	U16MapValueArray[PUC]		= netHostChangeS( (UINT16) (pSmpData->UcSampData * 463) );

	U16MapValueArray[PIA]		= netHostChangeS( (UINT16) (pSmpData->IaSampData * 463) );
	U16MapValueArray[PIB]		= netHostChangeS( (UINT16) (pSmpData->IbSampData * 463) );
	U16MapValueArray[PIC]		= netHostChangeS( (UINT16) (pSmpData->IcSampData * 463) );

	U16MapValueArray[ZERO_VAL]	= 0;
	U16MapValueArray[DELAY]		= netHostChangeS((UINT16) (pSmpData->netSendTMark) );
	U16MapValueArray[CNT]		= netHostChangeS((UINT16) (pSmpData->sampCnt) );

	int i,j;
	for(i = 0; i < MAX_FT3_OUTPUT_NUM; i++ )
	{
		pFT3_Protocol_Para = g_rtParams.FT3SendPara + i;

		pOriginFrm         = pFT3_Protocol_Para->frame;
		ucChannelNum       = pFT3_Protocol_Para->mapCount;
		pMapArray 		   = pFT3_Protocol_Para->chMap;
		ucFrmLen           = pFT3_Protocol_Para->frameLen;
		totalLen		  += ucFrmLen;

		//注意：
		// 1.第一帧的ucChannelNum不能为0，即第一帧不能是复制帧；
		// 2.复制帧的ucFrmLen与原始帧长度一致。
		if(0 == ucChannelNum)
		{
			memcpy(pDestFrm, pOldDestFrm, ucFrmLen);
		}
		else
		{
			memcpy(pDestFrm, pOriginFrm, ucFrmLen);
			for(j = 0; j < ucChannelNum; j++ )
			{
				index  = pMapArray[i].index;
				chType = pMapArray[i].chType;

				/* used for FT3 CRC map */
				if(chType < NEW609_CHTYPE_CRC_START )
				{
					pChData = (uint16_t*)(pDestFrm + index);
					*pChData = U16MapValueArray[chType - NEW609_CHTYPE_BASE];
				}
				else if((chType >= NEW609_CHTYPE_CRC_START ) && (chType < NEW609_CHTYPE_CRC_STOP))
				{
					CRC_CheckLen 		= chType - 0x80;
					CRC_Start    		= index - CRC_CheckLen;
					CRC          		= Cal_CRC16_ByByte(pDestFrm + CRC_Start, CRC_CheckLen);
					pDestFrm[index]     = CRC >> 8;
					pDestFrm[index + 1] = CRC & 0XFF;
				}
			}//for each channel
		}

		pOldDestFrm = pDestFrm;
		pDestFrm += ucFrmLen;

	}// for each port

	return totalLen;
}


///////////////////////////////////////////////////////

ADI_ETHER_BUFFER *PackForwardFrame( UINT16 CmdCode, uint32_t unSecond, uint32_t unNanoSecond,
										uint16_t FrmLen,
										 ADI_ETHER_BUFFER *pXmtBuf)
{

	ADI_ETHER_BUFFER *tx = pXmtBuf;

	uint8_t *head;

	uint16_t  PayLoadLen ;
	uint16_t TotalLen;

	UINT8 MsgType;
	UINT16 NetType;

	head = (uint8_t*)tx->Data +2;
//	TotalLen = msgPackForwardFrm ( head, CmdCode, PayLoadLen, unSecond, unNanoSecond );

	UINT8 *tempPoint = head + sizeof(MUTestMsgHeader);

	*(UINT32*)tempPoint = unSecond;
	tempPoint += sizeof(UINT32);

	*(UINT32*)tempPoint = unNanoSecond;
	tempPoint += sizeof(UINT32);

	MsgType = MSG_FORWARD_FRM_TYPE; //转发帧
	NetType = NET_609_TRANSMIT ;

	PayLoadLen  = MSG_FORWARD_FRM_HEADER_LEN + FrmLen;

	TotalLen = msgPackHeader ( head, MsgType, NetType, CmdCode, PayLoadLen );


	*(short*)tx->Data = TotalLen;

	tx->ElementCount = TotalLen + 2; // total element count including 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}
