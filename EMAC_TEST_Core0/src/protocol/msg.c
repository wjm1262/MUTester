#include "msg.h"

#include "myapp_cfg.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>



UINT8 PC_MAC[6] = {0x00, 0x02, 0x03, 0x04, 0x05, 0x81};

RUNTIME_PARAMS g_rtParams;

/*start：局部变量*/

UINT8 g_ControlPackSendBuf[MAX_COMM_PACK_LEN];			//发送控制帧的缓冲区
//UINT8 netRecv1DataSendBuf[MAX_COMM_PACK_LEN];			//点对点转发帧的缓冲区
//UINT8 netRecv2DataSendBuf[MAX_COMM_PACK_LEN];			//点对点转发帧的缓冲区
UINT8 g_FT3Recv1DataSendBuf[MAX_COMM_PACK_LEN];			//FT3转发帧的缓冲区
UINT8 g_FT3Recv2DataSendBuf[MAX_COMM_PACK_LEN];			//FT3转发帧的缓冲区
UINT8 g_netGoose1SendBuf[MAX_COMM_PACK_LEN];			//抄送点对点GOOSE1发送信息缓冲区
UINT8 g_netGoose2SendBuf[MAX_COMM_PACK_LEN];			//抄送点对点GOOSE2发送信息缓冲区

UINT8 g_standardDataSendBuf[MAX_COMM_PACK_LEN];			//发送标准采样数据缓冲区

UINT8 g_timeErrorSendBuf[MAX_COMM_PACK_LEN];			//对时误差上报缓冲区
UINT8 g_DIEventSendBuf[MAX_COMM_PACK_LEN];				//开关量输入转发缓冲区
UINT8 g_DOEventSendBuf[MAX_COMM_PACK_LEN];				//开关量输出转发缓冲区

/*end：局部变量*/

UINT8 COMM_clearBuf();

UINT8 COMM_initBuf()
{
	return 1;
}

UINT8 COMM_clearBuf()
{
	return 1;
}


UINT16 netHostChangeS(UINT16 netshort)
{
	UINT16 retValue;
	*((UINT8*)&retValue) = *(((UINT8*)&netshort)+1);
	*(((UINT8*)&retValue)+1) = *((UINT8*)&netshort);
	return retValue;
}


UINT32 netHostChangeL(UINT32 netlong)
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

	UINT16 NetType  = NET_609_TRANSMIT;

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


		NetType  = NET_609_TRANSMIT;
		CmdCode = TYPE609_CONT_SMV_FORMAT_READ;
		DataLen = tempPoint - StartPos;

		return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );
	}
	else
	{
		return msgPackDefaultReply(0,TYPE609_CONT_SMV_FORMAT_READ,COMM_ACK_ERROR, NULL);
	}
}

UINT8 COMM_isFT3FormatWrite(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16* sendSize)
{
	UINT8 retValue = 0;

//	UINT8 *tempPoint = netData;
//
//	UINT16 sampCount = *(UINT16*)tempPoint;
//	g_rtParams.FT3Send1Para.sampCount = sampCount;
//	g_rtParams.FT3Send2Para.sampCount = sampCount;
//	g_rtParams.FT3Send3Para.sampCount = sampCount;
//	g_rtParams.FT3Send4Para.sampCount = sampCount;
//	g_rtParams.FT3Send5Para.sampCount = sampCount;
//	g_rtParams.FT3Send6Para.sampCount = sampCount;
//	tempPoint += sizeof(UINT16);
//
//	//测试用
//	FT3_PROTOCOL_PARA *tt1 = FT3Send1Para;
//	FT3_PROTOCOL_PARA *tt2 = FT3Send2Para;
//	FT3_PROTOCOL_PARA *tt3 = FT3Send3Para;
//	FT3_PROTOCOL_PARA *tt4 = FT3Send4Para;
//	FT3_PROTOCOL_PARA *tt5 = FT3Send5Para;
//	FT3_PROTOCOL_PARA *tt6 = FT3Send6Para;
//	//end 测试用
//
//
//	g_rtParams.FT3Send1Para.mapCount = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send2Para.mapCount = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send3Para.mapCount = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send4Para.mapCount = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send5Para.mapCount = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send6Para.mapCount = *tempPoint;
//	tempPoint += sizeof(UINT8);
//
//	if(g_rtParams.FT3Send1Para.mapCount)
//	{
//		memcpy(g_rtParams.FT3Send1Para.chMap,tempPoint,g_rtParams.FT3Send1Para.mapCount * sizeof(FT3_CH_Map_TYPE));
//		tempPoint += g_rtParams.FT3Send1Para.mapCount * sizeof(FT3_CH_Map_TYPE);
//	}
//	if(g_rtParams.FT3Send2Para.mapCount)
//	{
//		memcpy(g_rtParams.FT3Send2Para.chMap,tempPoint,g_rtParams.FT3Send2Para.mapCount * sizeof(FT3_CH_Map_TYPE));
//		tempPoint += g_rtParams.FT3Send2Para.mapCount * sizeof(FT3_CH_Map_TYPE);
//	}
//	if(g_rtParams.FT3Send3Para.mapCount)
//	{
//		memcpy(g_rtParams.FT3Send3Para.chMap,tempPoint,g_rtParams.FT3Send3Para.mapCount * sizeof(FT3_CH_Map_TYPE));
//		tempPoint += g_rtParams.FT3Send3Para.mapCount * sizeof(FT3_CH_Map_TYPE);
//	}
//	if(g_rtParams.FT3Send4Para.mapCount)
//	{
//		memcpy(g_rtParams.FT3Send4Para.chMap,tempPoint,g_rtParams.FT3Send4Para.mapCount * sizeof(FT3_CH_Map_TYPE));
//		tempPoint += g_rtParams.FT3Send4Para.mapCount * sizeof(FT3_CH_Map_TYPE);
//	}
//	if(g_rtParams.FT3Send5Para.mapCount)
//	{
//		memcpy(g_rtParams.FT3Send5Para.chMap,tempPoint,g_rtParams.FT3Send5Para.mapCount * sizeof(FT3_CH_Map_TYPE));
//		tempPoint += g_rtParams.FT3Send5Para.mapCount * sizeof(FT3_CH_Map_TYPE);
//	}
//	if(g_rtParams.FT3Send6Para.mapCount)
//	{
//		memcpy(g_rtParams.FT3Send6Para.chMap,tempPoint,g_rtParams.FT3Send6Para.mapCount * sizeof(FT3_CH_Map_TYPE));
//		tempPoint += g_rtParams.FT3Send6Para.mapCount * sizeof(FT3_CH_Map_TYPE);
//	}
//
//	g_rtParams.FT3Send1Para.frameLen = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send2Para.frameLen = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send3Para.frameLen = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send4Para.frameLen = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send5Para.frameLen = *tempPoint;
//	tempPoint += sizeof(UINT8);
//	g_rtParams.FT3Send6Para.frameLen = *tempPoint;
//	tempPoint += sizeof(UINT8);
//
//	if(g_rtParams.FT3Send1Para.frameLen)
//	{
//		memcpy(g_rtParams.FT3Send1Para.frame,tempPoint,g_rtParams.FT3Send1Para.frameLen);
//		tempPoint += g_rtParams.FT3Send1Para.frameLen;
//	}
//	if(g_rtParams.FT3Send2Para.frameLen)
//	{
//		memcpy(g_rtParams.FT3Send2Para.frame,tempPoint,g_rtParams.FT3Send2Para.frameLen);
//		tempPoint += g_rtParams.FT3Send2Para.frameLen;
//	}
//	if(g_rtParams.FT3Send3Para.frameLen)
//	{
//		memcpy(g_rtParams.FT3Send3Para.frame,tempPoint,g_rtParams.FT3Send3Para.frameLen);
//		tempPoint += g_rtParams.FT3Send3Para.frameLen;
//	}
//	if(g_rtParams.FT3Send4Para.frameLen)
//	{
//		memcpy(g_rtParams.FT3Send4Para.frame,tempPoint,g_rtParams.FT3Send4Para.frameLen);
//		tempPoint += g_rtParams.FT3Send4Para.frameLen;
//	}
//	if(g_rtParams.FT3Send5Para.frameLen)
//	{
//		memcpy(g_rtParams.FT3Send5Para.frame,tempPoint,g_rtParams.FT3Send5Para.frameLen);
//		tempPoint += g_rtParams.FT3Send5Para.frameLen;
//	}
//	if(g_rtParams.FT3Send6Para.frameLen)
//	{
//		memcpy(g_rtParams.FT3Send6Para.frame,tempPoint,g_rtParams.FT3Send6Para.frameLen);
//		tempPoint += g_rtParams.FT3Send6Para.frameLen;
//	}
//
//	retValue = msgPackDefaultReply(1,TYPE609_CONT_FT3_FORMAT_WRITE,COMM_ACK_RIGHT, NULL);
//
//
//	if(retValue)
//		return COMM_returnData(sendData,sendSize);
//	else
		return 0;
}

UINT8 COMM_isFT3FormatRead(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16* sendSize )
{
	UINT8 retValue = 0;

	UINT8 *tempPoint = g_ControlPackSendBuf + sizeof(MUTestMsgHeader);

	*(UINT16*)tempPoint = g_rtParams.FT3Send1Para.sampCount;
	tempPoint += sizeof(UINT16);

	*tempPoint = g_rtParams.FT3Send1Para.mapCount;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send2Para.mapCount;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send3Para.mapCount;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send4Para.mapCount;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send5Para.mapCount;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send6Para.mapCount;
	tempPoint += sizeof(UINT8);

	memcpy(tempPoint,g_rtParams.FT3Send1Para.chMap,g_rtParams.FT3Send1Para.mapCount * sizeof(FT3_CH_Map_TYPE));
	tempPoint += g_rtParams.FT3Send1Para.mapCount * sizeof(FT3_CH_Map_TYPE);

	memcpy(tempPoint,g_rtParams.FT3Send2Para.chMap,g_rtParams.FT3Send2Para.mapCount * sizeof(FT3_CH_Map_TYPE));
	tempPoint += g_rtParams.FT3Send2Para.mapCount * sizeof(FT3_CH_Map_TYPE);

	memcpy(tempPoint,g_rtParams.FT3Send3Para.chMap,g_rtParams.FT3Send3Para.mapCount * sizeof(FT3_CH_Map_TYPE));
	tempPoint += g_rtParams.FT3Send3Para.mapCount * sizeof(FT3_CH_Map_TYPE);

	memcpy(tempPoint,g_rtParams.FT3Send4Para.chMap,g_rtParams.FT3Send4Para.mapCount * sizeof(FT3_CH_Map_TYPE));
	tempPoint += g_rtParams.FT3Send4Para.mapCount * sizeof(FT3_CH_Map_TYPE);


	memcpy(tempPoint,g_rtParams.FT3Send5Para.chMap,g_rtParams.FT3Send5Para.mapCount * sizeof(FT3_CH_Map_TYPE));
	tempPoint += g_rtParams.FT3Send5Para.mapCount * sizeof(FT3_CH_Map_TYPE);


	memcpy(tempPoint,g_rtParams.FT3Send6Para.chMap,g_rtParams.FT3Send6Para.mapCount * sizeof(FT3_CH_Map_TYPE));
	tempPoint += g_rtParams.FT3Send6Para.mapCount * sizeof(FT3_CH_Map_TYPE);

	*tempPoint = g_rtParams.FT3Send1Para.frameLen;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send2Para.frameLen;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send3Para.frameLen;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send4Para.frameLen;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send5Para.frameLen;
	tempPoint += sizeof(UINT8);
	*tempPoint = g_rtParams.FT3Send6Para.frameLen;
	tempPoint += sizeof(UINT8);


	memcpy(tempPoint,g_rtParams.FT3Send1Para.frame,g_rtParams.FT3Send1Para.frameLen);
	tempPoint += g_rtParams.FT3Send1Para.frameLen;

	memcpy(tempPoint,g_rtParams.FT3Send2Para.frame,g_rtParams.FT3Send2Para.frameLen);
	tempPoint += g_rtParams.FT3Send2Para.frameLen;

	memcpy(tempPoint,g_rtParams.FT3Send3Para.frame,g_rtParams.FT3Send3Para.frameLen);
	tempPoint += g_rtParams.FT3Send3Para.frameLen;

	memcpy(tempPoint,g_rtParams.FT3Send4Para.frame,g_rtParams.FT3Send4Para.frameLen);
	tempPoint += g_rtParams.FT3Send4Para.frameLen;

	memcpy(tempPoint,g_rtParams.FT3Send5Para.frame,g_rtParams.FT3Send5Para.frameLen);
	tempPoint += g_rtParams.FT3Send5Para.frameLen;

	memcpy(tempPoint,g_rtParams.FT3Send6Para.frame,g_rtParams.FT3Send6Para.frameLen);
	tempPoint += g_rtParams.FT3Send6Para.frameLen;

//	comSendPackHead->code = TYPE609_CONT_FT3_FORMAT_READ;
//	comSendPackHead->dataLeng = tempPoint - comSendPackDataStart;
//	retValue = COMM_returnData(sendData,sendSize);

	return retValue;
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

UINT8 COMM_isU32ParaWrite(UINT8 *netData,UINT16 netDataSize,UINT8** sendData,UINT16 *sendSize )
{
//	UINT8 retValue = 0;
//
//	UINT8 *tempPoint = netData;
//
//	tempPoint += 2;		//保留
//
//	UINT16 arraySize = *(UINT16*)tempPoint;
//	tempPoint += sizeof(UINT16);
//
//	if(netDataSize >= arraySize * (sizeof(UINT8) + sizeof(UINT32)) + 4)
//	{
//		UINT8 *tempIndexPoint = tempPoint;
//		UINT32 *tempValuePoint = (UINT32*)&tempPoint[sizeof(UINT8) * arraySize];
//
//		for(int i = 0 ; i < arraySize; i ++)
//		{
//			if(tempIndexPoint[i])
//			{
//				U32Parameter[tempIndexPoint[i]] = tempValuePoint[i];
//				U32ParaChange[tempIndexPoint[i]] = 1;		//标识参数变化
//			}
//		}
//		retValue = msgPackDefaultReply(1,TYPE609_CONT_UINT32_PAR_WRITE,COMM_ACK_RIGHT, NULL);
//	}else
//	{//长度错误
//		retValue = msgPackDefaultReply(0,TYPE609_CONT_UINT32_PAR_WRITE,COMM_ACK_ERROR, NULL);
//	}
//	if(retValue)
//		return COMM_returnData(sendData,sendSize);
//	else
		return 0;
}

UINT8 COMM_isU32ParaRead(UINT8 *netData,UINT16 netDataSize,UINT8** sendData, UINT16* sendSize )
{
	UINT8 retValue = 0;

//	UINT8 *tempPoint = comSendPackDataStart;
//
//	tempPoint += 2;		//保留
//
//	*(UINT16*)tempPoint = U32_PARAMETER_COUNT;
//	tempPoint += sizeof(UINT16);
//
//	memcpy(tempPoint,U32Parameter,U32_PARAMETER_COUNT * sizeof(UINT32));
//	tempPoint += (U32_PARAMETER_COUNT * sizeof(UINT32));
//
//	comSendPackHead->code = TYPE609_CONT_UINT32_PAR_READ;
//	comSendPackHead->dataLeng = tempPoint - comSendPackDataStart;
//	retValue = COMM_returnData(sendData,sendSize);

	return retValue;
}

UINT8 msgUnpackU8ParaWrite(UINT8 *netData,UINT16 netDataSize )
{
	UINT8 retValue = 0;

	UINT8 *tempPoint = netData;

	tempPoint += 2;		//保留

	UINT16 arraySize = *(UINT16*)tempPoint;
	tempPoint += sizeof(UINT16);
	UINT8 Index;

	if(netDataSize >= arraySize * (sizeof(UINT8) + sizeof(UINT8)) + 4)
	{
		UINT8 *tempIndexPoint = tempPoint;
		UINT8 *tempValuePoint = tempPoint + sizeof(UINT8) * arraySize;

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

	NetType  = NET_609_TRANSMIT;
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

		NetType  = NET_609_TRANSMIT;
		CmdCode = TYPE609_CONT_GOOSE_FORMAT_READ;
		DataLen = tempPoint - StartPos;

		return msgPackHeader ( OutBuf, MsgType, NetType, CmdCode, DataLen );

	}else
	{//端口错误
		return msgPackDefaultReply(0,TYPE609_CONT_GOOSE_FORMAT_READ,COMM_ACK_ERROR, NULL);
	}

}

UINT8 msgPackGooseDataWrite(UINT8 *netData,UINT16 netDataSize  )
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


UINT8 COMM_transmitData(UINT8 port,UINT32 sec,UINT32 nSec,UINT8* recvData,UINT16 recvDataSize,UINT8** sendData,UINT16* sendDataSize)
{
	UINT8 *sendBuf = 0;
//	switch(port)
//	{
//	case 0:
//		sendBuf = netRecv1DataSendBuf;
//		break;
//	case 1:
//		sendBuf = netRecv2DataSendBuf;
//		break;
//	case 2:
//		sendBuf = FT3Recv1DataSendBuf;
//		break;
//	case 3:
//		sendBuf = FT3Recv2DataSendBuf;
//		break;
//	case 4:
//		sendBuf = netGoose1SendBuf;
//		break;
//	case 5:
//		sendBuf = netGoose2SendBuf;
//		break;
//	default:
//		sendBuf = 0;
//		break;
//	}

	if(recvData && sendBuf)
	{
		MUTestMsgHeader *packHead = (MUTestMsgHeader*)sendBuf;

		UINT8 *dataStart = &sendBuf[sizeof(MUTestMsgHeader)];
		UINT8 *tempPoint = dataStart;

		*(UINT32*)tempPoint = sec;
		tempPoint += sizeof(UINT32);

		*(UINT32*)tempPoint = nSec;
		tempPoint += sizeof(UINT32);

		memcpy(tempPoint,recvData,recvDataSize);
		tempPoint += recvDataSize;

		packHead->dataLeng = tempPoint - dataStart;

		*sendData = sendBuf;
		*sendDataSize = packHead->dataLeng + sizeof(MUTestMsgHeader);

		return 1;
	}else
	{
		return 0;
	}
}


UINT8 COMM_transmitStandData(PTR_STAND_SAMP_TYPE data,UINT8** sendData,UINT16* sendDataSize,UINT8 asduNum,UINT8 currentAsdu)
{
	if(data && currentAsdu < asduNum)
	{
//		UINT8 *dataStart = &standardDataSendBuf[sizeof(MUTestMsgHeader)];
//		UINT8* tempPoint = dataStart;
//
//		*tempPoint = asduNum;
//		tempPoint ++;
//
//		tempPoint += 3;		//保留对齐
//
//		STAND_SAMP_TYPE *oneAsdu = (STAND_SAMP_TYPE*)tempPoint;
//
//		memcpy(&oneAsdu[currentAsdu],data,sizeof(STAND_SAMP_TYPE));
//
//		if(currentAsdu + 1 == asduNum)
//		{//填满了就发。
//			tempPoint += sizeof(STAND_SAMP_TYPE) * asduNum;
//
//			*sendData = standardDataSendBuf;
//			*sendDataSize = (tempPoint - dataStart) + sizeof(MUTestMsgHeader);
//			return 1;
//		}
	}

	return 0;
}



//only one ASDU for each SV frame
int PackSmvFrm( UINT8* OutBuf, UINT32 data[6], unsigned int SmpCnt, UINT8 Port )
{
	SMV_PROTOCOL_PARA *pSmvPara = 0;
	UINT8 uChNo = 0;
	UINT8 uAsduNo = 0;

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

	UINT32 IA, IB, IC, UA, UB, UC;

	IA   = data[0];
	IB   = data[1];
	IC   = data[2];

	UA   = data[3];
	UB   = data[4];
	UC   = data[5];


	UINT8 *smvBuf = pSmvPara->frame;
	UINT16 smvBufSize = pSmvPara->frameLen;

	//
	UINT16 cntIndex = pSmvPara->cntIndex[0];		//采样序号的位置
	UINT16 *cntPoint = (UINT16*)smvBuf + cntIndex;
	*cntPoint = netHostChangeS(SmpCnt);

	UINT16 ch1Index;
	SMV_92_CH_TYPE *chValuePoint = NULL;
	UINT8 cType;

	///
	ch1Index = pSmvPara->firstValueIndex[0];

	chValuePoint = (SMV_92_CH_TYPE*) (smvBuf+ ch1Index);

	for( uChNo = 0; uChNo < pSmvPara->chNum; uChNo++,chValuePoint++)
	{
		cType = pSmvPara->chType[uChNo];

		switch(cType)
		{
		case NEW609_CHTYPE_MUA:
			chValuePoint->value = UA;
			break;
		case NEW609_CHTYPE_MUB:
			chValuePoint->value = UB;
			break;
		case NEW609_CHTYPE_MUC:
			chValuePoint->value = UC;
			break;
		case NEW609_CHTYPE_MIA:
			chValuePoint->value = IA;
			break;
		case NEW609_CHTYPE_MIB:
			chValuePoint->value = IB;
			break;
		case NEW609_CHTYPE_MIC:
			chValuePoint->value = IC;
			break;
		case NEW609_CHTYPE_PUA:
			chValuePoint->value = UA;
			break;
		case NEW609_CHTYPE_PUB:
			chValuePoint->value = UB;
			break;
		case NEW609_CHTYPE_PUC:
			chValuePoint->value = UC;
			break;
		case NEW609_CHTYPE_PIA:
			chValuePoint->value = IA;
			break;
		case NEW609_CHTYPE_PIB:
			chValuePoint->value = IB;
			break;
		case NEW609_CHTYPE_PIC:
			chValuePoint->value = IC;
			break;
		case NEW609_CHTYPE_0:
			chValuePoint->value = 0;
			break;
		case NEW609_CHTYPE_CNT:
			chValuePoint->value = netHostChangeL(SmpCnt);
			break;
		case NEW609_CHTYPE_DELAY:
			chValuePoint->value = 1;//额定延时 需要测试
			break;
		case NEW609_CHTYPE_CRC_START:
			break;
		case NEW609_CHTYPE_CRC_STOP:
			break;
		}//switch

	}//for each channel

	// 换成MDMA
	memcpy(OutBuf, smvBuf, smvBufSize);

	return 1;
}
