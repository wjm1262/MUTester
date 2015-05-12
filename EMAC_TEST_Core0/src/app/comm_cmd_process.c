/*
 * comm_cmd_process.c
 *
 *  Created on: 2015-4-22
 *      Author: Administrator
 */
#include <stddef.h>
#include "sys.h"

#include "comm_cmd_process.h"
#include "msg.h"
#include "post_debug.h"

UINT8 Comm_processCmd(UINT8 *recvData,UINT16 recvSize )
{
	UINT8 ret= 0;
	INT32 MsgLen;
	UINT16 Code;
	MUTestMsgHeader head ;

	msgUnpackHeader ( recvData, &head );

	if(recvSize < head.dataLeng + sizeof(MUTestMsgHeader))
		return 0;

	UINT8 *netData =  recvData + sizeof(MUTestMsgHeader);
	UINT16 netDataSize = head.dataLeng;

	if( head.netType == NET_609_CONCROL )
	{
		//是控制命令帧，在这里处理
		Code = head.code;

		switch( Code )
		{
		case TYPE609_CONT_SOFT_DOWN:

			break;

		case TYPE609_CONT_SOFT_VER:

			break;

		case TYPE609_CONT_SMV_FORMAT_WRITE:

			ret =  msgUnpackSmvFormatWrite( netData, netDataSize );
			if (ret)
			{
				MsgLen = msgPackDefaultReply(1,TYPE609_CONT_SMV_FORMAT_WRITE,COMM_ACK_RIGHT, NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,TYPE609_CONT_SMV_FORMAT_WRITE,COMM_ACK_ERROR, NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;

		case TYPE609_CONT_SMV_FORMAT_READ:

			MsgLen =  msgPackSmvFormatRead(netData, netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
//		case TYPE609_CONT_FT3_FORMAT_WRITE:
//			return COMM_isFT3FormatWrite(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_FT3_FORMAT_READ:
//			return COMM_isFT3FormatRead(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_VIRTUAL_WIRTE:
//			return COMM_isVirtualWrite(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_VIRTUAL_READ:
//			return COMM_isVirtualRead(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_UINT32_PAR_WRITE:
//			return COMM_isU32ParaWrite(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_UINT32_PAR_READ:
//			return COMM_isU32ParaRead(netData,netDataSize,sendData,sendSize);
////			break;
		case TYPE609_CONT_UINT8_PAR_WRITE:

			ret =  msgUnpackU8ParaWrite(netData,netDataSize);
			if(ret)
			{
				MsgLen = msgPackDefaultReply(1,TYPE609_CONT_UINT8_PAR_WRITE,COMM_ACK_RIGHT, NULL);

			}
			else
			{
				MsgLen = msgPackDefaultReply(0,TYPE609_CONT_UINT8_PAR_WRITE,COMM_ACK_ERROR, NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

//			if(U8ParaChange[U8PARA_NETIN1_TRANSTOPC])
//			{
//				U8ParaAction(0,U8PARA_NETIN1_TRANSTOPC,smvRecvThreadHandle1,smvRecvThread);
//				printf("点对点接收1转发到网络\r\n");
//			}
//			if(U8ParaChange[U8PARA_NETIN2_TRANSTOPC])
//			{
//				U8ParaAction(1,U8PARA_NETIN2_TRANSTOPC,smvRecvThreadHandle2,smvRecvThread);
//				printf("点对点接收2转发到网络\r\n");
//			}
//			if(U8ParaChange[U8PARA_FT3IN1_TRANSTOPC])
//			{
//				U8ParaAction(2,U8PARA_FT3IN1_TRANSTOPC,FT3RecvThreadHandle1,FT3RecvThread);
//				printf("FT3接收1转发到网络\r\n");
//			}
//			if(U8ParaChange[U8PARA_FT3IN2_TRANSTOPC])
//			{
//				U8ParaAction(3,U8PARA_FT3IN2_TRANSTOPC,FT3RecvThreadHandle2,FT3RecvThread);
//				printf("FT3接收2转发到网络\r\n");
//			}

//			if(U8ParaChange[U8PARA_FT3_SEND1])
//			{
//				U8ParaAction(4,U8PARA_FT3_SEND1,FT3SendThreadHandle,FT3SendThread);
//				printf("FT3发送1开始发送\r\n");
//			}

//			if(U8ParaChange[U8PARA_FT3_SEND1])
//			{
//				U8ParaAction(4,U8PARA_FT3_SEND1,FT3SendThreadHandle,FT3SendThread);
//				printf("FT3发送1开始发送\r\n");
//			}

//			if(U8ParaChange[U8PARA_NET_SEND1])
//			{
//				U8ParaAction(6,U8PARA_NET_SEND1,smvSendThreadHandle1,smvSendThread);
//				printf("点对点发送1开始发送\r\n");
//			}
//			if(U8ParaChange[U8PARA_NET_SEND2])
//			{
//				U8ParaAction(7,U8PARA_NET_SEND2,smvSendThreadHandle2,smvSendThread);
//				printf("点对点发送2开始发送\r\n");
//			}
//			if(U8ParaChange[U8PARA_STAND_TRANSTOPC])
//			{
//				U8ParaAction(8,U8PARA_STAND_TRANSTOPC,standardSendThreadHandle,standardSendThread);
//				printf("标准采样值发送到网络\r\n");
//			}
			break;

		case TYPE609_CONT_UINT8_PAR_READ:

			MsgLen =  msgPackU8ParaRead(netData,netDataSize);

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);
			break;

//		case TYPE609_CONT_YC_CODE:
//			return COMM_isYcCode(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_INTEGRITY_WRITE:
//			return COMM_isInterityWrite(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_INTEGRITY_READ:
//			return COMM_isInterityRead(netData,netDataSize,sendData,sendSize);
////			break;
		case TYPE609_CONT_GOOSE_FORMAT_WRITE:

			ret = msgUnpackGooseFormatWrite(netData,netDataSize);

			if(ret)
			{
				MsgLen = msgPackDefaultReply(1,TYPE609_CONT_GOOSE_FORMAT_WRITE,COMM_ACK_RIGHT, NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,TYPE609_CONT_GOOSE_FORMAT_WRITE,COMM_ACK_ERROR, NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
		case TYPE609_CONT_GOOSE_FORMAT_READ:

			MsgLen =  msgPackGooseFormatRead(netData,netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
		case TYPE609_CONT_GOOSE_DATA_WRITE:

			ret = msgUnpackGooseDataWrite(netData, netDataSize );

			if(ret == 1)
			{
				MsgLen = msgPackDefaultReply(1,TYPE609_CONT_GOOSE_DATA_WRITE,COMM_ACK_RIGHT, NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,TYPE609_CONT_GOOSE_DATA_WRITE,COMM_ACK_ERROR, NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
		default:
			return 0;
		}
	}
	else
	{
		DEBUG_PRINT("error NET TYPE:%0X.\n\n", head.netType);
	}
	return 0;
}
