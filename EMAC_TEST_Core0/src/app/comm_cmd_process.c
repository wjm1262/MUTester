/*
 * comm_cmd_process.c
 *
 *  Created on: 2015-4-22
 *      Author: Wu JM
 */
#include <stddef.h>
#include "sys.h"

#include "comm_cmd_process.h"
#include "msg.h"

#include "VerUpgrade.h"

#include "post_debug.h"
extern void VerUpgrade(void);

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

			ret = msgUnPackSoftWareVersionUpdate( netData, netDataSize );

			if( ACK_FRM_OK == ret  )
			{
				// do nothing
			}
			else if( ACK_OK == ret )
			{
				//
				MsgLen = PackSoftWareVersionUpdateAckFrm(1,	ret);
				MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

				// update and reset
				VerUpgrade();
			}
			else
			{
				//NAK
				MsgLen = PackSoftWareVersionUpdateAckFrm(0,	ret);
				MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);
			}

			break;

		case TYPE609_CONT_SOFT_VER:

			MsgLen =  msgPackSoftWareVersionRead( netData, netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;

		case TYPE609_CONT_SMV_FORMAT_WRITE:

			ret =  msgUnpackSmvFormatWrite( netData, netDataSize );
			if (ret)
			{
				MsgLen = msgPackDefaultReply(1,
							TYPE609_CONT_SMV_FORMAT_WRITE,
							COMM_ACK_RIGHT,
							NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,
							TYPE609_CONT_SMV_FORMAT_WRITE,
							COMM_ACK_ERROR,
							NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;

		case TYPE609_CONT_SMV_FORMAT_READ:

			MsgLen =  msgPackSmvFormatRead(netData, netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
		case TYPE609_CONT_FT3_FORMAT_WRITE:

			msgUnpackFT3FormatWrite( netData, netDataSize );

			MsgLen = msgPackDefaultReply(1,
						TYPE609_CONT_FT3_FORMAT_WRITE,
						COMM_ACK_RIGHT,
						NULL);

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;

		case TYPE609_CONT_FT3_FORMAT_READ:

			MsgLen =  msgPackFT3FormatRead(netData, netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
//		case TYPE609_CONT_VIRTUAL_WIRTE:
//			return COMM_isVirtualWrite(netData,netDataSize,sendData,sendSize);
////			break;
//		case TYPE609_CONT_VIRTUAL_READ:
//			return COMM_isVirtualRead(netData,netDataSize,sendData,sendSize);
////			break;
		case TYPE609_CONT_UINT32_PAR_WRITE:

			ret =  msgUnpackU32ParaWrite( netData, netDataSize );
			if (ret)
			{
				MsgLen = msgPackDefaultReply(1,
							TYPE609_CONT_UINT32_PAR_WRITE,
							COMM_ACK_RIGHT,
							NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,
							TYPE609_CONT_UINT32_PAR_WRITE,
							COMM_ACK_ERROR,
							NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);
			break;

		case TYPE609_CONT_UINT32_PAR_READ:

			MsgLen = msgPackU32ParaRead(netData, netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);
			break;

		case TYPE609_CONT_UINT8_PAR_WRITE:

			ret =  msgUnpackU8ParaWrite(netData,netDataSize);
			if(ret)
			{
				MsgLen = msgPackDefaultReply(1,
							TYPE609_CONT_UINT8_PAR_WRITE,
							COMM_ACK_RIGHT,
							NULL);

			}
			else
			{
				MsgLen = msgPackDefaultReply(0,
							TYPE609_CONT_UINT8_PAR_WRITE,
							COMM_ACK_ERROR,
							NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

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

			if(ret )
			{
				MsgLen = msgPackDefaultReply(1,
							TYPE609_CONT_GOOSE_FORMAT_WRITE,
							COMM_ACK_RIGHT,
							NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,
							TYPE609_CONT_GOOSE_FORMAT_WRITE,
							COMM_ACK_ERROR,
							NULL);
			}

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			//process init event

			break;
		case TYPE609_CONT_GOOSE_FORMAT_READ:

			MsgLen =  msgPackGooseFormatRead(netData,netDataSize );

			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			break;
		case TYPE609_CONT_GOOSE_DATA_WRITE:

			ret = msgUnpackGooseDataWrite(netData, netDataSize );

			if(ret )
			{
				MsgLen = msgPackDefaultReply(1,
							TYPE609_CONT_GOOSE_DATA_WRITE,
							COMM_ACK_RIGHT,
							NULL);
			}
			else
			{
				MsgLen = msgPackDefaultReply(0,
							TYPE609_CONT_GOOSE_DATA_WRITE,
							COMM_ACK_ERROR,
							NULL);
			}
			MuTesterSystem.Device.exEth.EthSend ( g_ControlPackSendBuf, MsgLen);

			//process value change event


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
