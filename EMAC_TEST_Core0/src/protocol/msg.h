/*
 * msg.h
 *
 *  Created on: 2015-3-24
 *      Author: Administrator
 */

#ifndef MSG_H_
#define MSG_H_

#include "mutester_comm_protocol.h"

/*******************************
start����������
*******************************/


typedef struct sRUNTIME_PARAMS
{

	SMV_PROTOCOL_PARA NetSend1SmvPara;		//��Ե�1���Ͳ���
	SMV_PROTOCOL_PARA NetSend2SmvPara;		//��Ե�2���Ͳ���

	FT3_PROTOCOL_PARA FT3Send1Para;
	FT3_PROTOCOL_PARA FT3Send2Para;
	FT3_PROTOCOL_PARA FT3Send3Para;
	FT3_PROTOCOL_PARA FT3Send4Para;
	FT3_PROTOCOL_PARA FT3Send5Para;
	FT3_PROTOCOL_PARA FT3Send6Para;

	VIRTUAL_DATA_TYPE virtualData;					//���ⷢ��������,�����СΪVIRTUAL_DATA_SIZE

	UINT32 U32Parameter[U32_PARAMETER_COUNT];				//U32�������棬�����С��U32_PARAMETER_COUNT
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
extern UINT8 g_ControlPackSendBuf[MAX_COMM_PACK_LEN];			//���Ϳ���֡�Ļ�����
/*******************************
end����������
*******************************/

/*******************************
start�������ӿ�
*******************************/

//��ʼ������
UINT8 COMM_initBuf();			//OK

//�ͷŻ���
UINT8 COMM_clearBuf();			//OK



/*
���ܣ�	ת����Ӧ���ݵ�PC����
������	port:	0����Ե�1��������
				1����Ե�2��������
				2��FT3����1��������
				3��FT3����2��������
				4��GOOSE����1��������
				5��GOOSE����2��������
				��������Ч
		sec,nSec����֡����ʱ�ꡣ
		recvData�����յ�����ָ�룬
		recvDataSize,���յ����ݳ��ȡ�
		sendData���������ݻ�������
		sendDataSize���������ݳ���
����ֵ��1����ȷ��0������ȷ������1�������ݷ��͵�PCͨ�ſ�
˵����������
*/
UINT8 COMM_transmitData(UINT8 port,UINT32 sec,UINT32 nSec,UINT8* recvData,UINT16 recvDataSize,UINT8** sendData,UINT16* sendDataSize);

/*
���ܣ�	���ͱ�׼��������
������	data��ÿһ������ı�׼��������
		sendData���������ݻ�������
		sendDataSize���������ݳ���
		asduNum,���ٵ㷢һ֡��ע���4kҪ����������10�ȽϺ�
		currentAsdu����ǰ�ĵ�ţ�0~asduNum-1
����ֵ��1����ȷ��0������ȷ������1�������ݷ��͵�PCͨ�ſ�
˵�����������룬���̵߳���
*/
UINT8 COMM_transmitStandData(PTR_STAND_SAMP_TYPE data,UINT8** sendData,UINT16* sendDataSize,UINT8 asduNum,UINT8 currentAsdu);


UINT16 netHostChangeS(UINT16 netshort);


UINT32 netHostChangeL(UINT32 netlong);

void msgUnpackHeader ( UINT8 *InBuf, MUTestMsgHeader *header );

int msgPackHeader ( UINT8 *OutBuf, UINT8 MsgType, UINT16 NetType,
									UINT16 CmdCode, UINT16 DataLeng );

int msgPackForwardFrm ( UINT8 *OutBuf, UINT16 CmdCode, UINT16 DataLeng,
									UINT32 sec,UINT32 nanoSec );

int msgPackStandADFrm(UINT8* pOutBuf,  UINT8 asduNum);

int msgPackDefaultReply(UINT8 isRight, UINT16 order, UINT16 errorCode, char *info );

UINT8 msgUnpackSmvFormatWrite(UINT8 *netData,UINT16 netDataSize);
int msgPackSmvFormatRead(UINT8 *netData, UINT16 netDataSize );

UINT8 msgUnpackU8ParaWrite(UINT8 *netData,UINT16 netDataSize );
INT32 msgPackU8ParaRead(UINT8 *netData,UINT16 netDataSize );

UINT8 msgUnpackGooseFormatWrite(UINT8 *netData,UINT16 netDataSize );
INT32 msgPackGooseFormatRead(UINT8 *netData,UINT16 netDataSize  );
UINT8 msgPackGooseDataWrite(UINT8 *netData,UINT16 netDataSize  );

//////////////////////////
int PackSmvFrm( UINT8* OutBuf, UINT32 data[6], unsigned int SmpCnt, UINT8 Port );


/*******************************
end�������ӿ�
*******************************/


#endif
