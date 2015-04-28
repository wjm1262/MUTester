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
start：共享数据
*******************************/


typedef struct sRUNTIME_PARAMS
{

	SMV_PROTOCOL_PARA NetSend1SmvPara;		//点对点1发送参数
	SMV_PROTOCOL_PARA NetSend2SmvPara;		//点对点2发送参数

	FT3_PROTOCOL_PARA FT3Send1Para;
	FT3_PROTOCOL_PARA FT3Send2Para;
	FT3_PROTOCOL_PARA FT3Send3Para;
	FT3_PROTOCOL_PARA FT3Send4Para;
	FT3_PROTOCOL_PARA FT3Send5Para;
	FT3_PROTOCOL_PARA FT3Send6Para;

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

//初始化缓存
UINT8 COMM_initBuf();			//OK

//释放缓存
UINT8 COMM_clearBuf();			//OK



/*
功能：	转发相应数据到PC网口
参数：	port:	0，点对点1接收数据
				1，点对点2接收数据
				2，FT3输入1接收数据
				3，FT3输入2接收数据
				4，GOOSE发送1抄送数据
				5，GOOSE发送2抄送数据
				其它，无效
		sec,nSec，该帧接收时标。
		recvData，接收的数据指针，
		recvDataSize,接收的数据长度。
		sendData，发送数据缓存引用
		sendDataSize，发送数据长度
返回值：1，正确；0，不正确。返回1即将数据发送到PC通信口
说明：可重入
*/
UINT8 COMM_transmitData(UINT8 port,UINT32 sec,UINT32 nSec,UINT8* recvData,UINT16 recvDataSize,UINT8** sendData,UINT16* sendDataSize);

/*
功能：	发送标准采样数据
参数：	data，每一采样点的标准采样数据
		sendData，发送数据缓存引用
		sendDataSize，发送数据长度
		asduNum,多少点发一帧，注意跟4k要整除，比如10比较好
		currentAsdu，当前的点号，0~asduNum-1
返回值：1，正确；0，不正确。返回1即将数据发送到PC通信口
说明：不可重入，单线程调用
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
end：导出接口
*******************************/


#endif
