/*
 * xl-6004_forward_protocol.c
 *
 *  Created on: 2014-10-22
 *      Author: Wu JM
 */

#include "EMAC_TEST_Core0.h"
#include <stdio.h>
//#include "post_debug.h"
#include "xl-6004_forward_protocol.h"
#include "mutester_comm_protocol.h"

#include "msg.h"



#define MIN_ETHER_FRAME_LEN (64)
#define MAX_ETHER_FRAME_LEN (1500)

#define LDR_DATA_BUFFER_LOCATE_HEAP 2  //define the ldr data buffer area in heap.

uint8_t *g_pLdrDataBuff = NULL;//Global parameter, pointer of ldr data buffer

volatile int g_ACKOK_XMT_Completed = 0;

/*
 * 鎺у埗鍩�
	0x00锛氭帶鍒朵俊鎭抚锛岀敤浜�09涓嶱C绔繘琛屾帶鍒朵俊鎭殑閫氫俊锛屼笉杩涜浠讳綍杞彂鎿嶄綔
	0x01锛�09 to飪燩C绔紝鍙戦�瀹炴椂浠ュお缃戞帴鍙ｆ帴鏀跺埌鏁版嵁
	0x02锛�09 to PC绔紝鍙戦�鍏変覆鍙ｆ帴鏀舵暟鎹�
	0x03锛�09 to飪燩C绔紝鍙戦�寮�叧閲忚緭鍏ユ帴鍙ｆ暟鎹�

	0x81锛歅C绔儬 to 609锛氳浆鍙戞暟鎹埌瀹炴椂浠ュお缃戞帴鍙�
	0x82锛歅C绔儬 to 609锛氳浆鍙戞暟鎹埌鍏変覆鍙ｈ緭鍑�
 *
 * */
FORWARD_ETHER_FRAME board_info =
{
	0,
	{0x06, 0x05, 0x04, 0x03, 0x02, 0x01},
	0,
	0x05,
	BF609_FORWARD_SMV_PC,
	{BF609_FORWARD_SMV_TYPE_LO, BF609_FORWARD_SMV_TYPE_HI_BASE},
	{0}
};


static void PackForwardSMVFrmHeader ( void *pForwardFrmHeader, uint32_t unNanoSecond,
		uint16_t unPktDataLen )
{
	FORWARD_ETHER_FRAME* pHeader = (FORWARD_ETHER_FRAME*)pForwardFrmHeader;

	pHeader->NoBytes    = 14 + unPktDataLen;//only the frame size excluding 2 byte header
	pHeader->DestMAC[0] = board_info.DestMAC[0];
	pHeader->DestMAC[1] = board_info.DestMAC[1];
	pHeader->DestMAC[2] = board_info.DestMAC[2];
	pHeader->DestMAC[3] = board_info.DestMAC[3];
	pHeader->DestMAC[4] = board_info.DestMAC[4];
	pHeader->DestMAC[5] = board_info.DestMAC[5];

	pHeader->TimeStamp 	= unNanoSecond;

	pHeader->MUAddr 	= board_info.MUAddr; //
	pHeader->CtrlField 	= BF609_FORWARD_SMV_PC;//forward smv

	pHeader->LTfield[0] = BF609_FORWARD_SMV_TYPE_LO;
	pHeader->LTfield[1] = BF609_FORWARD_SMV_TYPE_HI_BASE + board_info.MUAddr;
}

/********************/
/* CreateForwardSMVFrame: copy the data from SMVFrame to a new buffer, so uses 'memcpy',
 * 						which is high time consumption.(for 30000ns)
 *
 * PackForwardSMVFrame: there are 14 bytes reserved at the head of original recv
 * 						buffer (see 'set_descriptor' ), so, only to need pack ForwardSMVFrmHeader
 * 						at the reserved room,which is low time consumption.(for 1600ns)
 */
//NOTES: if the SMVFrame more than 1500 bytes, then it is been cut off to 1500 bytes.
ADI_ETHER_BUFFER *CreateForwardSMVFrame ( uint32_t unNanoSecond, char *SMVFrame,
		uint16_t SmvFrmLen, ADI_ETHER_BUFFER *pXmtBuf)
{
	char *head, *data, *Dst;

	uint16_t  PayLoadLen = SmvFrmLen;
	uint16_t HeaderLen =14;
	ADI_ETHER_BUFFER *tx = pXmtBuf;

	if ( tx == NULL )
	{
		return NULL;
	}

	if( PayLoadLen > MAX_ETHER_FRAME_LEN )
	{
		//if the SMVFrame more than 1500 bytes, then it is been cut off to 1500 bytes.
		//DEBUG_PRINT ( "PackForwardSMVFrame: frame (Len:%d) more than MAX_ETHER_FRAME_LEN (Len:%d), cut off to 1500bytes.\n\n", SmvFrmLen, MAX_ETHER_FRAME_LEN );
		PayLoadLen = MAX_ETHER_FRAME_LEN;
	}

	// copy data from pbuf(s) into our buffer
	data = SMVFrame;
	head = ( char * ) tx->Data;
	PackForwardSMVFrmHeader ( head, unNanoSecond, PayLoadLen);

	// the first two bytes reserved for length
	Dst = ( char * ) tx->Data + 2 + HeaderLen;

	memcpy ( Dst, data, PayLoadLen );//

	tx->ElementCount = HeaderLen + PayLoadLen + 2; // total element count including 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}
/*
 * */
ADI_ETHER_BUFFER *PackForwardSMVFrame( uint32_t unNanoSecond,
										uint16_t SmvFrmLen,
										 ADI_ETHER_BUFFER *pXmtBuf)
{

	ADI_ETHER_BUFFER *tx = pXmtBuf;

	char *head;

	uint16_t  PayLoadLen = SmvFrmLen;
	uint16_t HeaderLen =14;

	// 	// the first two bytes reserved for length
	head = (char*)tx->Data;
	PackForwardSMVFrmHeader ( head, unNanoSecond, PayLoadLen);


	tx->ElementCount = HeaderLen + PayLoadLen + 2; // total element count including 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}
ADI_ETHER_BUFFER *PackSMVFrame( uint32_t unNanoSecond, uint16_t SmvFrmLen , ADI_ETHER_BUFFER *pXmtBuf)
{

	ADI_ETHER_BUFFER *tx = pXmtBuf;
	FORWARD_ETHER_FRAME* pHeader;

	char *head;

	uint16_t  PayLoadLen = SmvFrmLen;

	// 	// the first two bytes reserved for length
	head = (char*)tx->Data;
	pHeader = (FORWARD_ETHER_FRAME*)head;
	pHeader->NoBytes = PayLoadLen;



	tx->ElementCount = PayLoadLen + 2; // total element count including 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}



ADI_ETHER_BUFFER *PackACKFrmOfUpdateVerion ( BF609_COMM_ACK_CODE AckCode,
													void *pCtrlInfoFrmBuf,
													ADI_ETHER_BUFFER *pXmtBuf )

{

	ADI_ETHER_BUFFER *tx = pXmtBuf;
	FORWARD_ETHER_FRAME *pRxData, *pTxData; 	//Ethernet data ptr
	CONTROL_FRAME    *pCtrFrm;	//control data ptr

	unsigned short *ps;

	int  len;

	pRxData = (FORWARD_ETHER_FRAME *)pCtrlInfoFrmBuf;

	if ( tx == NULL )
	{
		return NULL;
	}

	// init header of ether frm, default it is ACK_OK
	pTxData = ( FORWARD_ETHER_FRAME * ) tx->Data;

	pTxData->DestMAC[0] = board_info.DestMAC[0];
	pTxData->DestMAC[1] = board_info.DestMAC[1];
	pTxData->DestMAC[2] = board_info.DestMAC[2];
	pTxData->DestMAC[3] = board_info.DestMAC[3];
	pTxData->DestMAC[4] = board_info.DestMAC[4];
	pTxData->DestMAC[5] = board_info.DestMAC[5];
	pTxData->MUAddr     = board_info.MUAddr;
	pTxData->CtrlField  = pRxData->CtrlField;//BF609_CTR;//0x00
	pTxData->LTfield[0] = BF609_UPDATE_VER_ACKOK_TYPE_LO;
	pTxData->LTfield[1] = BF609_UPDATE_VER_ACKOK_TYPE_HI;

	//init data of ether frm
	pTxData->PktData[0] =  0x68;
	pTxData->PktData[1] = 0x0a;
	pTxData->PktData[2] = 0x00;
	pTxData->PktData[3] =  0x68;
	pTxData->PktData[4] = 0x00;
	pTxData->PktData[5] = 0x10;// ACK_OK
	pTxData->PktData[6] =  0x01;//VERSION_UPDATE
	pTxData->PktData[7] = 0x00;
	pTxData->PktData[8] = 0x00;
	pTxData->PktData[9] =  0x16;

	if(AckCode != ACK_OK)
	{
		pTxData->LTfield[0] = BF609_UPDATE_VER_NAK_TYPE_LO;
		pTxData->LTfield[1] = BF609_UPDATE_VER_NAK_TYPE_HI;

		pTxData->PktData[5] = 0x80;// NAK
		pTxData->PktData[7] = AckCode;

	}

	tx->ElementCount = 14+10 + 2; // total element count including 2 byte header

	ps = ( unsigned short * ) tx->Data;
	*ps = tx->ElementCount - 2; // only the frame size excluding 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}

ADI_ETHER_BUFFER *PackACKFrmOfReadVersion ( void *pCtrlInfoFrmBuf, ADI_ETHER_BUFFER *pXmtBuf )
{

	ADI_ETHER_BUFFER *tx = pXmtBuf;
	FORWARD_ETHER_FRAME *pRxData, *pTxData; 	//Ethernet data ptr
	CONTROL_FRAME    *pCtrFrm;	//control data ptr

	unsigned short *ps;

	pRxData = (FORWARD_ETHER_FRAME *)pCtrlInfoFrmBuf;
	if ( tx == NULL )
	{
		return NULL;
	}

	// init header of ether frm, default it is ACK_OK
	pTxData = ( FORWARD_ETHER_FRAME * ) tx->Data;

	pTxData->DestMAC[0] = board_info.DestMAC[0];
	pTxData->DestMAC[1] = board_info.DestMAC[1];
	pTxData->DestMAC[2] = board_info.DestMAC[2];
	pTxData->DestMAC[3] = board_info.DestMAC[3];
	pTxData->DestMAC[4] = board_info.DestMAC[4];
	pTxData->DestMAC[5] = board_info.DestMAC[5];
	pTxData->MUAddr     = board_info.MUAddr;
	pTxData->CtrlField  = pRxData->CtrlField;//BF609_CTR;//0x00

	pTxData->LTfield[0] = BF609_READ_VER_TYPE_LO;
	pTxData->LTfield[1] = BF609_READ_VER_TYPE_HI;

	//init data of ether frm
	size_t len = strlen(VersionString);
	pTxData->PktData[0] =  0x68;
	pTxData->PktData[1] = 8+len;
	pTxData->PktData[2] = 0x00;
	pTxData->PktData[3] =  0x68;
	pTxData->PktData[4] = 0x00;
	pTxData->PktData[5] = VERSION_GET;//

	memcpy(&(pTxData->PktData[6]), VersionString, len );

	pTxData->PktData[6+len] =  0x00;//chksum, no use
	pTxData->PktData[7+len] = 0x16;

	tx->ElementCount = 14+ pTxData->PktData[1]  + 2; // total element count including 2 byte header

	ps = ( unsigned short * ) tx->Data;
	*ps = tx->ElementCount - 2; // only the frame size excluding 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}




