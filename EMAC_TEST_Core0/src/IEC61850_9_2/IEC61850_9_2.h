#ifndef _IEC61850_9_2_H
#define _IEC61850_9_2_H

#include <stdint.h>

typedef unsigned char  UINT8;
typedef signed char           INT8;
typedef unsigned short UINT16;
typedef short          INT16;
typedef unsigned int   UINT32;
typedef int            INT32;

#define GOCBREF_VALUE_LEN 65
#define DATASET_VALUE_LEN 65



#pragma pack(1)
/*
* Ethernet frame head definition
*/
typedef struct Ethehead{

	UINT8  des[6];
	UINT8  src[6];
	//VLAN tag
	UINT16  TPID;
	UINT16  TCI;

	UINT16 EtheType;

	//APDU type
	UINT16 APPID;
	UINT16 FrameLen;// APPID 到帧结束 长度

	char reserver[4];

}FRAME_HEAD;//26-Bytes
#pragma pack()

/*
* ASDU info
*/
typedef struct ChannelData{
	INT32 value;
	INT32 quality;
}CHANNEL_DATA;
#define CHANNEL_NUM_MAX  180
#define ASDU_DATA_LEN_MAX  1000
typedef struct ASDU_Info{
	
	/*
		* ASDU tag value.
		*/
		char   SvID_Offset;//0x80
		UINT16  *pSvID;

		bool   bDataSet;
		UINT16   DataSet_Offset; //0x81
		char  *pDataSet;

		UINT16   SmpCnt_Offset; //0x82
		UINT16 SmpCnt;

		UINT16   ConfRev_Offset;//0x83
		UINT32 ConfRev;

		bool bRefrTm;
		UINT16 RefrTm_Offset;     //0x84
		UINT8 RefrTm[8];   /*64 bit, utcTime*/

		UINT16  SmpSync_Offset; //0x85
		UINT8 SmpSync;

		bool   bSmpRate;
		UINT16   SmpRate_Offset;//0x86
		UINT16 SmpRate;

		UINT16 Data_Offset;//0x87

		UINT8 DataChannelNum;
		CHANNEL_DATA ChannelData[CHANNEL_NUM_MAX];

		///ASDU offset
		UINT16 ASDU_Start;
		UINT16 ASDU_End;


}ASDU_INFO;

typedef struct IEC61850_9_2
{
	FRAME_HEAD FrameHead;
	bool       bSecurity;//是否含有security
	UINT8      ASDU_Num;
	ASDU_INFO *pASDU_info;
	char       SendBuff[1600];
	UINT16     FrameLen;
	INT16      offset;
}IEC61850_9_2;

extern IEC61850_9_2 *pIEC61850_9_2;

void Init_IEC_9_2(void);

void* Packet_9_2Frame(void *des , void *pAD_Value, uint8_t AD_ByteLength);

void* Packet_9_2Frame2( void *pAD_Value, uint8_t AD_ByteLength);
void Send_9_2Frame(void);


#endif
