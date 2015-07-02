/*
 * comm_pc_protocol.h
 *
 *  Created on: 2015-3-24
 *      Author: Wu JM
 */

#ifndef COMM_PC_PROTOCOL_H_
#define COMM_PC_PROTOCOL_H_

//typedef unsigned long long UINT64;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;
typedef int INT32;
typedef short int INT16;
typedef signed char INT8;
typedef unsigned short WORD16;
typedef float FLOAT32;
typedef double FLOAT64;

#define MSG_HEADER_LEN (14+6)
#define MSG_FORWARD_FRM_HEADER_LEN (8)

#define MAX_ASDU_NUM			8			//���ASDU����
#define MAX_CH_NUM				64			//���ͨ����
#define MAX_SMV_PACK_LEN		1400		//SMV֡��󳤶�
#define MAX_FT3_CH_NUM			22			//FT3���ͨ����
#define MAX_FT3_PACK_LEN		100			//FT3���֡����
#define MAX_GOOSE_PACK_LEN		1400		//GOOSE֡��󳤶�
#define MAX_KZ_CACHE_COUNT		256			//����������������
#define MAX_COMM_PACK_LEN		1514		//��PCͨ�ŵ�֡��󳤶�


#define VIRTUAL_DATA_SIZE		1400		//�������ݷ��������ô�С
#define U32_PARAMETER_COUNT		256			//U32��������
#define U8_PARAMETER_COUNT		256

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define M_2PI		6.28318530717958647692
#define M_SQRT2		1.41421356237309504880

//ʱ���ʾ����
#define SECOND_MICROSECOND	1000000			//����΢���ʾ
#define SECOND_NANOSECOND	1000000000		//���������ʾ
#define MICROSECOND_NANOSECOND	1000		//΢���������ʾ
#define SECOND_MILLISECOND	1000			//���ú����ʾ
#define MILLISECOND_MICROSECOND	1000		//������΢���ʾ

#pragma pack(1)
/*start 609ͨ��Э���������Ͷ���*/
typedef struct
{
	//֡ͷ��ʽ
	UINT8 descMac[6];
	UINT8 sourMac[6];
	UINT16 netType;		//��������,Ĭ��Ϊ0x01,0x33,ͨ�ſ�����0x01,0x34
	UINT16 code;		//�������
	UINT16 unDef;		//����
	UINT16 dataLeng;	//���ݳ���
}MUTestMsgHeader;


typedef struct
{
	UINT32 softVer;
	UINT32 hardVer;
	UINT32 otherVer1;
	UINT32 otherVer2;
	UINT32 otherVer3;
	UINT32 otherVer4;
}NEW609VER_TYPE,*pNEW609VER_TYPE,*ppNEW609VER_TYPE;

typedef struct
{//���õ�Ե㷢��֡��ʽ
	UINT8 proType;		//Э�����ͣ�0��-9-2��1��-9-1
	UINT8 port;			//0:��Ч�� port=1����Ե�1��port=2����Ե�2.
	UINT8 asduNum;	//ASDU��
	UINT8 chNum;		//ͨ������
	UINT16 sampCount;	//������
	UINT16 undef2;
}SMV_PROTOCOL_PARA_HEAD,*PTR_SMV_PROTOCOL_PARA_HEAD;

typedef struct
{
	UINT8 proType;							//Э�����ͣ�0��-9-2��1��-9-1
	UINT8 undef;							//
	UINT8 asduNum;							//ASDU��
	UINT8 chNum;							//ͨ������
	UINT8 chType[MAX_CH_NUM];				//ͨ�����Ͷ���
	UINT16 sampCount;						//������
	UINT16 frameLen;						//֡����
	UINT16 cntIndex[MAX_ASDU_NUM];			//���λ��
	UINT16 firstValueIndex[MAX_ASDU_NUM];	//�׸�ͨ��λ��
	UINT8 frame[MAX_SMV_PACK_LEN];			//֡
}SMV_PROTOCOL_PARA,*PTR_SMV_PROTOCOL_PARA;

typedef struct
{
	UINT8 index;	//֡��ƫ��
	UINT8 chType;	//����
}FT3_CH_Map_TYPE,*pFT3_CH_Map_TYPE;

typedef struct
{
	UINT8 mapCount;
	UINT8 frameLen;
	UINT16 sampCount;
	UINT8 frame[MAX_FT3_PACK_LEN];
	FT3_CH_Map_TYPE chMap[MAX_FT3_CH_NUM];		//����ӳ��,?����᲻��ṹ�屻4�ֽڶ���
}FT3_PROTOCOL_PARA,*pFT3_PROTOCOL_PARA;

typedef struct
{
	UINT8 undef1;
	UINT8 type;
	UINT16 paraSize;
	UINT8 buf[VIRTUAL_DATA_SIZE];
}VIRTUAL_DATA_TYPE,*pVIRTUAL_DATA_TYPE;

typedef struct
{//�������������
	UINT16 DataItem;	//��λ0~7��ʾ��ʾ8·�������1��ʾ������
	UINT16 Value;	//��λ 0x01�� ��   0x02: ��  ����������
	UINT32 Sec;	//ʱ����ֵ
	UINT32 nSec;	//ʱ������ֵ
}KZ_VALUE_TYPE,*pKZ_VALUE_TYPE;

typedef struct
{
	UINT8 kzz_0;
	UINT8 kzz_1;
	UINT16 kz_Size;
	UINT32 undef;
	KZ_VALUE_TYPE kz[MAX_KZ_CACHE_COUNT];
}KZ_PARAMETER_TYPE,*pKZ_PARAMETER_TYPE;

typedef struct
{//�������¼�����
	UINT8 loseType;		//0�ޣ���������1�����֡��2���㶪֡����������
	UINT8 loseMode;		//0�ظ���֡��1һ�ζ�֡
	UINT8 ranType;			//�����֡��ʽ��0��ɢ��1����
	UINT8 nofDef1;			//0��δ����
	UINT16 startCnt;		//��ʼ���
	UINT16 stopCnt;		//ֹͣ���
	FLOAT32 loseRadio;		//��֡�ʣ�0.1��ʾ10%
}LOSE_TYPE,*pLOSE_TYPE;

typedef struct
{
	UINT8 StNumType;			//Ĭ��0x05��UINT32�����Ȳ����ֱ���
	UINT8 SqNumType;			//Ĭ��0x05��UINT32�����Ȳ����ֱ���
	UINT16 timeLiveIndex;		//��Чʱ����֡�е�ƫ��λ��
	UINT16 actiomTimeIndex;		//��λʱ����֡�е�ƫ��λ��
	UINT16 StNumIndex;			//StNum��֡�е�ƫ��λ��
	UINT16 SqNumIndex;			//SqNum��֡�е�ƫ��λ��
	UINT16 allDataIndex;		//��������֡�е�ƫ��λ��
	UINT16 frameLen;			//����֡����
	UINT16 undef;				//δ����
	UINT32 T0;					//�ش�ʱ��,��λuS
	UINT32 T1;					//�ش�ʱ��,��λuS
	UINT32 T2;					//�ش�ʱ��,��λuS
	UINT32 T3;					//�ش�ʱ��,��λuS
	UINT32 undef1;
	UINT32 undef2;
	UINT32 undef3;
	UINT8 frame[MAX_GOOSE_PACK_LEN];
}GOOSE_PARA_TYPE,*PTR_GOOSE_PARA_TYPE;

typedef struct
{
	UINT8 asduNum;		//������Ԫ��
	UINT8 undef1;		//δ����
	UINT16 undef2;		//δ����
}STAND_SAMP_HEAD_TYPE,*pSTAND_SAMP_HEAD_TYPE;

typedef struct
{
	UINT32 value;
	UINT32 pinzi;
}SMV_92_CH_TYPE,*pSMV_92_CH_TYPE;


typedef struct
{
	UINT16 stateLabel;		//״̬��ʶ
	UINT16 sampCnt;			//�������
	UINT32 sampTMark;		//������ʱ
	UINT32 netSendTMark;	//��Ե㷢����ʱ
	UINT32 FT3SendTMark;	//FT3������ʱ
	INT32 UaSampData;		//UA����ֵ
	INT32 UbSampData;		//UB����ֵ
	INT32 UcSampData;		//UC����ֵ
	INT32 IaSampData;		//IA����ֵ
	INT32 IbSampData;		//IB����ֵ
	INT32 IcSampData;		//IC����ֵ
}STAND_SAMP_TYPE,*PTR_STAND_SAMP_TYPE;
#define STAND_SAMP_COUNT_EACH_PKT 10

#pragma pack()

/*end 609ͨ��Э���������Ͷ���*/


UINT16 netHostChangeS(UINT16 netshort);
UINT32 netHostChangeL(UINT32 netlong);

// ��Ϣ���Ͷ��壬����ΪDestMAC�����һ���ֽڣ������ϲ�ץ������
#define MSG_FORWARD_FRM_TYPE 0X80
#define MSG_CONTROL_FRM_TYPE 0X81




//���籨�����Ͷ���
#define NET_IEC61850  0x88ba
#define NET_GOOSE  0x88B8
#define NET_VLAN  0x8100
//#define NET_CONCROL  0x01FF
#define NET_609_TRANSMIT  0x0633
#define NET_609_CONCROL  0x0634

/*********************modify by wjm@2015-6-4**************************/
//��609ת����ͨ������
#define	NEW609_CHTYPE_BASE		 0x00

#define	NEW609_CHTYPE_MUA		(NEW609_CHTYPE_BASE + 0)
#define	NEW609_CHTYPE_MUB		(NEW609_CHTYPE_BASE + 1)
#define	NEW609_CHTYPE_MUC		(NEW609_CHTYPE_BASE + 2)
#define	NEW609_CHTYPE_MIA		(NEW609_CHTYPE_BASE + 3)
#define	NEW609_CHTYPE_MIB		(NEW609_CHTYPE_BASE + 4)
#define	NEW609_CHTYPE_MIC		(NEW609_CHTYPE_BASE + 5)
#define	NEW609_CHTYPE_PUA		(NEW609_CHTYPE_BASE + 6)
#define	NEW609_CHTYPE_PUB		(NEW609_CHTYPE_BASE + 7)
#define	NEW609_CHTYPE_PUC		(NEW609_CHTYPE_BASE + 8)
#define	NEW609_CHTYPE_PIA		(NEW609_CHTYPE_BASE + 9)
#define	NEW609_CHTYPE_PIB		(NEW609_CHTYPE_BASE + 10)
#define	NEW609_CHTYPE_PIC		(NEW609_CHTYPE_BASE + 11)

#define NEW609_CHTYPE_0			(NEW609_CHTYPE_BASE + 12)
#define	NEW609_CHTYPE_DELAY		(NEW609_CHTYPE_BASE + 13)
#define	NEW609_CHTYPE_CNT		(NEW609_CHTYPE_BASE + 14)
/********************************************************************/
#define	NEW609_CHTYPE_CRC_START	 0x80			//У������ʼ��ʶ
#define	NEW609_CHTYPE_CRC_STOP	 0xA0			//У���������ʶ


//��609ת����֡���Ͷ��壬������Ķ���
#define TYPE609_CONT_RIGHT_REPLY			 0x0000		//��ȷӦ��
#define TYPE609_CONT_ERROR_REPLY			 0x0001		//����Ӧ��
#define TYPE609_CONT_SOFT_DOWN				 0x0002		//��������
#define TYPE609_CONT_SOFT_VER				 0x0003		//��ȡ����汾��Ϣ
#define TYPE609_CONT_SMV_FORMAT_WRITE		 0x0004		//���õ�Ե㷢��֡��ʽ
#define TYPE609_CONT_SMV_FORMAT_READ		 0x0005		//��ȡ��Ե㷢��֡��ʽ
#define TYPE609_CONT_FT3_FORMAT_WRITE		 0x0006		//����FT3���ݷ��͸�ʽ
#define TYPE609_CONT_FT3_FORMAT_READ		 0x0007		//��ȡFT3���ݷ��͸�ʽ
#define TYPE609_CONT_VIRTUAL_WIRTE			 0x0008		//�����������ݷ���������
#define TYPE609_CONT_VIRTUAL_READ			 0x0009		//��ȡ�������ݷ���������
#define TYPE609_CONT_UINT32_PAR_WRITE		 0x000A		//����U32����������Ϣ
#define TYPE609_CONT_UINT32_PAR_READ		 0x000B		//��ȡU32����������Ϣ
#define TYPE609_CONT_UINT8_PAR_WRITE		 0x000C		//����U8����������Ϣ
#define TYPE609_CONT_UINT8_PAR_READ			 0x000D		//��ȡU8����������Ϣ
#define TYPE609_CONT_YC_CODE				 0x000E		//�������������
#define TYPE609_CONT_INTEGRITY_WRITE		 0x0010		//�����������¼�
#define TYPE609_CONT_INTEGRITY_READ			 0x0011		//��ȡ�������¼�
#define TYPE609_CONT_GOOSE_FORMAT_WRITE		 0x0012		//����GOOSE֡���͸�ʽ
#define TYPE609_CONT_GOOSE_FORMAT_READ		 0x0013		//��ȡGOOSE֡���͸�ʽ
#define TYPE609_CONT_GOOSE_DATA_WRITE		 0x0014

#define TYPE609_CONT_NET_RECV1_DATA			 0x0301		//ת����Ե�1��̫���ӿڽ�������
#define TYPE609_CONT_NET_RECV2_DATA			 0x0302		//ת����Ե�2��̫���ӿڽ�������

#define TYPE609_CONT_FT3_RECV1_DATA			 0x0303		//ת��FT3����1��������
#define TYPE609_CONT_FT3_RECV2_DATA			 0x0304		//ת��FT3����2��������

#define TYPE609_CONT_GOOSE_SEND1_DATA		 0x0305		//���͵�Ե�1GOOSE��������
#define TYPE609_CONT_GOOSE_SEND2_DATA		 0x0306		//���͵�Ե�2GOOSE��������

#define TYPE609_CONT_STANDARD_DATA			 0x0307		//���ͱ�׼��������
#define TYPE609_CONT_YX_DATA				 0x0308		//�����������¼��ϱ�
#define TYPE609_CONT_TIME_ERROR_DATA		 0x0309		//ʱ�������Ϣ�ϱ�
#define TYPE609_CONT_YC_DATA				 0x030A		//����������ϱ�

#define TYPE609_CONT_NO_REPLY				 0xFFFF		//��Ӧ��


//start 609ͨ�Ŵ�����붨��
#define COMM_ACK_ERROR					0x0000
#define COMM_ACK_RIGHT					0x0001		//��ȷ
//end 609ͨ�Ŵ�����붨��

//start 609��U32�����±궨��
#define U32PARA_UNDEF						0			//��Ч
#define U32PARA_METER_VOL_RADIO				1			//������ѹ����ϵ��
#define U32PARA_METER_CUR_RADIO				2			//������������ϵ��
#define U32PARA_PRO_VOL_RADIO				3			//������ѹ����ϵ��
#define U32PARA_PRO_CUR_RADIO				4			//������������ϵ��
#define U32PARA_VIR_UA_AMP					5			//����UA��ֵ
#define U32PARA_VIR_UB_AMP					6			//����UB��ֵ
#define U32PARA_VIR_UC_AMP					7			//����UC��ֵ
#define U32PARA_VIR_IA_AMP					8			//����IA��ֵ
#define U32PARA_VIR_IB_AMP					9			//����IB��ֵ
#define U32PARA_VIR_IC_AMP					10			//����IC��ֵ
#define U32PARA_VIR_UA_PHA					11			//����UA��λ
#define U32PARA_VIR_UB_PHA					12
#define U32PARA_VIR_UC_PHA					13
#define U32PARA_VIR_IA_PHA					14
#define U32PARA_VIR_IB_PHA					15
#define U32PARA_VIR_IC_PHA					16
#define U32PARA_VIR_UA_FRE					17			//����UAƵ��
#define U32PARA_VIR_UB_FRE					18
#define U32PARA_VIR_UC_FRE					19
#define U32PARA_VIR_IA_FRE					20
#define U32PARA_VIR_IB_FRE					21
#define U32PARA_VIR_IC_FRE					22
#define U32PARA_VIR_RATE_DELAY1				23			//���������ʱ1
#define U32PARA_VIR_RATE_DELAY2				24			//���������ʱ2
#define U32PARA_CURRENT_SEC					25			//ʱ��-��������609�����ڲ���ʱ
#define U32PARA_CURRENT_NSEC				26			//ʱ��-���룬��609�����ڲ���ʱ
#define U32PARA_LASER_POWER1				27			//���1�⹦��
#define U32PARA_LASER_POWER2				28			//��Ʒ�⹦��

//end 609��U32�����±궨��

//start 609��U8�����±궨��
#define U8PARA_UNDEF						0				//��Ч
#define U8PARA_SAMP_TYPE					1				//����ģʽ��0��AD������1����������
#define U8PARA_VALUE_TYPE					2				//ģ�����������ͣ�0��ģ�������룬1��С�ź�������
#define U8PARA_UA_GEAR						3				//UA��λ
#define U8PARA_UB_GEAR						4				//
#define U8PARA_UC_GEAR						5				//
#define U8PARA_IA_GEAR						6				//
#define U8PARA_IB_GEAR						7				//
#define U8PARA_IC_GEAR						8				//
#define U8PARA_NET_RECV1					9				//��Ե�1����
#define U8PARA_NET_RECV2					10				//
#define U8PARA_FT3_RECV1					11				//FT3����1����
#define U8PARA_FT3_RECV2					12				//
#define U8PARA_NET_SEND1					13				//��Ե�1����
#define U8PARA_NET_SEND2					14				//
#define U8PARA_FT3_SEND1					15				//FT3���1����
#define U8PARA_FT3_SEND2					16				//
#define U8PARA_FT3_SEND3					17				//
#define U8PARA_FT3_SEND4					18				//
#define U8PARA_FT3_SEND5					19				//
#define U8PARA_FT3_SEND6					20				//
#define U8PARA_YX_RECV						21				//����������
#define U8PARA_YC_SEND						22				//���������
#define U8PARA_STAND_TRANSTOPC				23				//��׼ֵ���͵�����
#define U8PARA_NETIN1_TRANSTOPC				24				//��Ե�1���뷢�͵�����
#define U8PARA_NETIN2_TRANSTOPC				25				//
#define U8PARA_FT3IN1_TRANSTOPC				26				//FT3����1����������
#define U8PARA_FT3IN2_TRANSTOPC				27				//
#define U8PARA_YC_COUNT						28				//�������������
#define U8PARA_YX_COUNT						29				//�������������
#define U8PARA_TIME_Y						30				//��ʱ-��
#define U8PARA_TIME_M						31				//��ʱ-��
#define U8PARA_TIME_d						32				//��ʱ-��
#define U8PARA_TIME_HH						33				//��ʱ-ʱ
#define U8PARA_TIME_MM						34				//��ʱ-��
#define U8PARA_TIME_SS						35				//��ʱ-��
#define U8PARA_SYN_STATE	         			36				//ͬ������״̬
#define U8PARA_TIME_ERR						37				//����ʱ�����
#define U8PARA_SYN_TYPE						38				//ͬ����ʽ
#define U8PARA_SYN_IN_PORT					39				//ͬ��ʱ�������
#define U8PARA_MU_TIME_IN_PORT				40				//Muʱ�������
#define U8PARA_SYN_IN_TYPE					41				//ͬ���ź���������
#define U8PARA_SYN_OUT_TYPE					42				//ͬ���ź��������
#define U8PARA_SYN_OUT_STOP					43				//ͬ���ź����ֹͣ
#define U8PARA_1588_IN_PORT					44				//1588���Ӷ˿�
#define U8PARA_SYN_IN1_LIGHT_BACK			45				//��ʱ������1ȡ��
#define U8PARA_SYN_IN1_ELE_BACK				46				//��ʱ������1ȡ��
#define U8PARA_SYN_IN2_LIGHT_BACK			47				//��ʱ������2ȡ��
#define U8PARA_SYN_IN2_ELE_BACK				48				//��ʱ������2ȡ��
#define U8PARA_SYN_OUT_BACK					49				//ͬ�����ȡ��
#define U8PARA_SYN_OUT_IRIG_B_CHECK			50				//���B��У��λ
#define U8PARA_FT3_OUT_LOGIC				51				//FT3�����߼�
#define U8PARA_FT3_IN_LOGIC					52				//FT3�����߼�
#define U8PARA_FT3_OUT_MODE					53				//FT3����ģʽ
#define U8PARA_FT3_IN_MODE					54				//FT3����ģʽ
#define U8PARA_FT3_OUT_SPEED				55				//FT3��������
#define U8PARA_FT3_IN_SPEED					56				//FT3��������


//end 609��U8�����±궨��

#endif /* MUTESTER_COMM_PROTOCOL_H_ */
