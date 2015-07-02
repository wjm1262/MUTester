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

#define MAX_ASDU_NUM			8			//最大ASDU个数
#define MAX_CH_NUM				64			//最大通道数
#define MAX_SMV_PACK_LEN		1400		//SMV帧最大长度
#define MAX_FT3_CH_NUM			22			//FT3最大通道数
#define MAX_FT3_PACK_LEN		100			//FT3最大帧长度
#define MAX_GOOSE_PACK_LEN		1400		//GOOSE帧最大长度
#define MAX_KZ_CACHE_COUNT		256			//开关量输出缓存个数
#define MAX_COMM_PACK_LEN		1514		//与PC通信的帧最大长度


#define VIRTUAL_DATA_SIZE		1400		//虚拟数据发生器配置大小
#define U32_PARAMETER_COUNT		256			//U32参数个数
#define U8_PARAMETER_COUNT		256

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define M_2PI		6.28318530717958647692
#define M_SQRT2		1.41421356237309504880

//时间表示定义
#define SECOND_MICROSECOND	1000000			//秒用微秒表示
#define SECOND_NANOSECOND	1000000000		//秒用纳秒表示
#define MICROSECOND_NANOSECOND	1000		//微秒用纳秒表示
#define SECOND_MILLISECOND	1000			//秒用毫秒表示
#define MILLISECOND_MICROSECOND	1000		//毫秒用微秒表示

#pragma pack(1)
/*start 609通信协议数据类型定义*/
typedef struct
{
	//帧头格式
	UINT8 descMac[6];
	UINT8 sourMac[6];
	UINT16 netType;		//网络类型,默认为0x01,0x33,通信控制是0x01,0x34
	UINT16 code;		//命令编码
	UINT16 unDef;		//保留
	UINT16 dataLeng;	//数据长度
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
{//设置点对点发送帧格式
	UINT8 proType;		//协议类型，0：-9-2，1：-9-1
	UINT8 port;			//0:无效， port=1：点对点1，port=2：点对点2.
	UINT8 asduNum;	//ASDU数
	UINT8 chNum;		//通道总数
	UINT16 sampCount;	//采样率
	UINT16 undef2;
}SMV_PROTOCOL_PARA_HEAD,*PTR_SMV_PROTOCOL_PARA_HEAD;

typedef struct
{
	UINT8 proType;							//协议类型，0：-9-2，1：-9-1
	UINT8 undef;							//
	UINT8 asduNum;							//ASDU数
	UINT8 chNum;							//通道总数
	UINT8 chType[MAX_CH_NUM];				//通道类型定义
	UINT16 sampCount;						//采样率
	UINT16 frameLen;						//帧长度
	UINT16 cntIndex[MAX_ASDU_NUM];			//序号位置
	UINT16 firstValueIndex[MAX_ASDU_NUM];	//首个通道位置
	UINT8 frame[MAX_SMV_PACK_LEN];			//帧
}SMV_PROTOCOL_PARA,*PTR_SMV_PROTOCOL_PARA;

typedef struct
{
	UINT8 index;	//帧内偏移
	UINT8 chType;	//类型
}FT3_CH_Map_TYPE,*pFT3_CH_Map_TYPE;

typedef struct
{
	UINT8 mapCount;
	UINT8 frameLen;
	UINT16 sampCount;
	UINT8 frame[MAX_FT3_PACK_LEN];
	FT3_CH_Map_TYPE chMap[MAX_FT3_CH_NUM];		//数组映射,?这里会不会结构体被4字节对齐
}FT3_PROTOCOL_PARA,*pFT3_PROTOCOL_PARA;

typedef struct
{
	UINT8 undef1;
	UINT8 type;
	UINT16 paraSize;
	UINT8 buf[VIRTUAL_DATA_SIZE];
}VIRTUAL_DATA_TYPE,*pVIRTUAL_DATA_TYPE;

typedef struct
{//开关量输出控制
	UINT16 DataItem;	//低位0~7表示表示8路输出，置1表示操作。
	UINT16 Value;	//低位 0x01： 分   0x02: 合  其它：不变
	UINT32 Sec;	//时间秒值
	UINT32 nSec;	//时间纳秒值
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
{//完整性事件控制
	UINT8 loseType;		//0无（正常），1随机丢帧，2定点丢帧，其它保留
	UINT8 loseMode;		//0重复丢帧，1一次丢帧
	UINT8 ranType;			//随机丢帧方式，0分散，1连续
	UINT8 nofDef1;			//0，未定义
	UINT16 startCnt;		//起始序号
	UINT16 stopCnt;		//停止序号
	FLOAT32 loseRadio;		//丢帧率，0.1表示10%
}LOSE_TYPE,*pLOSE_TYPE;

typedef struct
{
	UINT8 StNumType;			//默认0x05即UINT32，可先不区分保留
	UINT8 SqNumType;			//默认0x05即UINT32，可先不区分保留
	UINT16 timeLiveIndex;		//有效时间在帧中的偏移位置
	UINT16 actiomTimeIndex;		//变位时间在帧中的偏移位置
	UINT16 StNumIndex;			//StNum在帧中的偏移位置
	UINT16 SqNumIndex;			//SqNum在帧中的偏移位置
	UINT16 allDataIndex;		//数据域在帧中的偏移位置
	UINT16 frameLen;			//网络帧长度
	UINT16 undef;				//未定义
	UINT32 T0;					//重传时间,单位uS
	UINT32 T1;					//重传时间,单位uS
	UINT32 T2;					//重传时间,单位uS
	UINT32 T3;					//重传时间,单位uS
	UINT32 undef1;
	UINT32 undef2;
	UINT32 undef3;
	UINT8 frame[MAX_GOOSE_PACK_LEN];
}GOOSE_PARA_TYPE,*PTR_GOOSE_PARA_TYPE;

typedef struct
{
	UINT8 asduNum;		//采样单元数
	UINT8 undef1;		//未定义
	UINT16 undef2;		//未定义
}STAND_SAMP_HEAD_TYPE,*pSTAND_SAMP_HEAD_TYPE;

typedef struct
{
	UINT32 value;
	UINT32 pinzi;
}SMV_92_CH_TYPE,*pSMV_92_CH_TYPE;


typedef struct
{
	UINT16 stateLabel;		//状态标识
	UINT16 sampCnt;			//采样序号
	UINT32 sampTMark;		//采样延时
	UINT32 netSendTMark;	//点对点发送延时
	UINT32 FT3SendTMark;	//FT3发送延时
	INT32 UaSampData;		//UA采样值
	INT32 UbSampData;		//UB采样值
	INT32 UcSampData;		//UC采样值
	INT32 IaSampData;		//IA采样值
	INT32 IbSampData;		//IB采样值
	INT32 IcSampData;		//IC采样值
}STAND_SAMP_TYPE,*PTR_STAND_SAMP_TYPE;
#define STAND_SAMP_COUNT_EACH_PKT 10

#pragma pack()

/*end 609通信协议数据类型定义*/


UINT16 netHostChangeS(UINT16 netshort);
UINT32 netHostChangeL(UINT32 netlong);

// 消息类型定义，设置为DestMAC的最后一个字节，便于上层抓包过滤
#define MSG_FORWARD_FRM_TYPE 0X80
#define MSG_CONTROL_FRM_TYPE 0X81




//网络报文类型定义
#define NET_IEC61850  0x88ba
#define NET_GOOSE  0x88B8
#define NET_VLAN  0x8100
//#define NET_CONCROL  0x01FF
#define NET_609_TRANSMIT  0x0633
#define NET_609_CONCROL  0x0634

/*********************modify by wjm@2015-6-4**************************/
//新609转发板通道定义
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
#define	NEW609_CHTYPE_CRC_START	 0x80			//校验码起始标识
#define	NEW609_CHTYPE_CRC_STOP	 0xA0			//校验码结束标识


//新609转发板帧类型定义，命令域的定义
#define TYPE609_CONT_RIGHT_REPLY			 0x0000		//正确应答
#define TYPE609_CONT_ERROR_REPLY			 0x0001		//错误应答
#define TYPE609_CONT_SOFT_DOWN				 0x0002		//程序下载
#define TYPE609_CONT_SOFT_VER				 0x0003		//读取程序版本信息
#define TYPE609_CONT_SMV_FORMAT_WRITE		 0x0004		//设置点对点发送帧格式
#define TYPE609_CONT_SMV_FORMAT_READ		 0x0005		//读取点对点发送帧格式
#define TYPE609_CONT_FT3_FORMAT_WRITE		 0x0006		//设置FT3数据发送格式
#define TYPE609_CONT_FT3_FORMAT_READ		 0x0007		//读取FT3数据发送格式
#define TYPE609_CONT_VIRTUAL_WIRTE			 0x0008		//设置虚拟数据发生器配置
#define TYPE609_CONT_VIRTUAL_READ			 0x0009		//读取虚拟数据发生器配置
#define TYPE609_CONT_UINT32_PAR_WRITE		 0x000A		//设置U32参数配置信息
#define TYPE609_CONT_UINT32_PAR_READ		 0x000B		//读取U32参数配置信息
#define TYPE609_CONT_UINT8_PAR_WRITE		 0x000C		//设置U8参数配置信息
#define TYPE609_CONT_UINT8_PAR_READ			 0x000D		//读取U8参数配置信息
#define TYPE609_CONT_YC_CODE				 0x000E		//开关量输出控制
#define TYPE609_CONT_INTEGRITY_WRITE		 0x0010		//设置完整性事件
#define TYPE609_CONT_INTEGRITY_READ			 0x0011		//读取完整性事件
#define TYPE609_CONT_GOOSE_FORMAT_WRITE		 0x0012		//设置GOOSE帧发送格式
#define TYPE609_CONT_GOOSE_FORMAT_READ		 0x0013		//读取GOOSE帧发送格式
#define TYPE609_CONT_GOOSE_DATA_WRITE		 0x0014

#define TYPE609_CONT_NET_RECV1_DATA			 0x0301		//转发点对点1以太网接口接收数据
#define TYPE609_CONT_NET_RECV2_DATA			 0x0302		//转发点对点2以太网接口接收数据

#define TYPE609_CONT_FT3_RECV1_DATA			 0x0303		//转发FT3输入1接收数据
#define TYPE609_CONT_FT3_RECV2_DATA			 0x0304		//转发FT3输入2接收数据

#define TYPE609_CONT_GOOSE_SEND1_DATA		 0x0305		//抄送点对点1GOOSE发送数据
#define TYPE609_CONT_GOOSE_SEND2_DATA		 0x0306		//抄送点对点2GOOSE发送数据

#define TYPE609_CONT_STANDARD_DATA			 0x0307		//发送标准采样数据
#define TYPE609_CONT_YX_DATA				 0x0308		//开关量输入事件上报
#define TYPE609_CONT_TIME_ERROR_DATA		 0x0309		//时钟误差信息上报
#define TYPE609_CONT_YC_DATA				 0x030A		//开关量输出上报

#define TYPE609_CONT_NO_REPLY				 0xFFFF		//无应答


//start 609通信错误编码定义
#define COMM_ACK_ERROR					0x0000
#define COMM_ACK_RIGHT					0x0001		//正确
//end 609通信错误编码定义

//start 609的U32参数下标定义
#define U32PARA_UNDEF						0			//无效
#define U32PARA_METER_VOL_RADIO				1			//计量电压比例系数
#define U32PARA_METER_CUR_RADIO				2			//计量电流比例系数
#define U32PARA_PRO_VOL_RADIO				3			//保护电压比例系数
#define U32PARA_PRO_CUR_RADIO				4			//保护电流比例系数
#define U32PARA_VIR_UA_AMP					5			//虚拟UA幅值
#define U32PARA_VIR_UB_AMP					6			//虚拟UB幅值
#define U32PARA_VIR_UC_AMP					7			//虚拟UC幅值
#define U32PARA_VIR_IA_AMP					8			//虚拟IA幅值
#define U32PARA_VIR_IB_AMP					9			//虚拟IB幅值
#define U32PARA_VIR_IC_AMP					10			//虚拟IC幅值
#define U32PARA_VIR_UA_PHA					11			//虚拟UA相位
#define U32PARA_VIR_UB_PHA					12
#define U32PARA_VIR_UC_PHA					13
#define U32PARA_VIR_IA_PHA					14
#define U32PARA_VIR_IB_PHA					15
#define U32PARA_VIR_IC_PHA					16
#define U32PARA_VIR_UA_FRE					17			//虚拟UA频率
#define U32PARA_VIR_UB_FRE					18
#define U32PARA_VIR_UC_FRE					19
#define U32PARA_VIR_IA_FRE					20
#define U32PARA_VIR_IB_FRE					21
#define U32PARA_VIR_IC_FRE					22
#define U32PARA_VIR_RATE_DELAY1				23			//虚拟采样延时1
#define U32PARA_VIR_RATE_DELAY2				24			//虚拟采样延时2
#define U32PARA_CURRENT_SEC					25			//时钟-秒数，即609启动内部计时
#define U32PARA_CURRENT_NSEC				26			//时钟-纳秒，即609启动内部计时
#define U32PARA_LASER_POWER1				27			//光口1光功率
#define U32PARA_LASER_POWER2				28			//光品光功率

//end 609的U32参数下标定义

//start 609的U8参数下标定义
#define U8PARA_UNDEF						0				//无效
#define U8PARA_SAMP_TYPE					1				//采样模式，0：AD采样，1：虚拟生成
#define U8PARA_VALUE_TYPE					2				//模拟量输入类型，0：模拟量输入，1：小信号量输入
#define U8PARA_UA_GEAR						3				//UA档位
#define U8PARA_UB_GEAR						4				//
#define U8PARA_UC_GEAR						5				//
#define U8PARA_IA_GEAR						6				//
#define U8PARA_IB_GEAR						7				//
#define U8PARA_IC_GEAR						8				//
#define U8PARA_NET_RECV1					9				//点对点1接收
#define U8PARA_NET_RECV2					10				//
#define U8PARA_FT3_RECV1					11				//FT3输入1接收
#define U8PARA_FT3_RECV2					12				//
#define U8PARA_NET_SEND1					13				//点对点1发送
#define U8PARA_NET_SEND2					14				//
#define U8PARA_FT3_SEND1					15				//FT3输出1发送
#define U8PARA_FT3_SEND2					16				//
#define U8PARA_FT3_SEND3					17				//
#define U8PARA_FT3_SEND4					18				//
#define U8PARA_FT3_SEND5					19				//
#define U8PARA_FT3_SEND6					20				//
#define U8PARA_YX_RECV						21				//开关量输入
#define U8PARA_YC_SEND						22				//开关量输出
#define U8PARA_STAND_TRANSTOPC				23				//标准值发送到网络
#define U8PARA_NETIN1_TRANSTOPC				24				//点对点1输入发送到网络
#define U8PARA_NETIN2_TRANSTOPC				25				//
#define U8PARA_FT3IN1_TRANSTOPC				26				//FT3输入1发生到网络
#define U8PARA_FT3IN2_TRANSTOPC				27				//
#define U8PARA_YC_COUNT						28				//开关量输入个数
#define U8PARA_YX_COUNT						29				//开关量输出个数
#define U8PARA_TIME_Y						30				//对时-年
#define U8PARA_TIME_M						31				//对时-月
#define U8PARA_TIME_d						32				//对时-日
#define U8PARA_TIME_HH						33				//对时-时
#define U8PARA_TIME_MM						34				//对时-分
#define U8PARA_TIME_SS						35				//对时-秒
#define U8PARA_SYN_STATE	         			36				//同步输入状态
#define U8PARA_TIME_ERR						37				//测试时钟误差
#define U8PARA_SYN_TYPE						38				//同步方式
#define U8PARA_SYN_IN_PORT					39				//同步时钟输入端
#define U8PARA_MU_TIME_IN_PORT				40				//Mu时钟输入端
#define U8PARA_SYN_IN_TYPE					41				//同步信号输入类型
#define U8PARA_SYN_OUT_TYPE					42				//同步信号输出类型
#define U8PARA_SYN_OUT_STOP					43				//同步信号输出停止
#define U8PARA_1588_IN_PORT					44				//1588连接端口
#define U8PARA_SYN_IN1_LIGHT_BACK			45				//光时钟输入1取反
#define U8PARA_SYN_IN1_ELE_BACK				46				//电时钟输入1取反
#define U8PARA_SYN_IN2_LIGHT_BACK			47				//光时钟输入2取反
#define U8PARA_SYN_IN2_ELE_BACK				48				//电时钟输入2取反
#define U8PARA_SYN_OUT_BACK					49				//同步输出取反
#define U8PARA_SYN_OUT_IRIG_B_CHECK			50				//输出B码校验位
#define U8PARA_FT3_OUT_LOGIC				51				//FT3发送逻辑
#define U8PARA_FT3_IN_LOGIC					52				//FT3接收逻辑
#define U8PARA_FT3_OUT_MODE					53				//FT3发送模式
#define U8PARA_FT3_IN_MODE					54				//FT3接收模式
#define U8PARA_FT3_OUT_SPEED				55				//FT3发送速率
#define U8PARA_FT3_IN_SPEED					56				//FT3接收速率


//end 609的U8参数下标定义

#endif /* MUTESTER_COMM_PROTOCOL_H_ */
