/*
 * sys.h
 *
 *  Created on: 2015-2-6
 *      Author: wu jm
 */

#ifndef SYS_H_
#define SYS_H_

#include "device_def.h"

//#include "os.h"

/*******************************************************************************
* 结构体名	: SystemStruct
* 描述	    : 最重要的一个结构体，封装了System层的所有接口
********************************************************************************
*版本     作者            日期            说明
*V0.1    Wujm        2015/02/6       初始版本
*******************************************************************************/
typedef struct Dev_Ethnet
{
	// data section
	void* hDev;

	// operation
	void* (*Open)(void);
//	void* (*Close)(void* hDev);
	ADI_ETHER_RESULT (*Close)( ADI_ETHER_HANDLE hEtherDevice );

	int  (*Configure)(void* hDev);
	 void (*SetCallbakFn)(void* hDev, void* CallbakFn );

	void (*EnableRxTimeStamped)(void* hDev);
	void (*EnableTxTimeStamped)(void* hDev);
	void (*EnableAuxiTimeStamped)(void* hDev);
	void (*EnableTimingTx)(void* hDev);

	void (*EnableGemacDMA)(void* hDev, int nMode);
	void (*EnableGemacInt)(void* hDev);
	void (*EnableGemacTx)(void* hDev);
	void (*EnableGemacRx)(void* hDev);
	void (*EnableGemacTxRx)(void* hDev);

	ADI_ETHER_RESULT (*Read) ( ADI_ETHER_HANDLE const phDevice, ADI_ETHER_BUFFER *pBuffer );
	ADI_ETHER_RESULT (*Write)( ADI_ETHER_HANDLE const phDevice, ADI_ETHER_BUFFER *pBuffer );
	ADI_ETHER_BUFFER* (*Recv)( ADI_ETHER_HANDLE const hDevice );

}DEV_ETHNET;

typedef struct Dev_ExEthnet
{
	// data section
//	void* hDev;

	// operation

//	int  (*Configure)(void* hDev);


	void (*InitExEthnet)(uint8_t* srcMac );

	void (*EnableMACIntInterrupt) (bool enable);

	int (*EthSend)(void*buffer, int len);
	void* (*EthRecv)(void);

	int (*InitExEthQueue ) ( EXEMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pBuffer );
	int (*PushUnprocessElem) ( EXEMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pBuffer );
	ADI_ETHER_BUFFER* (*PopProcessedElem)( EXEMAC_FRAME_Q *pQueue, int n);
	ADI_ETHER_BUFFER* (*PopUnprocessElem) ( EXEMAC_FRAME_Q *pQueue );

}DEV_EXETHNET;


typedef struct Dev_SysTime
{
	// data section

	// operation
	int  (*InitSystemTime)(void* hEthDev);
	void (*EnableTimeStampAuxinInterrupt) (void* hEthDev);
	void (*SetAuxiTMTriggerHandler) (void* hEthDev, void* handler );
	void (*SetTargetTMTriggerHandler) (void* hEthDev, void* handler );

}DEV_SYSTIME;
typedef struct Dev_AD
{
	// data section

	// operation

	uint32_t (*InitADDevice)(void);

	void (*RegisterBuzyIOCallback)( AD7608_Busy_ISR_Handler handler );

	void (*EnableBuzyIOInterrupt) (bool enable);

//	void (*RegisterSPI0Callback)( SPI_CallbackFn pfCallback);
	void (*RegisterSportCallback) ( SPORT_CallbackFn pfCallback);

	//

}DEV_AD;

#define GOOSE_TIME_NUM (5)
typedef struct Goose_Object
{
	long m_nTmInitVal[GOOSE_TIME_NUM]; // T0,T1,T2,T3,T4, 的初始值, 顺序不能更改
	long m_nTime[GOOSE_TIME_NUM];         // T0,T1,T2,T3,T4, , 顺序不能更改
	uint32_t m_stNum;
	uint32_t m_sqNum;
	uint8_t  m_pFrame[MAX_GOOSE_PACK_LEN];

	uint8_t StNumType;			//默认0x05即UINT32，可先不区分保留
	uint8_t SqNumType;			//默认0x05即UINT32，可先不区分保留
	uint16_t timeLiveIndex;		//有效时间在帧中的偏移位置
	uint16_t actiomTimeIndex;		//变位时间在帧中的偏移位置
	uint16_t StNumIndex;			//StNum在帧中的偏移位置
	uint16_t SqNumIndex;			//SqNum在帧中的偏移位置
	uint16_t allDataIndex;		//数据域在帧中的偏移位置
	uint16_t frameLen;			//网络帧长度
	uint32_t bSend;

	//初始化T0,T1,T2,T3,T4, 的初始值, 顺序不能更改，
	void (*GooseObject_initTime)(struct Goose_Object*pObj, long t0, long t1 , long t2,  long t3 , long t4 );

	//发生状态变位事件，设置T0,T1,T2,T3,T4,Treverse 为 初始值
	void (*GooseObject_resetTime)(struct Goose_Object*pObj);


	void (*OnMyTimer)( struct Goose_Object*pObj );
	void (*OnStValChangedEvent)( struct Goose_Object*pObj );
}GooseObject;
typedef struct System_Struct
{
    void (*Initialize)(void);

    struct Device
    {
//        RtcStruct Rtc;
//
//        struct IO
//        {
//            void (*SetBeep)(bool status);
//        }IO;
//
//        struct Adc
//        {
//            void (*Register)(AdcChannelEnum adcChannel, ushort * dataPointer);
//        }Adc;
//
//        struct Lcd
//        {
//            void (* DisplayString)(byte y, string string);
//        }Lcd;

//        struct Usart1
//        {
//            void (*Open)(void);
//            void (*Close)(void);
//            void (*RxdRegister)(void * registerFunction);
//            bool (*WriteByte)(byte data);
//            void (*Write)(byte * dataPointer, int sum);
//        }Usart1;

    	DEV_ETHNET Eth0;
    	DEV_ETHNET Eth1;
    	DEV_EXETHNET exEth;

    	DEV_SYSTIME SysTime0;
    	DEV_SYSTIME SysTime1;

    	DEV_AD AD7608;

//        struct Timer
//        {
//            byte (*Start)(TimerhandleModeEnum mode, uint delay, function registerFunction);
//            void (*Stop)(byte id);
//        }Timer;
//
//        struct Storage
//        {
//            struct Parameter
//            {
//                ParameterStruct (*Profile)(void);
//                void (*Erase)(void);
//                bool (*Read)(uint * dataPointer);
//                bool (*Write)(uint * dataPointer);
//                void (*Clean)(ParameterStruct parameter);
//            }Parameter;
//            struct Log
//            {
//                LogStruct (*Profile)(void);
//                void (*Erase)(void);
//                char * (*Read)(int sequence);
//                bool (*Write)(char *fmt, ...);
//            }Log;
//        }Storage;
//
//        struct Systick
//        {
//            bool (*Register)(SystickEnum systemTick, function registerFunction);
//        }Systick;

    }Device;

}SYSTEM_STRUCT;

extern SYSTEM_STRUCT MuTesterSystem;


#endif /* SYS_H_ */
