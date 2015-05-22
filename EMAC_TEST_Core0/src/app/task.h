/*
 * app.h
 *
 *  Created on: 2015-2-10
 *      Author: Administrator
 */

#ifndef APP_H_
#define APP_H_

#include "queue.h"

#include <services/gpio/adi_gpio.h> //should before adi_initialize.h

/// task eth
void Task_Eth0_Rx( void *p_arg );

void Task_Eth0_Tx( void *p_arg );

void Task_Eth1_Rx( void *p_arg );

void Task_Eth1_Tx( void *p_arg );

void Ethernet0_Callback ( void *arg1, unsigned int event, void *FrameBuffers );

void Ethernet1_Callback ( void *arg1, unsigned int event, void *FrameBuffers );


/// task sys time
void Task_SystemTime0( void* p_arg );

void Task_SystemTime1( void* p_arg );



extern AUXI_SNAPSHOT_TM_QUEUE  g_Eth0AuxiTMQueue;
extern AUXI_SNAPSHOT_TM_QUEUE  g_Eth1AuxiTMQueue;


// TASK_AD
typedef struct sTASK_AD_PARA
{
	ADI_ETHER_BUFFER *pStandardADFrmSendBuf ;
	unsigned short usSmpCnt;
}TASK_AD_PARA;

extern TASK_AD_PARA g_TaskADPara;

void Task_AD7608( void* p_arg );


void Task_exEth_Tx_Rx( void *p_arg );




/*! Enters critical region */
#define ENTER_CRITICAL_REGION()  (adi_osal_EnterCriticalRegion())
/*! Exit critical region */
#define EXIT_CRITICAL_REGION()   (adi_osal_ExitCriticalRegion())

#endif /* APP_H_ */
