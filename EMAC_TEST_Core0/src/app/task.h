/*
 * app.h
 *
 *  Created on: 2015-2-10
 *      Author: wu jm
 */

#ifndef APP_H_
#define APP_H_


#include <services/gpio/adi_gpio.h> //should before adi_initialize.h
#include "dri_ether_header.h"

/// task eth
void Task_Eth0_Rx( void *p_arg );

void Task_Eth0_Tx( void *p_arg );

void Task_Eth1_Rx( void *p_arg );

void Task_Eth1_Tx( void *p_arg );

void Ethernet0_Callback ( void *arg1, unsigned int event, void *FrameBuffers );

void Ethernet1_Callback ( void *arg1, unsigned int event, void *FrameBuffers );


void Task_exEth_Tx_Rx( void *p_arg );

/// task sys time
void Task_SystemTime0( void* p_arg );

void Task_SystemTime1( void* p_arg );



// TASK_AD
typedef struct sTASK_AD_PARA
{
	ADI_ETHER_BUFFER *pStandardADFrmSendBuf ;
	unsigned short usSmpCnt;
}TASK_AD_PARA;

extern TASK_AD_PARA g_TaskADPara;

void Task_AD7608( void* p_arg );

//StandardSmpData convert to SV and FT3
int StandardSmpDataFormatConverter();
int OutputFT3Frm(void);






/*! Enters critical region */
#define ENTER_CRITICAL_REGION()  (adi_osal_EnterCriticalRegion())
/*! Exit critical region */
#define EXIT_CRITICAL_REGION()   (adi_osal_ExitCriticalRegion())

#endif /* APP_H_ */
