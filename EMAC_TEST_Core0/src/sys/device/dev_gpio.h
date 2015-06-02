/*
 * dev_gpio.h
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */

#ifndef DEV_GPIO_H_
#define DEV_GPIO_H_

#include <ccblkfn.h>
#include <adi_osal.h>
#include <services/gpio/adi_gpio.h> //should before adi_initialize.h

void Init_GPIO(void);

//IO口的分配参考 “合并单元计量保护一体化测试仪FPGA通信协议 Ver1.3”


//////////PD09 for dm9000a int
ADI_GPIO_RESULT Set_GPIO_PD09_IODirection( ADI_GPIO_DIRECTION Direction );
ADI_GPIO_RESULT Init_GPIO_PD09_INT( void );
void Register_Callback_GPIO_PD09_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam );
void Enable_GPIO_PD09_INT(bool enable);

///////////PB11 for dm9000a
ADI_GPIO_RESULT Set_GPIO_PB11_IODirection( ADI_GPIO_DIRECTION Direction );

//////////// PE03 for  SPORT1B_FS
ADI_GPIO_RESULT Set_GPIO_PE03_IODirection( ADI_GPIO_DIRECTION Direction );

//////////// PE04 for SPORT1B_CLK
ADI_GPIO_RESULT Set_GPIO_PE04_IODirection( ADI_GPIO_DIRECTION Direction );


/**************************************GPIO PD00 for ad 7608 busy int***************/
ADI_GPIO_RESULT Set_GPIO_PD00_IODirection( ADI_GPIO_DIRECTION Direction );
ADI_GPIO_RESULT Init_GPIO_PD00_INT( void );
void Register_Callback_GPIO_PD00_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam );
void Enable_GPIO_PD00_INT(bool enable);

#endif /* DEV_GPIO_H_ */
