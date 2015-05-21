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

//////////PD06 for ad 7608 busy int
ADI_GPIO_RESULT Set_GPIO_PD06_IODirection( ADI_GPIO_DIRECTION Direction );
ADI_GPIO_RESULT Init_GPIO_PD06_INT( void );
void Register_Callback_GPIO_PD06_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam );
void Enable_GPIO_PD06_INT(bool enable);

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

/**************************************GPIO PG08***********************************/
ADI_GPIO_RESULT Set_GPIO_PG08_IODirection( ADI_GPIO_DIRECTION Direction );
ADI_GPIO_RESULT Init_GPIO_PG08_INT( void );
void Register_Callback_GPIO_PG08_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam );
void Enable_GPIO_PG08_INT(bool enable);
#endif /* DEV_GPIO_H_ */
