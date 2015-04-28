/*
 * uart_test.h
 *
 *  Created on: 2014-12-6
 *      Author: Administrator
 */

#ifndef UART_TEST_H_
#define UART_TEST_H_

#include <adi_osal.h>
#include <cdefbf609.h>
#include <ccblkfn.h>

#include <drivers/uart/adi_uart.h>

#include "post_debug.h"
//#include "post_common.h"


/*******************************************************************
*  global var
*******************************************************************/



/*******************************************************************
*  function prototypes
*******************************************************************/
void Init_UART0(void);
int PutChar(const char c);
int GetChar(char *const c);
int TEST_UART(void);
void ClearReceiver(void);

void Init_UART1(void);
int PutChar1(const char c);
int GetChar1(char *const c);
void ClearReceiver1(void);


/* if we are using the UART to print debug info, define the following */
#ifdef __DEBUG_UART__
int UART_DEBUG_PRINT(void);
int UART_DEBUG_PRINT1(void);
#endif



#endif /* UART_TEST_H_ */
