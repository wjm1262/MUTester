/*
 * FT3_Test.h
 *
 *  Created on: 2015-6-1
 *      Author: Administrator
 *   Note:
 *        FT3 test
 */
#ifndef FT3_TEST_H_
#define FT3_TEST_H_

#include "BF609_FPGA_Comm.h"

/* 资源分配情况：
 *  PIN:
* 		PD10				：被检PPS
* 		PD11				：参考输入的B码解码数据
* 		PD12/PII0_D20		：SOE遥信捕获，既8个遥信输入有跳变
* 		PD15/PII0_D21		：FT3接收时FPGA给DSP的触发信号，告诉DSP有FT3数据了；
*  		PE02/PII0_D22		：FT3加量时使能FPGA，让FPGA接收DSP发送的数据；
 * Peripheral:
 *  	SPI0
 *  	SPI1
 *  	Linkport2
 */

/*
 * Init FT3 test
 * user only need invoke this function in INIT code, then the FT3 send process is running in Timer6.
 */
void Init_FT3_Test(unsigned char FT3_heap_id);
unsigned short Cal_CRC16_ByByte(void *p, unsigned int cnt);

extern FT3_TEST_DATA    *g_pBF609_FPGA_FT3;
extern EX_FT3_TEST_DATA *g_pBF609_FPGA_FT3_ex;

#endif /* FT3_TEST_H_ */
