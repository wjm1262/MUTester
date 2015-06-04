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

/* ��Դ���������
 *  PIN:
* 		PD10				������PPS
* 		PD11				���ο������B���������
* 		PD12/PII0_D20		��SOEң�Ų��񣬼�8��ң������������
* 		PD15/PII0_D21		��FT3����ʱFPGA��DSP�Ĵ����źţ�����DSP��FT3�����ˣ�
*  		PE02/PII0_D22		��FT3����ʱʹ��FPGA����FPGA����DSP���͵����ݣ�
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
