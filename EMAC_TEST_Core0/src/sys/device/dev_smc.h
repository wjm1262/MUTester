/*
 * SMC_IO.h
 *
 *  Created on: 2015-4-15
 *      Author: Administrator
 */

#ifndef SMC_IO_H_
#define SMC_IO_H_


#define pBANK1_ADDR_BASE        ((volatile unsigned short *)0XB4000000)
#define pBANK1_EMAC_CTR_BASE    ((volatile unsigned short *)0XB4080000)
#define pBANK1_ADHD_CTR_BASE    ((volatile unsigned short *)0XB4100000)
#define pBANK1_ADS1278_CTR_BASE ((volatile unsigned short *)0XB4200000)
#define pBANK1_AD7608_CTR_BASE  ((volatile unsigned short *)0XB4280000)

int Setup_SMC_Bank1(void);

int Setup_AD7608_SMC(void);

#endif /* SMC_IO_H_ */
