/*
 * FT3_Cfg.h
 *
 *  Created on: 2015-4-23
 *      Author: Administrator
 */

#ifndef FT3_CFG_H_
#define FT3_CFG_H_
#include <drivers/linkport/adi_linkport.h>
#include <drivers/spi/adi_spi.h>

/* ռ��������Դ���������
 * 	PD10				������PPS
 * 	PD11				���ο������B���������
 * 	PD12/PII0_D20		��SOEң�Ų��񣬼�8��ң������������
 * 	PD15/PII0_D21		��FT3����ʱFPGA��DSP�Ĵ����źţ�����DSP��FT3�����ˣ�
 *  PE02/PII0_D22		��FT3����ʱʹ��FPGA����FPGA����DSP���͵����ݣ�
 */
#define CLKIN_For_FT3  20  //MHz
#define SPI0_CLK       20 //MHz
#define SPI1_CLK       20 //MHz
/*
 *  Init_BF609_FPGA_Comm
 *  Input  Value��
 *
 *  Output Value��
 *  Return Value:
 */
int Init_BF609_FPGA_Comm(void);
int Init_BF609_FPGA_Frame(void *pBF609_FPGA_data);
/*
 * SPI call-back
 */
void SPI0_Callback(void *pCBParam, uint32_t nEvent, void *pArg);
void SPI1_Callback(void *pCBParam, uint32_t nEvent, void *pArg);
/*
 * LinkPort call-back
 */
void LinkPort_CallbackTx(void *pAppHandle, uint32_t nEvent, void *pArg);
void LinkPort_CallbackRx(void *pAppHandle,uint32_t  nEvent,void  *pArg);
/*
 *  Send FPGA CMD
 *  Input  Value��
 *
 *  Output Value��
 *  Return Value:
 */
int Send_FPGA_Cfg(void *pBF609_FPGA_data);
/*
 *  Recv FPGA CMD
 *  Input  Value��
 *
 *  Output Value��
 *  Return Value:
 */
int32_t Recv_FPGA_Info(void *pFPGA_BF609_data);
/*
 *  Send FT3 data
 *  Input  Value��
 *
 *  Output Value��
 *  Return Value:
 */
int32_t Send_FT3_Data(void *pBF609_FPGA_data, uint16_t size);
/*
 *  Recv FT3 data
 *  Input  Value��
 *
 *  Output Value��
 *  Return Value:
 */
int32_t Recv_FT3_Data(void *pFPGA_BF609_data);
ADI_LINKPORT_RESULT Init_LinkPort_FPGA(ADI_LINKPORT_DIRECTION dir);

#endif /* FT3_CFG_H_ */
