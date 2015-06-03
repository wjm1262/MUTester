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

/* 占用引脚资源分配情况：
 * 	PD10				：被检PPS
 * 	PD11				：参考输入的B码解码数据
 * 	PD12/PII0_D20		：SOE遥信捕获，既8个遥信输入有跳变
 * 	PD15/PII0_D21		：FT3接收时FPGA给DSP的触发信号，告诉DSP有FT3数据了；
 *  PE02/PII0_D22		：FT3加量时使能FPGA，让FPGA接收DSP发送的数据；
 */
#define CLKIN_For_FT3  20  //MHz
#define SPI0_CLK       20 //MHz
#define SPI1_CLK       20 //MHz
/*
 *  Init_BF609_FPGA_Comm
 *  Input  Value：
 *
 *  Output Value：
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
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int Send_FPGA_Cfg(void *pBF609_FPGA_data);
/*
 *  Recv FPGA CMD
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int32_t Recv_FPGA_Info(void *pFPGA_BF609_data);
/*
 *  Send FT3 data
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int32_t Send_FT3_Data(void *pBF609_FPGA_data, uint16_t size);
/*
 *  Recv FT3 data
 *  Input  Value：
 *
 *  Output Value：
 *  Return Value:
 */
int32_t Recv_FT3_Data(void *pFPGA_BF609_data);
ADI_LINKPORT_RESULT Init_LinkPort_FPGA(ADI_LINKPORT_DIRECTION dir);

#endif /* FT3_CFG_H_ */
