/*
 * dev_spi.h
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#ifndef DEV_SPI_H_
#define DEV_SPI_H_


#include <cdefbf609.h>
#include <ccblkfn.h>
#include <drivers/spi/adi_spi.h>

extern ADI_SPI_HANDLE g_hSPI0;

/*
 * AD7608数字信号读取完成的回调函数，此函数中的变量void *pArg就是SPI接收到的数据buffer指针
 * 用户需要在此函数中进行数据操作，完成后触发新一次的AD采集
 */
typedef void (*SPI_CallbackFn)(void *pCBParam, uint32_t nEvent, void *pArg);

uint32_t Init_SPI0(void);

void EnableSPI0(void);

void DisableSPI0(void);

void Register_SPI0_Callback(SPI_CallbackFn pfCallback);

#endif /* DEV_SPI_H_ */
