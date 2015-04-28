/*
 * dev_sport.h
 *
 *  Created on: 2015-4-22
 *      Author: Administrator
 */

#ifndef DEV_SPORT_H_
#define DEV_SPORT_H_

#include <ccblkfn.h>
#include <stdio.h>

#include <services/int/adi_int.h>
#include <services/gpio/adi_gpio.h>
#include <drivers/sport/adi_sport.h>
#include "post_debug.h"


/*
 * SPORT 数据线选择 1: Enable , 0: Disable
 */
#define SPORT_DUAL_DATA_LINE  1
extern ADI_SPORT_HANDLE          g_hSportRx;

/*
 * AD7608数字信号读取完成的回调函数，此函数中的变量void *pArg就是SPORT接收到的数据buffer指针
 * 用户需要在此函数中进行数据操作，完成后触发新一次的AD采集
 */

typedef void (*SPORT_CallbackFn) (
    void        *pAppHandle,
    uint32_t     nEvent,
    void        *pArg);

void Register_SPORT1B_Callback(SPORT_CallbackFn pfCallback);

void CancelClk_SPORT1B(void);

void Enable_SPORT1B(void);

void Disable_SPORT1B(void);

void SubmitBuffer_SPORT1B(void);

ADI_SPORT_RESULT Init_SPORT1B(void);

#endif /* DEV_SPORT_H_ */
