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
 * SPORT ������ѡ�� 1: Enable , 0: Disable
 */
#define SPORT_DUAL_DATA_LINE  1
extern ADI_SPORT_HANDLE          g_hSportRx;

/*
 * AD7608�����źŶ�ȡ��ɵĻص��������˺����еı���void *pArg����SPORT���յ�������bufferָ��
 * �û���Ҫ�ڴ˺����н������ݲ�������ɺ󴥷���һ�ε�AD�ɼ�
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
