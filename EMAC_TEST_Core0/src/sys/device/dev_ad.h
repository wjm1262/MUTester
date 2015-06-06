/*
 * dev_ad.h
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#ifndef DEV_AD_H_
#define DEV_AD_H_


#include "dri_ad7608.h"
#include "comm_pc_protocol.h"
#include "queue_ad_modual.h"

void RegisterADModual(void);

extern AD_STAND_SMPDATA_Q g_StandardSmpDataQueue;
extern Ft3FrmQueue g_Ft3FrmQueue;

#endif /* DEV_AD_H_ */
