/*
 * FT3_Test.c
 *
 *  Created on: 2015-6-1
 *      Author: Administrator
 */
#include <stdint.h>
#include "../BF609_FPGA_Cfg.h"
#include "../BF609_FPGA_Comm.h"
#include "../BF609_FPGA_Comm_Protocol.h"
#include "FT3_Frame_Definition_Test.h"
#include <stdio.h>
#include "crc_calculator.h"
/*
 * net to 16
 */
static inline uint16_t myHtons(uint16_t netShort)
{
	uint16_t retValue;
	*(((uint8_t*)&retValue)+0) = (uint8_t)(netShort >> 8);
	*(((uint8_t*)&retValue)+1) = (uint8_t)(netShort);
	return retValue;
}
/*
 * General Timer initialization.
 * 1, #include <services/tmr/adi_tmr.h>
 * 2, #include <services/int/adi_int.h>  when you use adi_int_InstallHandler function.
 * 3, Define the ADI_TMR_HANDLE and the Memory for the GP timer.
 * 4, #define the value of the period, the default SCK is 125M.
 * 5, Setting the pin multiplex in system.svc if you want to output the PWM.
 * 6, Define the interrupt Handler of the GP timer if used.
 */
#include <services/tmr/adi_tmr.h>
#include <services/int/adi_int.h>
#include <services/gpio/adi_gpio.h>
#define FT3_4000 1

/* Timer event handler */
static void Timer5_ISR1(void *pCBParam, uint32_t Event, void *pArg);

FT3_TEST_DATA    *pBF609_FPGA_FT3;
EX_FT3_TEST_DATA *pBF609_FPGA_FT3_ex;

void ADData2FT3Frm( void *pAD_Value, uint8_t AD_ByteLength, uint32_t smpCnt)
{
	uint16_t crc;

	/* set the FPGA trigger pin in low */
//	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

#if FT3_4000
	uint16_t tmp_cnt = myHtons(smpCnt);
	/*
	 *
	 *  single phase FT3 data
	 *
	 */
//	pBF609_FPGA_FT3->SinglePhaseData[0].SmpCnt    = tmp_cnt;
//	pBF609_FPGA_FT3->SinglePhaseData[0].data1[0] = 1;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData[0].data1), 16);
////	pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = 1111;
//
//	/* cal 4 bytes crc */
////	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->SinglePhaseData[0].Status), 4);
////	pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = 1111;
//
//
//	pBF609_FPGA_FT3->SinglePhaseData[1].SmpCnt    = tmp_cnt;
//	pBF609_FPGA_FT3->SinglePhaseData[1].data1[0] = 2;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData[1].data1), 16);
////	pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = 1111;
//
//	/* cal 4 bytes crc */
////	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->SinglePhaseData[1].Status), 4);
////	pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = 1111;
//
//	pBF609_FPGA_FT3->SinglePhaseData[2].SmpCnt    = tmp_cnt;
//	pBF609_FPGA_FT3->SinglePhaseData[2].data1[0] = 3;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->SinglePhaseData[2].data1), 16);
////	pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = 1111;
//
//	/* cal 4 bytes crc */
////	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->SinglePhaseData[2].Status), 4);
////	pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = 1111;
//
//	/*
//	 *
//	 *  Three phase FT3 data
//	 *
//	 */
//	pBF609_FPGA_FT3->ThreePhaseData[0].SmpCnt     = tmp_cnt;
//	pBF609_FPGA_FT3->ThreePhaseData[0].data1[0]  = 4;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[0].data1), 16);
////	pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = 1111;
//
//	/* cal 16 bytes crc */
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[0].data2), 16);
////	pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = 111;
//
//	/* cal 8 bytes crc */
////	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
////	pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = 1111;
//
//
//	pBF609_FPGA_FT3->ThreePhaseData[1].SmpCnt     = tmp_cnt;
//	pBF609_FPGA_FT3->ThreePhaseData[1].data1[0]  = 5;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[1].data1), 16);
////	pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = 11111;
//
//	/* cal 16 bytes crc */
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[1].data2), 16);
////	pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = 1111;
//
//	/* cal 8 bytes crc */
////	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseData[1].CphaseData), 8);
////	pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = 1111;
//
//
//	pBF609_FPGA_FT3->ThreePhaseData[2].SmpCnt     = tmp_cnt;
//	pBF609_FPGA_FT3->ThreePhaseData[2].data1[0]  = 6;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[2].data1), 16);
////	pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = 1111;
//
//	/* cal 16 bytes crc */
////	crc = Cal_CRC16_ByByte((pBF609_FPGA_FT3->ThreePhaseData[2].data2), 16);
////	pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = 1111;
//
//	/* cal 8 bytes crc */
////	crc = Cal_CRC16_ByByte(&(pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
////	pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = myHtons(crc);
//	pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = 11111;
//
//	Send_FT3_Data(pBF609_FPGA_FT3, sizeof(FT3_TEST_DATA));
//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#else
	/* 12.8k sample rate */


	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[15] = SmpCnt;

	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[14] = SmpCnt >> 8;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[15] = SmpCnt;


	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC1      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC1      = crc;


	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data2, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC2      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC2      = crc;

	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data3, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC3      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC3      = crc;

	crc = Cal_CRC16_ByByte(pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data4, 16);
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC4      = crc;
	pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC4      = crc;
	Send_FT3_Data(pBF609_FPGA_FT3_ex, sizeof(EX_FT3_TEST_DATA));
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#endif

}


FT3_TEST_DATA    *g_pBF609_FPGA_FT3;
EX_FT3_TEST_DATA *g_pBF609_FPGA_FT3_ex;

static void Timer5_ISR1(void *pCBParam, uint32_t Event, void *pArg)
{
//	static uint16_t SmpCnt = 0;
//	uint16_t crc;
//
//	/* set the FPGA trigger pin in low */
//	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);
//
#if FT3_4000
//	SmpCnt = (SmpCnt + 1) % 4000;
//	uint16_t tmp_cnt = myHtons(SmpCnt);
//	/*
//	 *
//	 *  single phase FT3 data
//	 *
//	 */
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].SmpCnt    = tmp_cnt;
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].data1[0] = 1;
//
//	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->SinglePhaseData[0].data1), 16);
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = myHtons(crc);
////	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = 1111;
//
//	/* cal 4 bytes crc */
//	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->SinglePhaseData[0].Status), 4);
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = myHtons(crc);
////	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = 1111;
//
//
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].SmpCnt    = tmp_cnt;
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].data1[0] = 2;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->SinglePhaseData[1].data1), 16);
////	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = myHtons(crc);
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = 1111;
//
//	/* cal 4 bytes crc */
////	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->SinglePhaseData[1].Status), 4);
////	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = myHtons(crc);
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = 1111;
//
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].SmpCnt    = tmp_cnt;
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].data1[0] = 3;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->SinglePhaseData[2].data1), 16);
////	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = myHtons(crc);
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = 1111;
//
//	/* cal 4 bytes crc */
////	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->SinglePhaseData[2].Status), 4);
////	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = myHtons(crc);
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = 1111;
//
//	/*
//	 *
//	 *  Three phase FT3 data
//	 *
//	 */
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].SmpCnt     = tmp_cnt;
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].data1[0]  = 4;
//
//	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[0].data1), 16);
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = myHtons(crc);
////	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = 1111;
//
//	/* cal 16 bytes crc */
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[0].data2), 16);
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = myHtons(crc);
////	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = 1111;
//
//	/* cal 8 bytes crc */
//	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = myHtons(crc);
////	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = 1111;
//
//
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].SmpCnt     = tmp_cnt;
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].data1[0]  = 5;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[1].data1), 16);
////	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = 1111;
//
//	/* cal 16 bytes crc */
////	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[1].data2), 16);
////	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = 1111;
//
//	/* cal 8 bytes crc */
////	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->ThreePhaseData[1].CphaseData), 8);
////	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = 1111;
//
//
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].SmpCnt     = tmp_cnt;
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].data1[0]  = 6;
//
//	/* cal 16 bytes crc*/
////	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[2].data1), 16);
////	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = 1111;
//
//	/* cal 16 bytes crc */
////	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[2].data2), 16);
////	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = 1111;
//
//	/* cal 8 bytes crc */
////	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
////	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = 1111;
//
//	Send_FT3_Data(g_pBF609_FPGA_FT3, sizeof(FT3_TEST_DATA));
//	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#else   /* 12.8k sample rate */
	SmpCnt = (SmpCnt + 1) % 12800;

	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[14] = SmpCnt >> 8;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1[15] = SmpCnt;

	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[14] = SmpCnt >> 8;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].data1[15] = SmpCnt;

	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[14] = SmpCnt >> 8;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].data1[15] = SmpCnt;

	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[14] = SmpCnt >> 8;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].data1[15] = SmpCnt;

	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[14] = SmpCnt >> 8;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].data1[15] = SmpCnt;

	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[14] = SmpCnt >> 8;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].data1[15] = SmpCnt;


	crc = Cal_CRC16_ByByte(g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data1, 16);
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC1      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC1      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC1      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC1      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC1      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC1      = crc;


	crc = Cal_CRC16_ByByte(g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data2, 16);
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC2      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC2      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC2      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC2      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC2      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC2      = crc;

	crc = Cal_CRC16_ByByte(g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data3, 16);
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC3      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC3      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC3      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC3      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC3      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC3      = crc;

	crc = Cal_CRC16_ByByte(g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].data4, 16);
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[0].CRC4      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[1].CRC4      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[2].CRC4      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[3].CRC4      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[4].CRC4      = crc;
	g_pBF609_FPGA_FT3_ex->FT3_Extend_Data_sa[5].CRC4      = crc;
	Send_FT3_Data(g_pBF609_FPGA_FT3_ex, sizeof(EX_FT3_TEST_DATA));
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

#endif

}

void Init_FT3_Test(unsigned char FT3_heap_id)
{

	/*
	 *  FPGA control data buffer malloc.
	 */
	uint8_t *pBF609_FPGA_data = heap_malloc(FT3_heap_id, sizeof(BF609_FPAG_CFG));
	if(pBF609_FPGA_data == NULL)
	{
		printf("FT3 send buffer malloc failed !\n");
	}
	/*
	 * FT3 data buffer malloc.
	 */
	g_pBF609_FPGA_FT3 = heap_malloc(FT3_heap_id, sizeof(FT3_TEST_DATA)+3);
	if(g_pBF609_FPGA_FT3 == NULL)
	{
		printf("FT3 send buffer malloc failed !\n");
	}

//	/* set the default data to 0xa5 */
//	for(int i = 0; i < sizeof(FT3_TEST_DATA); i++)
//		((uint8_t *)g_pBF609_FPGA_FT3)[i] = 0xA5;
//
//	/* set the frame start code */
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].StartCode = 0x6405;
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].StartCode = 0x6405;
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].StartCode = 0x6405;
//
//
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].StartCode  = 0x6405;
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].StartCode  = 0x6405;
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].StartCode  = 0x6405;

	/* Init FPGA communication */
	Init_BF609_FPGA_Comm();

	/* linkport init */
	Init_LinkPort_FPGA(ADI_LINKPORT_DIR_TX);

	/* used for FT3 send , 4k */
//	GP_Timer5_Init();
}
