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
static unsigned short CRC16_Table[256]=
{
		 0x0000, 0x3d65, 0x7aca, 0x47af, 0xf594, 0xc8f1, 0x8f5e, 0xb23b, 0xd64d, 0xeb28, 0xac87, 0x91e2, 0x23d9, 0x1ebc, 0x5913, 0x6476,
		 0x91ff, 0xac9a, 0xeb35, 0xd650, 0x646b, 0x590e, 0x1ea1, 0x23c4, 0x47b2, 0x7ad7, 0x3d78, 0x1d, 0xb226, 0x8f43, 0xc8ec, 0xf589,
		 0x1e9b, 0x23fe, 0x6451, 0x5934, 0xeb0f, 0xd66a, 0x91c5, 0xaca0, 0xc8d6, 0xf5b3, 0xb21c, 0x8f79, 0x3d42, 0x27, 0x4788, 0x7aed,
		 0x8f64, 0xb201, 0xf5ae, 0xc8cb, 0x7af0, 0x4795, 0x3a, 0x3d5f, 0x5929, 0x644c, 0x23e3, 0x1e86, 0xacbd, 0x91d8, 0xd677, 0xeb12,
		 0x3d36, 0x53, 0x47fc, 0x7a99, 0xc8a2, 0xf5c7, 0xb268, 0x8f0d, 0xeb7b, 0xd61e, 0x91b1, 0xacd4, 0x1eef, 0x238a, 0x6425, 0x5940,
		 0xacc9, 0x91ac, 0xd603, 0xeb66, 0x595d, 0x6438, 0x2397, 0x1ef2, 0x7a84, 0x47e1, 0x4e, 0x3d2b, 0x8f10, 0xb275, 0xf5da, 0xc8bf,
		 0x23ad, 0x1ec8, 0x5967, 0x6402, 0xd639, 0xeb5c, 0xacf3, 0x9196, 0xf5e0, 0xc885, 0x8f2a, 0xb24f, 0x74, 0x3d11, 0x7abe, 0x47db,
		 0xb252, 0x8f37, 0xc898, 0xf5fd, 0x47c6, 0x7aa3, 0x3d0c, 0x69, 0x641f, 0x597a, 0x1ed5, 0x23b0, 0x918b, 0xacee, 0xeb41, 0xd624,
		 0x7a6c, 0x4709, 0xa6, 0x3dc3, 0x8ff8, 0xb29d, 0xf532, 0xc857, 0xac21, 0x9144, 0xd6eb, 0xeb8e, 0x59b5, 0x64d0, 0x237f, 0x1e1a,
		 0xeb93, 0xd6f6, 0x9159, 0xac3c, 0x1e07, 0x2362, 0x64cd, 0x59a8, 0x3dde, 0xbb, 0x4714, 0x7a71, 0xc84a, 0xf52f, 0xb280, 0x8fe5,
		 0x64f7, 0x5992, 0x1e3d, 0x2358, 0x9163, 0xac06, 0xeba9, 0xd6cc, 0xb2ba, 0x8fdf, 0xc870, 0xf515, 0x472e, 0x7a4b, 0x3de4, 0x81,
		 0xf508, 0xc86d, 0x8fc2, 0xb2a7, 0x9c, 0x3df9, 0x7a56, 0x4733, 0x2345, 0x1e20, 0x598f, 0x64ea, 0xd6d1, 0xebb4, 0xac1b, 0x917e,
		 0x475a, 0x7a3f, 0x3d90, 0xf5, 0xb2ce, 0x8fab, 0xc804, 0xf561, 0x9117, 0xac72, 0xebdd, 0xd6b8, 0x6483, 0x59e6, 0x1e49, 0x232c,
		 0xd6a5, 0xebc0, 0xac6f, 0x910a, 0x2331, 0x1e54, 0x59fb, 0x649e, 0xe8, 0x3d8d, 0x7a22, 0x4747, 0xf57c, 0xc819, 0x8fb6, 0xb2d3,
		 0x59c1, 0x64a4, 0x230b, 0x1e6e, 0xac55, 0x9130, 0xd69f, 0xebfa, 0x8f8c, 0xb2e9, 0xf546, 0xc823, 0x7a18, 0x477d, 0xd2, 0x3db7,
		 0xc83e, 0xf55b, 0xb2f4, 0x8f91, 0x3daa, 0xcf, 0x4760, 0x7a05, 0x1e73, 0x2316, 0x64b9, 0x59dc, 0xebe7, 0xd682, 0x912d, 0xac48,
};
/*
 * CRC16查表法计算
 */
unsigned short Cal_CRC16_ByByte(void *p, unsigned int cnt)
{
    unsigned short crc = 0;
    unsigned char  da;
    unsigned char * ptr = (unsigned char * )p;
    while (cnt--)
    {
        da = crc >> 8;  // CRC(h)
        crc <<= 8;
        crc ^= CRC16_Table[da ^ *ptr++];
    }
    return (~crc);
}
/*
 * CRC16 calculate (int start, char *p, int n)
 */
static unsigned short Cal_CRC16_By_HalfByte(unsigned char *ptr, int len)
{
	unsigned short crc;
		unsigned char da;
		unsigned short static crc_ta[16]={
		0x0000, 0x3d65, 0x7aca, 0x47af, 0xf594, 0xc8f1, 0x8f5e, 0xb23b,
		0xd64d, 0xeb28, 0xac87, 0x91e2, 0x23d9, 0x1ebc, 0x5913, 0x6476
		};
		crc = 0;
		while(len--)
		{
			da    = crc >> 12; //crc high byte high 4 bits

			crc <<= 4;

			crc  ^= crc_ta[da^(*ptr >> 4)]; //nor data high 4 bits

			da    = crc >> 12; ////crc high byte low 4 bits

			crc <<= 4;

			crc ^= crc_ta[da^(*ptr & 0x0f)];

			ptr++;
		}

	return(crc);
}
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
/* Timer pulse width, delay and period parameters */
#if FT3_4000
#define TIMER5_PERIOD        (25000ul)//(40960 >> 1ul)//163.84M   125M : 31250
#else
#define TIMER5_PERIOD        6400u//(6400u)//163.84M   125M :
#endif

#define TIMER5_DELAY         (10)
#define TIMER5_WIDTH        (TIMER5_PERIOD/2)
/* Timer to be used in the example */
#define GP_TIMER5_NUM        5
/* Timer handle */
ADI_TMR_HANDLE   ghTimer5;
/* Memory required for opening the timer */
uint8_t TimerMemory5[ADI_TMR_MEMORY];

/* Timer event handler */
static void Timer5_ISR(void *pCBParam, uint32_t Event, void *pArg);

void GP_Timer5_Init(void)
{
	ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;
    /* Open the timer */
    if( (eTmrResult = adi_tmr_Open (
            GP_TIMER5_NUM,
            TimerMemory5,
            ADI_TMR_MEMORY,
            Timer5_ISR,
            NULL,
            &ghTimer5)) != ADI_TMR_SUCCESS)
    {
        printf("Timer open failed 0x%08X \n", eTmrResult);
    }
    // Set the mode to PWM OUT
     if((eTmrResult = adi_tmr_SetMode(
             ghTimer5,
             ADI_TMR_MODE_CONTINUOUS_PWMOUT)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to open timer in PWM out mode 0x%08X \n", eTmrResult);
     }
      //Set the IRQ mode to get interrupt after timer counts to Delay + Width
     if((eTmrResult = adi_tmr_SetIRQMode(
             ghTimer5,
             ADI_TMR_IRQMODE_PERIOD)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer IRQ mode 0x%08X \n", eTmrResult);
     }

      //Set the Period
     if((eTmrResult = adi_tmr_SetPeriod(
             ghTimer5,
             TIMER5_PERIOD)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer Period 0x%08X \n", eTmrResult);
     }

      //Set the timer width
     if((eTmrResult = adi_tmr_SetWidth(
             ghTimer5,
             TIMER5_WIDTH)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer Width 0x%08X \n", eTmrResult);
     }

     // Set the timer Delay
     if((eTmrResult = adi_tmr_SetDelay(
             ghTimer5,
             TIMER5_DELAY)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to set the timer Delay 0x%08X \n", eTmrResult);
     }

      //Enable the timer to stop gracefully,used in PWM mode,set the PWM wave more graceful.
     if((eTmrResult = adi_tmr_EnableGracefulStop(
             ghTimer5,
             true)) != ADI_TMR_SUCCESS)
     {
         printf("Failed to enable the timer to stop gracefully 0x%08X \n", eTmrResult);
     }
     adi_tmr_Enable(ghTimer5, true);

}

FT3_TEST_DATA    *g_pBF609_FPGA_FT3;
EX_FT3_TEST_DATA *g_pBF609_FPGA_FT3_ex;
static void Timer5_ISR(void *pCBParam, uint32_t Event, void *pArg)
{
	static uint16_t SmpCnt = 0;
	uint16_t crc;

	/* set the FPGA trigger pin in low */
	adi_gpio_Clear(ADI_GPIO_PORT_E,ADI_GPIO_PIN_2);

#if FT3_4000
	SmpCnt = (SmpCnt + 1) % 4000;
	uint16_t tmp_cnt = myHtons(SmpCnt);
	/*
	 *
	 *  single phase FT3 data
	 *
	 */
	g_pBF609_FPGA_FT3->SinglePhaseData[0].SmpCnt    = tmp_cnt;
	g_pBF609_FPGA_FT3->SinglePhaseData[0].data1[0] = 1;

	/* cal 16 bytes crc*/
	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->SinglePhaseData[0].data1), 16);
	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = myHtons(crc);
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC1      = 1111;

	/* cal 4 bytes crc */
	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->SinglePhaseData[0].Status), 4);
	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = myHtons(crc);
//	g_pBF609_FPGA_FT3->SinglePhaseData[0].CRC2      = 1111;


	g_pBF609_FPGA_FT3->SinglePhaseData[1].SmpCnt    = tmp_cnt;
	g_pBF609_FPGA_FT3->SinglePhaseData[1].data1[0] = 2;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->SinglePhaseData[1].data1), 16);
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = myHtons(crc);
	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC1      = 1111;

	/* cal 4 bytes crc */
//	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->SinglePhaseData[1].Status), 4);
//	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = myHtons(crc);
	g_pBF609_FPGA_FT3->SinglePhaseData[1].CRC2      = 1111;

	g_pBF609_FPGA_FT3->SinglePhaseData[2].SmpCnt    = tmp_cnt;
	g_pBF609_FPGA_FT3->SinglePhaseData[2].data1[0] = 3;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->SinglePhaseData[2].data1), 16);
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = myHtons(crc);
	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC1      = 1111;

	/* cal 4 bytes crc */
//	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->SinglePhaseData[2].Status), 4);
//	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = myHtons(crc);
	g_pBF609_FPGA_FT3->SinglePhaseData[2].CRC2      = 1111;

	/*
	 *
	 *  Three phase FT3 data
	 *
	 */
	g_pBF609_FPGA_FT3->ThreePhaseData[0].SmpCnt     = tmp_cnt;
	g_pBF609_FPGA_FT3->ThreePhaseData[0].data1[0]  = 4;

	/* cal 16 bytes crc*/
	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[0].data1), 16);
	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC1      = 1111;

	/* cal 16 bytes crc */
	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[0].data2), 16);
	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC2      = 1111;

	/* cal 8 bytes crc */
	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = myHtons(crc);
//	g_pBF609_FPGA_FT3->ThreePhaseData[0].CRC3      = 1111;


	g_pBF609_FPGA_FT3->ThreePhaseData[1].SmpCnt     = tmp_cnt;
	g_pBF609_FPGA_FT3->ThreePhaseData[1].data1[0]  = 5;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[1].data1), 16);
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = myHtons(crc);
	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC1      = 1111;

	/* cal 16 bytes crc */
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[1].data2), 16);
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = myHtons(crc);
	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC2      = 1111;

	/* cal 8 bytes crc */
//	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->ThreePhaseData[1].CphaseData), 8);
//	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = myHtons(crc);
	g_pBF609_FPGA_FT3->ThreePhaseData[1].CRC3      = 1111;


	g_pBF609_FPGA_FT3->ThreePhaseData[2].SmpCnt     = tmp_cnt;
	g_pBF609_FPGA_FT3->ThreePhaseData[2].data1[0]  = 6;

	/* cal 16 bytes crc*/
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[2].data1), 16);
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = myHtons(crc);
	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC1      = 1111;

	/* cal 16 bytes crc */
//	crc = Cal_CRC16_ByByte((g_pBF609_FPGA_FT3->ThreePhaseData[2].data2), 16);
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = myHtons(crc);
	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC2      = 1111;

	/* cal 8 bytes crc */
//	crc = Cal_CRC16_ByByte(&(g_pBF609_FPGA_FT3->ThreePhaseData[2].CphaseData), 8);
//	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = myHtons(crc);
	g_pBF609_FPGA_FT3->ThreePhaseData[2].CRC3      = 1111;

	Send_FT3_Data(g_pBF609_FPGA_FT3, sizeof(FT3_TEST_DATA));
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);

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
	/* Init FPGA communication */
	Init_BF609_FPGA_Comm();

	/* linkport init */
	Init_LinkPort_FPGA(ADI_LINKPORT_DIR_TX);



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

	/* set the default data to 0xa5 */
	for(int i = 0; i < sizeof(FT3_TEST_DATA); i++)
		((uint8_t *)g_pBF609_FPGA_FT3)[i] = 0xA5;

	/* set the frame start code */
	g_pBF609_FPGA_FT3->SinglePhaseData[0].StartCode = 0x6405;
	g_pBF609_FPGA_FT3->SinglePhaseData[1].StartCode = 0x6405;
	g_pBF609_FPGA_FT3->SinglePhaseData[2].StartCode = 0x6405;

	g_pBF609_FPGA_FT3->ThreePhaseData[0].StartCode  = 0x6405;
	g_pBF609_FPGA_FT3->ThreePhaseData[1].StartCode  = 0x6405;
	g_pBF609_FPGA_FT3->ThreePhaseData[2].StartCode  = 0x6405;

	/* used for FT3 send , 4k */
//	GP_Timer5_Init();
}
