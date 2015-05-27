/*
 * SMC_IO.c
 *
 *  Created on: 2015-4-15
 *      Author: ChiliWang
 */
#include "dev_smc.h"

#include <ccblkfn.h>
#include <services/int/adi_int.h>
#include <services/int/adi_sec.h>

#include <math.h>

#include "dev_pwr.h"
/*
 * SMC & SPORT2-B pin MUX
 */
#define SMC0_A03_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<0))
#define SMC0_A04_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<2))
#define SMC0_A05_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define SMC0_A06_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define SMC0_A07_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define SMC0_A08_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define SMC0_A09_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define SMC0_A10_PORTA_MUX  ((uint16_t) ((uint16_t) 0<<14))
#define SMC0_A11_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<16))
#define SMC0_A12_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<18))
#define SMC0_A13_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define SMC0_A14_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<20))
#define SMC0_A15_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<22))
#define SMC0_A16_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define SMC0_A17_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<24))
#define SMC0_A18_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<26))
#define SMC0_A19_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<28))
#define SMC0_A20_PORTA_MUX  ((uint32_t) ((uint32_t) 0<<30))
#define SMC0_A21_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define SMC0_A22_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<14))
#define SMC0_A23_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<16))
#define SMC0_A24_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<20))
#define SMC0_A25_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<22))
#define SMC0_AMS1_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<2))
#define SMC0_AMS2_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define SMC0_AMS3_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define SMC0_NORCLK_PORTB_MUX  ((uint16_t) ((uint16_t) 0<<0))
#define SMC0_BG_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<24))
#define SMC0_BGH_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<18))

#define SPORT2_BCLK_PORTG_MUX  ((uint32_t) ((uint32_t) 1<<20))
#define SPORT2_BFS_PORTG_MUX  ((uint16_t) ((uint16_t) 0<<14))
#define SPORT2_BD0_PORTG_MUX  ((uint32_t) ((uint32_t) 0<<24))
#define SPORT2_BD1_PORTG_MUX  ((uint32_t) ((uint32_t) 0<<22))

#define SMC0_A03_PORTA_FER  ((uint16_t) ((uint16_t) 1<<0))
#define SMC0_A04_PORTA_FER  ((uint16_t) ((uint16_t) 1<<1))
#define SMC0_A05_PORTA_FER  ((uint16_t) ((uint16_t) 1<<2))
#define SMC0_A06_PORTA_FER  ((uint16_t) ((uint16_t) 1<<3))
#define SMC0_A07_PORTA_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SMC0_A08_PORTA_FER  ((uint16_t) ((uint16_t) 1<<5))
#define SMC0_A09_PORTA_FER  ((uint16_t) ((uint16_t) 1<<6))
#define SMC0_A10_PORTA_FER  ((uint16_t) ((uint16_t) 1<<7))
#define SMC0_A11_PORTA_FER  ((uint32_t) ((uint32_t) 1<<8))
#define SMC0_A12_PORTA_FER  ((uint32_t) ((uint32_t) 1<<9))
#define SMC0_A13_PORTB_FER  ((uint16_t) ((uint16_t) 1<<2))
#define SMC0_A14_PORTA_FER  ((uint32_t) ((uint32_t) 1<<10))
#define SMC0_A15_PORTA_FER  ((uint32_t) ((uint32_t) 1<<11))
#define SMC0_A16_PORTB_FER  ((uint16_t) ((uint16_t) 1<<3))
#define SMC0_A17_PORTA_FER  ((uint32_t) ((uint32_t) 1<<12))
#define SMC0_A18_PORTA_FER  ((uint32_t) ((uint32_t) 1<<13))
#define SMC0_A19_PORTA_FER  ((uint32_t) ((uint32_t) 1<<14))
#define SMC0_A20_PORTA_FER  ((uint32_t) ((uint32_t) 1<<15))
#define SMC0_A21_PORTB_FER  ((uint16_t) ((uint16_t) 1<<6))
#define SMC0_A22_PORTB_FER  ((uint16_t) ((uint16_t) 1<<7))
#define SMC0_A23_PORTB_FER  ((uint32_t) ((uint32_t) 1<<8))
#define SMC0_A24_PORTB_FER  ((uint32_t) ((uint32_t) 1<<10))
#define SMC0_A25_PORTB_FER  ((uint32_t) ((uint32_t) 1<<11))
#define SMC0_AMS1_PORTB_FER  ((uint16_t) ((uint16_t) 1<<1))
#define SMC0_AMS2_PORTB_FER  ((uint16_t) ((uint16_t) 1<<4))
#define SMC0_AMS3_PORTB_FER  ((uint16_t) ((uint16_t) 1<<5))
#define SMC0_NORCLK_PORTB_FER  ((uint16_t) ((uint16_t) 1<<0))
#define SMC0_BG_PORTB_FER  ((uint32_t) ((uint32_t) 1<<12))
#define SMC0_BGH_PORTB_FER  ((uint32_t) ((uint32_t) 1<<9))


/*
 * control ads1278 with SMC module bank1，so we should configure the SMC
 */
int Setup_EMAC_SMC(void);

int Setup_SMC_Bank1(void)
{

	float freq, cycle;
	int bclk;
	int wst, wht, wat, rst, rht, rat;
	int prest, preat, tt, it, pgws;
	uint32_t smc_b1ctl, smc_b1tim, smc_b1etim;

    /* SMC0 PORTx_MUX registers */
	*pREG_PORTA_MUX |= SMC0_A03_PORTA_MUX | SMC0_A04_PORTA_MUX
	 | SMC0_A05_PORTA_MUX | SMC0_A06_PORTA_MUX | SMC0_A07_PORTA_MUX
	 | SMC0_A08_PORTA_MUX | SMC0_A09_PORTA_MUX | SMC0_A10_PORTA_MUX
	 | SMC0_A11_PORTA_MUX | SMC0_A12_PORTA_MUX | SMC0_A14_PORTA_MUX
	 | SMC0_A15_PORTA_MUX | SMC0_A17_PORTA_MUX | SMC0_A18_PORTA_MUX
	 | SMC0_A19_PORTA_MUX | SMC0_A20_PORTA_MUX;
	*pREG_PORTB_MUX |= SMC0_A13_PORTB_MUX | SMC0_A16_PORTB_MUX
	 | SMC0_A21_PORTB_MUX | SMC0_A22_PORTB_MUX | SMC0_A23_PORTB_MUX
	 | SMC0_A24_PORTB_MUX | SMC0_A25_PORTB_MUX | SMC0_AMS1_PORTB_MUX
	 | SMC0_AMS2_PORTB_MUX | SMC0_AMS3_PORTB_MUX | SMC0_NORCLK_PORTB_MUX
	 | SMC0_BG_PORTB_MUX | SMC0_BGH_PORTB_MUX;

    /* SMC0 PORTx_FER registers */
    *pREG_PORTA_FER |= SMC0_A03_PORTA_FER | SMC0_A04_PORTA_FER
     | SMC0_A05_PORTA_FER | SMC0_A06_PORTA_FER | SMC0_A07_PORTA_FER
     | SMC0_A08_PORTA_FER | SMC0_A09_PORTA_FER | SMC0_A10_PORTA_FER
     | SMC0_A11_PORTA_FER | SMC0_A12_PORTA_FER | SMC0_A14_PORTA_FER
     | SMC0_A15_PORTA_FER | SMC0_A17_PORTA_FER | SMC0_A18_PORTA_FER
     | SMC0_A19_PORTA_FER | SMC0_A20_PORTA_FER;
    *pREG_PORTB_FER |= SMC0_A13_PORTB_FER | SMC0_A16_PORTB_FER
     | SMC0_A21_PORTB_FER | SMC0_A22_PORTB_FER | SMC0_A23_PORTB_FER
     | SMC0_A24_PORTB_FER | SMC0_A25_PORTB_FER | SMC0_AMS1_PORTB_FER
     | SMC0_AMS2_PORTB_FER | SMC0_AMS3_PORTB_FER | SMC0_NORCLK_PORTB_FER
     | SMC0_BG_PORTB_FER | SMC0_BGH_PORTB_FER;

	GetSCLK0(&freq, &cycle);
//	printf("%f, %f\n",freq,cycle);

	/* Write setup time,25ns */
	wst = ceilf(25 / cycle);

	/* Write hold time ,25ns*/
	wht = ceilf(25 / cycle);

	/* Write access time >= 50ns */
	wat = ceilf (50 / cycle);


	/* Read access time >= 25ns */
	rat = ceilf (25 / cycle);

	/* Read setup time */
	rst = ceilf (20 / cycle) - rat;
	if (rst < 1)
		rst = 1;

	/* Read hold time >= 20ns */
	rht = ceilf (20 / cycle);


/////must set the following pre-time,cannot be zero at the same time/////
	/* Pre setup time, must */
	prest = 1;

	/* Pre access time */
	preat = 1;


	smc_b1tim = (((wst << BITP_SMC_B1TIM_WST) & BITM_SMC_B1TIM_WST)
		     | ((wht << BITP_SMC_B1TIM_WHT) & BITM_SMC_B1TIM_WHT)
		     | ((wat << BITP_SMC_B1TIM_WAT) & BITM_SMC_B1TIM_WAT)
		     | ((rst << BITP_SMC_B1TIM_RST) & BITM_SMC_B1TIM_RST)
		     | ((rht << BITP_SMC_B1TIM_RHT) & BITM_SMC_B1TIM_RHT)
		     | ((rat << BITP_SMC_B1TIM_RAT) & BITM_SMC_B1TIM_RAT));

	smc_b1etim = (((prest << BITP_SMC_B1ETIM_PREST) & BITM_SMC_B1ETIM_PREST)
		      | ((preat << BITP_SMC_B1ETIM_PREAT) & BITM_SMC_B1ETIM_PREAT));

	smc_b1ctl = ((1 << BITP_SMC_B1CTL_EN)  //Enable bank1
		     | (0 << BITP_SMC_B1CTL_MODE)); //setup async  RAM mode



	*pREG_SMC0_B1TIM = smc_b1tim;
	asm(" ssync ; ");
	*pREG_SMC0_B1ETIM = smc_b1etim;
	asm(" ssync ; ");
	*pREG_SMC0_B1CTL = smc_b1ctl;
	asm(" ssync ; ");


	Setup_EMAC_SMC();
	return 0;
}
/*
 * ADS1278 parameter setup
 * input parameter : AD_Mode : 00: high speed mode,      data rate  = 128    KSPS, CLK = 32.768Mhz
 * 							   01: High resolution mode, data rate  = 52.734 KSPS. CLK = 27Mhz
 * 							   10： Low-Power mode,       data rate  = 52.734 KSPS. CLK = 27Mhz
 * 							   11： Low-Speed mode,       data rate  = 10.547 KSPS. CLK = 27Mhz
 *   				 bSPI    : 1 ：SPI方式读取数据,
 *   				 		   0： SPORT方式读取数据
 */
int ADS1278_Setup(uint8_t AD_Mode, uint8_t bSPI)
{
	if(AD_Mode > 0x03)
		return -1;
	if(bSPI) /* Format = 001 */
		*pBANK1_ADS1278_CTR_BASE = (AD_Mode | (1u << 5) | (1u << 2)); /* mode | CLK_DIV | format */
	else     /* Format = 100 */
		*pBANK1_ADS1278_CTR_BASE = (AD_Mode | (1u << 5) | (4u << 2)); /* mode | CLK_DIV | format */
	return 0;
}

/*
 * ADS7608 parameter setup
 * input parameter : None
 * SMC-D[4]   : Reset pin
 * SMC-D[3]   : Range, 1: 10Vpp, 0: 5Vpp
 * SMC-D[2:0] : AD0S2 AD0S1 ADOS0 (over sample bits)
 */
int Setup_AD7608_SMC(void)
{
	/*
	 * 1, set reset pin low.
	 * 2, set range in 5Vpp.
	 * 3, disable the over-sample.
	 */
	*pBANK1_AD7608_CTR_BASE &= ~((1u <<4) | (1u << 3) | (7u << 0));
	*pBANK1_AD7608_CTR_BASE |=  (1u << 4) | (0u << 0); //D4 Rising edge reset the ad7608

	*pBANK1_AD7608_CTR_BASE &= ~(1u << 4); //Restore the reset pin
	return 0;
}

int Setup_EMAC_SMC(void)
{
	*pBANK1_EMAC_CTR_BASE   = (0x0f);
	return 0;
}
