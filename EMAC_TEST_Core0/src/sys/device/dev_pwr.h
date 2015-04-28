/*
 * dev_pwr.h
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */

#ifndef DEV_PWR_H_
#define DEV_PWR_H_

#include <sysreg.h>
#include <ccblkfn.h>
#include <cdefBF609_rom.h>
#include <defBF609_rom.h>
#include <adi_osal.h>
#include <services/pwr/adi_pwr.h>


/* macros for CLKOUT options */

#define	CLKIN 0
#define	CCLK_BY_4  1
#define	SYSCLK_BY_2 2
#define	SCLK0 3
#define	SCLK1 4
#define	DCLK_BY_2 5
#define	USB_PLL_CLK 6
#define	OUT_CLK 7
#define	USB_CLKIN 8

//#define SYS_CLKIN 25000000
//#define SYS_CLKIN 16384000
#define SYS_CLKIN 20000000
/* Default values for the parameters */
//#define  MULTIPLIER_SEL		14
#define  MULTIPLIER_SEL		20
//#define  MULTIPLIER_SEL		16
#define	 DF_SEL				false
#define  CCLK_SEL			1
#define  DDRCLK_SEL			2
#define	 SCLK0_SEL			1
#define	 SCLK1_SEL			1
#define	 SYSCLK_SEL			2
#define	 OUTCLK_SEL			4
#define  CLKOUT_SEL			CCLK_BY_4


/* PLL Multiplier and Divisor Selections (Required Value, Bit Position) */
#define MSEL(X)   ((X << BITP_CGU_CTL_MSEL) 	& BITM_CGU_CTL_MSEL) 	/* PLL Multiplier Select [1-127]: PLLCLK = ((CLKIN x MSEL/DF+1)) = 1000MHz(max) */
#define DF(X)	((X << BITP_CGU_CTL_DF) & BITM_CGU_CTL_DF )				/* Divide frequency[true or false] */
#define CSEL(X)   ((X  << BITP_CGU_DIV_CSEL)	& BITM_CGU_DIV_CSEL)	/* Core Clock Divisor Select [1-31]: (CLKIN x MSEL/DF+1)/CSEL = 500MHz(max) */
#define SYSSEL(X) ((X  << BITP_CGU_DIV_SYSSEL) & BITM_CGU_DIV_SYSSEL)	/* System Clock Divisor Select [1-31]: (CLKIN x MSEL/DF+1)/SYSSEL = 250MHz(max) */
#define S0SEL(X)  ((X  << BITP_CGU_DIV_S0SEL)	& BITM_CGU_DIV_S0SEL)	/* SCLK0 Divisor Select [1-7]: SYSCLK/S0SEL = 125MHz(max) */
#define S1SEL(X)  ((X  << BITP_CGU_DIV_S1SEL)	& BITM_CGU_DIV_S1SEL)	/* SCLK1 Divisor Select [1-7]: SYSLCK/S1SEL = 125MHz(max) */
#define DSEL(X)   ((X  << BITP_CGU_DIV_DSEL)	& BITM_CGU_DIV_DSEL) 	/* DDR Clock Divisor Select [1-31]: (CLKIN x MSEL/DF+1)/DSEL = 250MHz(max) */
#define OSEL(X)   ((X  << BITP_CGU_DIV_OSEL)	& BITM_CGU_DIV_OSEL) 	/* OUTCLK Divisor Select [1-127]: (CLKIN x MSEL/DF+1)/OSEL = 125MHz(max). */

#define CLKOUTSEL(X)   ((X  << BITP_CGU_CLKOUTSEL_CLKOUTSEL)	& BITM_CGU_CLKOUTSEL_CLKOUTSEL) 	/* CLKOUT Select [0-8] */


ADI_PWR_RESULT Init_PowerService(void);

int GetSCLK0(float *freq, float *cycle);

#endif /* DEV_PWR_H_ */
