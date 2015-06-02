/*
 **
 ** pinmux_config.c source file generated on ÁùÔÂ 1, 2015 at 17:14:06.	
 **
 ** Copyright (C) 2000-2015 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the Pin Multiplexing configuration editor. Changes to the Pin Multiplexing
 ** configuration should be made by changing the appropriate options rather
 ** than editing this file.
 **
 ** Selected Peripherals
 ** --------------------
 ** ETH0 (MDC, MDIO, PHYINT, TXD0, TXD1, TXEN, REFCLK, RXD0, RXD1, CRS, RXERR, PTPPPS)
 ** ETH (PTPAUXIN)
 ** ETH1 (MDC, MDIO, PHYINT, TXD0, TXD1, TXEN, REFCLK, RXD0, RXD1, CRS, RXERR, PTPPPS)
 ** LP2 (CLK, ACK, D0, D1, D2, D3, D4, D5, D6, D7)
 ** TIMER0 (TMR1)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** PB13, PB14, PB15, PC00, PC01, PC02, PC03, PC04, PC05, PC06, PC07, PC09, PC11,
 ** PD06, PE08, PE09, PE10, PE11, PE12, PE13, PE14, PE15, PF00, PF01, PF02, PF03,
 ** PF04, PF05, PF06, PF07, PG00, PG02, PG03, PG04, PG05, PG06
 */

#include <sys/platform.h>
#include <stdint.h>

#define ETH0_MDC_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define ETH0_MDIO_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<14))
#define ETH0_PHYINT_PORTD_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define ETH0_TXD0_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define ETH0_TXD1_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define ETH0_TXEN_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<26))
#define ETH0_REFCLK_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<28))
#define ETH0_RXD0_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<0))
#define ETH0_RXD1_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<2))
#define ETH0_CRS_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define ETH0_RXERR_PORTC_MUX  ((uint16_t) ((uint16_t) 0<<8))
#define ETH0_PTPPPS_PORTB_MUX  ((uint32_t) ((uint32_t) 0<<30))
#define ETH_PTPAUXIN_PORTC_MUX  ((uint32_t) ((uint32_t) 2<<22))
#define ETH1_MDC_PORTE_MUX  ((uint32_t) ((uint32_t) 0<<20))
#define ETH1_MDIO_PORTE_MUX  ((uint32_t) ((uint32_t) 0<<22))
#define ETH1_PHYINT_PORTE_MUX  ((uint32_t) ((uint32_t) 0<<24))
#define ETH1_TXD0_PORTG_MUX  ((uint16_t) ((uint16_t) 0<<6))
#define ETH1_TXD1_PORTG_MUX  ((uint16_t) ((uint16_t) 0<<4))
#define ETH1_TXEN_PORTG_MUX  ((uint16_t) ((uint16_t) 0<<10))
#define ETH1_REFCLK_PORTG_MUX  ((uint16_t) ((uint16_t) 0<<12))
#define ETH1_RXD0_PORTG_MUX  ((uint16_t) ((uint16_t) 0<<0))
#define ETH1_RXD1_PORTE_MUX  ((uint32_t) ((uint32_t) 0<<30))
#define ETH1_CRS_PORTE_MUX  ((uint32_t) ((uint32_t) 0<<26))
#define ETH1_RXERR_PORTE_MUX  ((uint32_t) ((uint32_t) 0<<28))
#define ETH1_PTPPPS_PORTC_MUX  ((uint32_t) ((uint32_t) 0<<18))
#define LP2_CLK_PORTE_MUX  ((uint32_t) ((uint32_t) 2<<18))
#define LP2_ACK_PORTE_MUX  ((uint32_t) ((uint32_t) 2<<16))
#define LP2_D0_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<0))
#define LP2_D1_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<2))
#define LP2_D2_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<4))
#define LP2_D3_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<6))
#define LP2_D4_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<8))
#define LP2_D5_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<10))
#define LP2_D6_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<12))
#define LP2_D7_PORTF_MUX  ((uint16_t) ((uint16_t) 2<<14))
#define TIMER0_TMR1_PORTG_MUX  ((uint16_t) ((uint16_t) 1<<8))

#define ETH0_MDC_PORTC_FER  ((uint16_t) ((uint16_t) 1<<6))
#define ETH0_MDIO_PORTC_FER  ((uint16_t) ((uint16_t) 1<<7))
#define ETH0_PHYINT_PORTD_FER  ((uint16_t) ((uint16_t) 1<<6))
#define ETH0_TXD0_PORTC_FER  ((uint16_t) ((uint16_t) 1<<2))
#define ETH0_TXD1_PORTC_FER  ((uint16_t) ((uint16_t) 1<<3))
#define ETH0_TXEN_PORTB_FER  ((uint32_t) ((uint32_t) 1<<13))
#define ETH0_REFCLK_PORTB_FER  ((uint32_t) ((uint32_t) 1<<14))
#define ETH0_RXD0_PORTC_FER  ((uint16_t) ((uint16_t) 1<<0))
#define ETH0_RXD1_PORTC_FER  ((uint16_t) ((uint16_t) 1<<1))
#define ETH0_CRS_PORTC_FER  ((uint16_t) ((uint16_t) 1<<5))
#define ETH0_RXERR_PORTC_FER  ((uint16_t) ((uint16_t) 1<<4))
#define ETH0_PTPPPS_PORTB_FER  ((uint32_t) ((uint32_t) 1<<15))
#define ETH_PTPAUXIN_PORTC_FER  ((uint32_t) ((uint32_t) 1<<11))
#define ETH1_MDC_PORTE_FER  ((uint32_t) ((uint32_t) 1<<10))
#define ETH1_MDIO_PORTE_FER  ((uint32_t) ((uint32_t) 1<<11))
#define ETH1_PHYINT_PORTE_FER  ((uint32_t) ((uint32_t) 1<<12))
#define ETH1_TXD0_PORTG_FER  ((uint16_t) ((uint16_t) 1<<3))
#define ETH1_TXD1_PORTG_FER  ((uint16_t) ((uint16_t) 1<<2))
#define ETH1_TXEN_PORTG_FER  ((uint16_t) ((uint16_t) 1<<5))
#define ETH1_REFCLK_PORTG_FER  ((uint16_t) ((uint16_t) 1<<6))
#define ETH1_RXD0_PORTG_FER  ((uint16_t) ((uint16_t) 1<<0))
#define ETH1_RXD1_PORTE_FER  ((uint32_t) ((uint32_t) 1<<15))
#define ETH1_CRS_PORTE_FER  ((uint32_t) ((uint32_t) 1<<13))
#define ETH1_RXERR_PORTE_FER  ((uint32_t) ((uint32_t) 1<<14))
#define ETH1_PTPPPS_PORTC_FER  ((uint32_t) ((uint32_t) 1<<9))
#define LP2_CLK_PORTE_FER  ((uint32_t) ((uint32_t) 1<<9))
#define LP2_ACK_PORTE_FER  ((uint32_t) ((uint32_t) 1<<8))
#define LP2_D0_PORTF_FER  ((uint16_t) ((uint16_t) 1<<0))
#define LP2_D1_PORTF_FER  ((uint16_t) ((uint16_t) 1<<1))
#define LP2_D2_PORTF_FER  ((uint16_t) ((uint16_t) 1<<2))
#define LP2_D3_PORTF_FER  ((uint16_t) ((uint16_t) 1<<3))
#define LP2_D4_PORTF_FER  ((uint16_t) ((uint16_t) 1<<4))
#define LP2_D5_PORTF_FER  ((uint16_t) ((uint16_t) 1<<5))
#define LP2_D6_PORTF_FER  ((uint16_t) ((uint16_t) 1<<6))
#define LP2_D7_PORTF_FER  ((uint16_t) ((uint16_t) 1<<7))
#define TIMER0_TMR1_PORTG_FER  ((uint16_t) ((uint16_t) 1<<4))

int32_t adi_initpinmux(void);

/*
 * Initialize the Port Control MUX and FER Registers
 */
int32_t adi_initpinmux(void) {
    /* PORTx_MUX registers */
    *pREG_PORTB_MUX = ETH0_TXEN_PORTB_MUX | ETH0_REFCLK_PORTB_MUX
     | ETH0_PTPPPS_PORTB_MUX;
    *pREG_PORTC_MUX = ETH0_MDC_PORTC_MUX | ETH0_MDIO_PORTC_MUX
     | ETH0_TXD0_PORTC_MUX | ETH0_TXD1_PORTC_MUX | ETH0_RXD0_PORTC_MUX
     | ETH0_RXD1_PORTC_MUX | ETH0_CRS_PORTC_MUX | ETH0_RXERR_PORTC_MUX
     | ETH_PTPAUXIN_PORTC_MUX | ETH1_PTPPPS_PORTC_MUX;
    *pREG_PORTD_MUX = ETH0_PHYINT_PORTD_MUX;
    *pREG_PORTE_MUX = ETH1_MDC_PORTE_MUX | ETH1_MDIO_PORTE_MUX
     | ETH1_PHYINT_PORTE_MUX | ETH1_RXD1_PORTE_MUX | ETH1_CRS_PORTE_MUX
     | ETH1_RXERR_PORTE_MUX | LP2_CLK_PORTE_MUX | LP2_ACK_PORTE_MUX;
    *pREG_PORTF_MUX = LP2_D0_PORTF_MUX | LP2_D1_PORTF_MUX
     | LP2_D2_PORTF_MUX | LP2_D3_PORTF_MUX | LP2_D4_PORTF_MUX
     | LP2_D5_PORTF_MUX | LP2_D6_PORTF_MUX | LP2_D7_PORTF_MUX;
    *pREG_PORTG_MUX = ETH1_TXD0_PORTG_MUX | ETH1_TXD1_PORTG_MUX
     | ETH1_TXEN_PORTG_MUX | ETH1_REFCLK_PORTG_MUX | ETH1_RXD0_PORTG_MUX
     | TIMER0_TMR1_PORTG_MUX;

    /* PORTx_FER registers */
    *pREG_PORTB_FER = ETH0_TXEN_PORTB_FER | ETH0_REFCLK_PORTB_FER
     | ETH0_PTPPPS_PORTB_FER;
    *pREG_PORTC_FER = ETH0_MDC_PORTC_FER | ETH0_MDIO_PORTC_FER
     | ETH0_TXD0_PORTC_FER | ETH0_TXD1_PORTC_FER | ETH0_RXD0_PORTC_FER
     | ETH0_RXD1_PORTC_FER | ETH0_CRS_PORTC_FER | ETH0_RXERR_PORTC_FER
     | ETH_PTPAUXIN_PORTC_FER | ETH1_PTPPPS_PORTC_FER;
    *pREG_PORTD_FER = ETH0_PHYINT_PORTD_FER;
    *pREG_PORTE_FER = ETH1_MDC_PORTE_FER | ETH1_MDIO_PORTE_FER
     | ETH1_PHYINT_PORTE_FER | ETH1_RXD1_PORTE_FER | ETH1_CRS_PORTE_FER
     | ETH1_RXERR_PORTE_FER | LP2_CLK_PORTE_FER | LP2_ACK_PORTE_FER;
    *pREG_PORTF_FER = LP2_D0_PORTF_FER | LP2_D1_PORTF_FER
     | LP2_D2_PORTF_FER | LP2_D3_PORTF_FER | LP2_D4_PORTF_FER
     | LP2_D5_PORTF_FER | LP2_D6_PORTF_FER | LP2_D7_PORTF_FER;
    *pREG_PORTG_FER = ETH1_TXD0_PORTG_FER | ETH1_TXD1_PORTG_FER
     | ETH1_TXEN_PORTG_FER | ETH1_REFCLK_PORTG_FER | ETH1_RXD0_PORTG_FER
     | TIMER0_TMR1_PORTG_FER;
    return 0;
}

