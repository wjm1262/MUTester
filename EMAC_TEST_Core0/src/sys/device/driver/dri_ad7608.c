/*
 * dev_ad7608.c
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#include "stdio.h"
#include "dri_ad7608.h"
#include "dev_gpio.h"

#define _TEST_AD 0

#define CONVERT_HIGH() 	adi_gpio_Set(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);
#define CONVERT_LOW() 	adi_gpio_Clear(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);

#define ETH0_PHYINT_PORTD_FER  ((uint16_t) ((uint16_t) 1<<6))
#define TIMER0_TMR3_PORTG_FER  ((uint32_t) ((uint32_t) 1<<8))

/*
 * ad7608 pin init, including ConvertA-B, Reset, Busy etc.
 */

uint32_t Init_AD7608_IO(void)
{
	ADI_GPIO_RESULT IO_result;

	/*
	 * AD7608 I/O init
	 */

#if _TEST_AD

	/* set ad7606 busy to GPIO input */
	*pREG_PORTD_FER &= ~ETH0_PHYINT_PORTD_FER;
	IO_result = Set_GPIO_PD06_IODirection( ADI_GPIO_DIRECTION_INPUT );

#else

	*pREG_PORTG_FER &= ~TIMER0_TMR3_PORTG_FER;
	IO_result = Set_GPIO_PG08_IODirection( ADI_GPIO_DIRECTION_INPUT );

#endif

	return IO_result;
}



/*
 * ad7608 busy irq init
 */
ADI_GPIO_RESULT Init_Busy_IO_IRQ( void )
{
#if _TEST_AD
	return Init_GPIO_PD06_INT();
#else
	return Init_GPIO_PG08_INT();
#endif
}

void Register_Buzy_IO_Callback( AD7608_Busy_ISR_Handler handler )
{
#if _TEST_AD
	Register_Callback_GPIO_PD06_INT( handler, NULL );
#else
	Register_Callback_GPIO_PG08_INT( handler, NULL );
#endif
}

void Enable_Buzy_IO_Interrupt(bool enable)
{
#if _TEST_AD
	Enable_GPIO_PD06_INT(enable);
#else
	Enable_GPIO_PG08_INT(enable);
#endif
}

/*
 * ad7606 start, rising edge start AD
 */
void Start_AD7608(void)
{
	CONVERT_LOW();
	CONVERT_HIGH();//start AD convert
}





