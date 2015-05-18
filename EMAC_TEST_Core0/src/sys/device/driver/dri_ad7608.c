/*
 * dev_ad7608.c
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#include "stdio.h"
#include "dri_ad7608.h"
#include "dev_gpio.h"


#define CONVERT_HIGH() 	adi_gpio_Set(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);
#define CONVERT_LOW() 	adi_gpio_Clear(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);



/*
 * ad7608 pin init, including ConvertA-B, Reset, Busy etc.
 */

uint32_t Init_AD7608_IO(void)
{
	ADI_GPIO_RESULT IO_result;

	/*
	 * AD7608 I/O init
	 */

	/* set ad7606 busy to GPIO input */
	IO_result = Set_GPIO_PD06_IODirection( ADI_GPIO_DIRECTION_INPUT );


	return IO_result;
}



/*
 * ad7608 busy irq init
 */
ADI_GPIO_RESULT Init_Busy_IO_IRQ( void )
{
	return Init_GPIO_PD06_INT();
}

void Register_Buzy_IO_Callback( AD7608_Busy_ISR_Handler handler )
{
	Register_Callback_GPIO_PD06_INT( handler, NULL );
}

void Enable_Buzy_IO_Interrupt(bool enable)
{
	Enable_GPIO_PD06_INT(enable);
}

/*
 * ad7606 start, rising edge start AD
 */
void Start_AD7608(void)
{
	CONVERT_LOW();
	CONVERT_HIGH();//start AD convert
}





