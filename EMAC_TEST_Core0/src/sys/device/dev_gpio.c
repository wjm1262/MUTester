/*
 * dev_gpio.c
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */

#include "dev_gpio.h"
#include "post_debug.h"
void Init_GPIO(void)
{
	ADI_GPIO_RESULT result;
	static uint8_t gpioMemory[32];
	uint32_t gpioMaxCallbacks;

	result = adi_gpio_Init( (void*)gpioMemory,	32,	&gpioMaxCallbacks);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_Init:%d failed\n\n", result);
	}

	/* set GPIO output LED 1 */
	result = adi_gpio_SetDirection(	ADI_GPIO_PORT_G, ADI_GPIO_PIN_13, ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_SetDirection£º%d failed\n\n", result);
	}

	/* LED1 */
	result = adi_gpio_Clear(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_Set: %d failed\n\n", result);
	}

	/* set GPIO output WDI */
	result = adi_gpio_SetDirection(	ADI_GPIO_PORT_C, ADI_GPIO_PIN_15, ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_SetDirection:%d failed\n\n", result);
	}

	result = adi_gpio_Set(ADI_GPIO_PORT_C, ADI_GPIO_PIN_15);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_Set:%d failed\n\n", result);

	}

	result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_G,
			ADI_GPIO_PIN_1,
			ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_SetDirection G1:%d failed \n\n", result);
	}

	result = adi_gpio_Clear(ADI_GPIO_PORT_G, ADI_GPIO_PIN_1);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_Set G1:%d failed \n\n", result);

	}

	result = adi_gpio_SetDirection(
		ADI_GPIO_PORT_D,
		ADI_GPIO_PIN_0,
		ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		printf("adi_gpio_SetDirection failed : %d\n", result);

	}

	result = adi_gpio_Clear(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);
		if (result != ADI_GPIO_SUCCESS)
		{
			DEBUG_PRINT("adi_gpio_Set G1:%d failed \n\n", result);

		}


//	/* set GPIO output DM9000A PIIN A03 */
//	result = adi_gpio_SetDirection(
//		ADI_GPIO_PORT_A,
//		ADI_GPIO_PIN_0,
//		ADI_GPIO_DIRECTION_OUTPUT);
//	if (result != ADI_GPIO_SUCCESS) {
//		printf("%s failed\n", result);
//	}

}
