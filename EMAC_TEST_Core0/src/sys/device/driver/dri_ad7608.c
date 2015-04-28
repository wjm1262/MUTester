/*
 * dev_ad7608.c
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#include "stdio.h"
#include "dri_ad7608.h"



#define CONVERT_HIGH() 	adi_gpio_Set(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);
#define CONVERT_LOW() 	adi_gpio_Clear(ADI_GPIO_PORT_D, ADI_GPIO_PIN_0);



/*
 * ad7608 pin init, including ConvertA-B, Reset, Busy etc.
 */
uint32_t Init_AD7608_IO(void)
{
	ADI_GPIO_RESULT IO_result;
	static uint8_t gpioMemory[32];
	uint32_t gpioMaxCallbacks;

	/*
	 * AD7608 I/O init
	 */
	do
	{
		IO_result = adi_gpio_Init(
					(void*)gpioMemory,
					32,
					&gpioMaxCallbacks);
		if (IO_result != ADI_GPIO_SUCCESS)
		{
			printf("adi_gpio_Init failed : %d\n", IO_result);
			break;
		}

		/* set GPIO output ad7606 convertA-B */
//		IO_result = adi_gpio_SetDirection(
//			ADI_GPIO_PORT_D,
//			ADI_GPIO_PIN_0,
//			ADI_GPIO_DIRECTION_OUTPUT);
//		if (IO_result != ADI_GPIO_SUCCESS)
//		{
//			printf("adi_gpio_SetDirection failed : %d\n", IO_result);
//			break;
//		}

		/* set GPIO output ad7606 RESET */
		IO_result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_D,
			ADI_GPIO_PIN_3,
			ADI_GPIO_DIRECTION_OUTPUT);
		if (IO_result != ADI_GPIO_SUCCESS)
		{
			printf("adi_gpio_SetDirection failed : %d\n", IO_result);
			break;
		}

		/* set GPIO output ad7606 busy */
		IO_result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_D,
			ADI_GPIO_PIN_6,
			ADI_GPIO_DIRECTION_INPUT);
		if (IO_result != ADI_GPIO_SUCCESS)
		{
			printf("adi_gpio_SetDirection failed : %d\n", IO_result);
			break;
		}

		/* set GPIO output SPORT1-B-FS */
		IO_result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_E,
			ADI_GPIO_PIN_3,
			ADI_GPIO_DIRECTION_OUTPUT);
		if (IO_result != ADI_GPIO_SUCCESS)
		{
			printf("adi_gpio_SetDirection failed : %d\n", IO_result);
			break;
		}

		/* set GPIO output SPORT1-B-CLK, only used in SPORT 1 data line */
		IO_result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_E,
			ADI_GPIO_PIN_4,
			ADI_GPIO_DIRECTION_OUTPUT);
		if (IO_result != ADI_GPIO_SUCCESS)
		{
			printf("adi_gpio_SetDirection failed : %d\n", IO_result);
			break;
		}
	}while(0);

	return IO_result;
}



/*
 * ad7608 busy irq init
 */
ADI_GPIO_RESULT Init_Busy_IO_IRQ( void )
{
	ADI_GPIO_RESULT result;
	do
	{
		 //分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_ASSIGN_BYTE_0, ADI_GPIO_PIN_ASSIGN_PDL_PINT3);
	    if(result != ADI_GPIO_SUCCESS)
	    {
	  		  printf("\n\t Failed in function adi_gpio_PinInterruptAssignment : %d",result);
	  		  break;
	    }

	    //分配具体引脚和中断方式
	    result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_6, ADI_GPIO_SENSE_FALLING_EDGE);
	    if(result != ADI_GPIO_SUCCESS)
	    {
	  		  printf("\n\t Failed in function adi_gpio_SetPinIntEdgeSense : %d",result);
	  		  break;
	    }

//	    //登记IRQ回调函数
//	    result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_6, AD7608_Busy_ISR, NULL);
//	    if(result != ADI_GPIO_SUCCESS)
//	    {
//	  		  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
//	  		  break;
//	    }
//
//	    //使能中断
//	    result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3, ADI_GPIO_PIN_6, true);
//	    if(result != ADI_GPIO_SUCCESS)
//	    {
//	  		  printf("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
//	  		  break;
//	    }

	}while(0);

	return result;

}

void Register_Buzy_IO_Callback( AD7608_Busy_ISR_Handler handler )
{
	ADI_GPIO_RESULT result;
	 //登记IRQ回调函数
	result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3,
			ADI_GPIO_PIN_6,
		handler,
		NULL);
	if(result != ADI_GPIO_SUCCESS)
	{
		//DEBUG_PRINT("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
	}
}

void Enable_Buzy_IO_Interrupt(bool enable)
{
	ADI_GPIO_RESULT result;

	//使能中断
	result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3,
											ADI_GPIO_PIN_6,
											enable);
	if(result != ADI_GPIO_SUCCESS)
	{
		//DEBUG_PRINT("\n\t Failed in function adi_gpio_RegisterCallback : %d",result);
	}

}

/*
 * ad7606 start, rising edge start AD
 */
void Start_AD7608(void)
{
	CONVERT_LOW();
	CONVERT_HIGH();//start AD convert
}





