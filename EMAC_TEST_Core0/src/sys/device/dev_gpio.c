/*
 * dev_gpio.c
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */

#include "dev_gpio.h"
#include "post_debug.h"

#define GPIO_INT_CNT (10)
#define GPIO_CALLBACK_MEM_SIZE (GPIO_INT_CNT* ADI_GPIO_CALLBACK_MEM_SIZE)

void Init_GPIO(void)
{
	ADI_GPIO_RESULT result;
	static uint8_t gpioMemory[GPIO_CALLBACK_MEM_SIZE];
	uint32_t gpioMaxCallbacks;

	//注意：这个adi_gpio_Init只能被调用一次，否则之前注册的 callbackHandler会被后面的覆盖
	result = adi_gpio_Init( (void*)gpioMemory,	GPIO_CALLBACK_MEM_SIZE,	&gpioMaxCallbacks);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_Init:%d failed\n\n", result);
	}

	/* set GPIO output LED 1 */
	result = adi_gpio_SetDirection(	ADI_GPIO_PORT_G, ADI_GPIO_PIN_13, ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_SetDirection：%d failed\n\n", result);
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

//	/* set GPIO output DM9000A PIIN A03 */
//	result = adi_gpio_SetDirection(
//		ADI_GPIO_PORT_A,
//		ADI_GPIO_PIN_0,
//		ADI_GPIO_DIRECTION_OUTPUT);
//	if (result != ADI_GPIO_SUCCESS) {
//		printf("%s failed\n", result);
//	}
	// AD7608




}



/**************************************GPIO PD00***********************************/

ADI_GPIO_RESULT Set_GPIO_PD00_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;
	result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_D,
				ADI_GPIO_PIN_0,
				Direction);

	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PD00_IODirection failed : %d\n\n", result);
	}

	return result;
}

ADI_GPIO_RESULT Init_GPIO_PD00_INT( void )
{
	ADI_GPIO_RESULT result;
	do
	{
		 //分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3,
				ADI_GPIO_PIN_ASSIGN_BYTE_0,
				ADI_GPIO_PIN_ASSIGN_PDL_PINT3);
	    if(result != ADI_GPIO_SUCCESS)
	    {
	    	DEBUG_PRINT("Failed in function Init_GPIO_PD00_INT/adi_gpio_PinInterruptAssignment : %d\n\n",result);
	  		  break;
	    }

	    //分配具体引脚和中断方式
	    result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3,
	    		ADI_GPIO_PIN_0,
	    		ADI_GPIO_SENSE_FALLING_EDGE);//
	    if(result != ADI_GPIO_SUCCESS)
	    {
	    	DEBUG_PRINT("Failed in function Init_GPIO_PD06_INT/adi_gpio_SetPinIntEdgeSense : %d\n\n",result);
	  		  break;
	    }


	}while(0);

	return result;

}


void Register_Callback_GPIO_PD00_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam )
{
	ADI_GPIO_RESULT result;
	 //登记IRQ回调函数
	result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3,
										ADI_GPIO_PIN_0,
										handler,
										pCBParam);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Register_Callback_GPIO_PD00_INT : %d\n\n",result);
	}
}

void Enable_GPIO_PD00_INT(bool enable)
{
	ADI_GPIO_RESULT result;

	//使能中断
	result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3,
											ADI_GPIO_PIN_0,
											enable);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Enable_GPIO_PD00_INT : %d\n\n",result);
	}

}


/**************************************GPIO PE03***********************************/
ADI_GPIO_RESULT Set_GPIO_PE03_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;

	result = adi_gpio_SetDirection(
			ADI_GPIO_PORT_E,
			ADI_GPIO_PIN_3,
			Direction);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PE03_IODirection failed : %d\n\n", result);
	}

	return result;
}

/**************************************GPIO PE04***********************************/
ADI_GPIO_RESULT Set_GPIO_PE04_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;


	result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_E,
				ADI_GPIO_PIN_4,
				Direction);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PE04_IODirection failed : %d\n\n", result);
	}

	return result;
}

/**************************************GPIO PD09***********************************/

ADI_GPIO_RESULT Set_GPIO_PD09_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;
	result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_D,
				ADI_GPIO_PIN_9,
				Direction);

	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PD09_IODirection failed : %d\n\n", result);
	}

	return result;
}
ADI_GPIO_RESULT Init_GPIO_PD09_INT( void )
{
	ADI_GPIO_RESULT result;
	do
	{
		 //分配IRQ和字节
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3,
				ADI_GPIO_PIN_ASSIGN_BYTE_1,
				ADI_GPIO_PIN_ASSIGN_PDH_PINT3);
		if(result != ADI_GPIO_SUCCESS)
		{
			  DEBUG_PRINT("Failed in function Init_GPIO_PD09_INT\adi_gpio_PinInterruptAssignment : %d\n\n",result);
			  break;
		}

		//分配具体引脚和中断方式
		result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3,
				ADI_GPIO_PIN_9,
				ADI_GPIO_SENSE_RISING_EDGE);
		if(result != ADI_GPIO_SUCCESS)
		{
			  DEBUG_PRINT("Failed in function Init_GPIO_PD09_INT\adi_gpio_SetPinIntEdgeSense : %d\n\n",result);
			  break;
		}

	}while(0);

	return result;
}

void Register_Callback_GPIO_PD09_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam )
{
	ADI_GPIO_RESULT result;

	//登记IRQ回调函数
	result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3,
			ADI_GPIO_PIN_9,
			handler,
			pCBParam);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Register_Callback_GPIO_PD09_INT : %d\n\n",result);
	}

}

void Enable_GPIO_PD09_INT(bool enable)
{
	ADI_GPIO_RESULT result;

	//使能中断
	result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3,
											ADI_GPIO_PIN_9,
											enable);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Enable_GPIO_PD06_INT : %d\n\n",result);
	}

}

/**************************************GPIO PB11***********************************/

ADI_GPIO_RESULT Set_GPIO_PB11_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;

	/* set GPIO output DM9000A PIIN A25 */
	result = adi_gpio_SetDirection(
		ADI_GPIO_PORT_B,
		ADI_GPIO_PIN_11,
		ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PB11_IODirection failed: %d \n\n", result);
	}

	return result;
}


