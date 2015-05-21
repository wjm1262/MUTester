/*
 * dev_gpio.c
 *
 *  Created on: 2015-3-4
 *      Author: Administrator
 */

#include "dev_gpio.h"
#include "post_debug.h"

#define GPIO_INT_CNT (5)
#define GPIO_CALLBACK_MEM_SIZE (GPIO_INT_CNT* ADI_GPIO_CALLBACK_MEM_SIZE)

void Init_GPIO(void)
{
	ADI_GPIO_RESULT result;
	static uint8_t gpioMemory[GPIO_CALLBACK_MEM_SIZE];
	uint32_t gpioMaxCallbacks;

	result = adi_gpio_Init( (void*)gpioMemory,	GPIO_CALLBACK_MEM_SIZE,	&gpioMaxCallbacks);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_Init:%d failed\n\n", result);
	}

	/* set GPIO output LED 1 */
	result = adi_gpio_SetDirection(	ADI_GPIO_PORT_G, ADI_GPIO_PIN_13, ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("adi_gpio_SetDirection��%d failed\n\n", result);
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
	// AD7608


//		/* set GPIO output ad7606 busy */
//		result = adi_gpio_SetDirection(
//			ADI_GPIO_PORT_D,
//			ADI_GPIO_PIN_6,
//			ADI_GPIO_DIRECTION_INPUT);
//		if (result != ADI_GPIO_SUCCESS)
//		{
//			printf("adi_gpio_SetDirection failed : %d\n", result);
//
//		}
//
//		/* set GPIO output SPORT1-B-FS */
//		result = adi_gpio_SetDirection(
//			ADI_GPIO_PORT_E,
//			ADI_GPIO_PIN_3,
//			ADI_GPIO_DIRECTION_OUTPUT);
//		if (result != ADI_GPIO_SUCCESS)
//		{
//			printf("adi_gpio_SetDirection failed : %d\n", result);
//
//		}
//
//		/* set GPIO output SPORT1-B-CLK, only used in SPORT 1 data line */
//		result = adi_gpio_SetDirection(
//			ADI_GPIO_PORT_E,
//			ADI_GPIO_PIN_4,
//			ADI_GPIO_DIRECTION_OUTPUT);
//		if (result != ADI_GPIO_SUCCESS)
//		{
//			printf("adi_gpio_SetDirection failed : %d\n", result);
//
//		}

}



/**************************************GPIO PD06***********************************/

ADI_GPIO_RESULT Set_GPIO_PD06_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;
	result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_D,
				ADI_GPIO_PIN_6,
				Direction);

	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PD06_IODirection failed : %d\n\n", result);
	}

	return result;
}

ADI_GPIO_RESULT Init_GPIO_PD06_INT( void )
{
	ADI_GPIO_RESULT result;
	do
	{
		 //����IRQ���ֽ�
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3,
				ADI_GPIO_PIN_ASSIGN_BYTE_0,
				ADI_GPIO_PIN_ASSIGN_PDL_PINT3);
	    if(result != ADI_GPIO_SUCCESS)
	    {
	    	DEBUG_PRINT("Failed in function Init_GPIO_PD06_INT/adi_gpio_PinInterruptAssignment : %d\n\n",result);
	  		  break;
	    }

	    //����������ź��жϷ�ʽ
	    result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3,
	    		ADI_GPIO_PIN_6,
	    		ADI_GPIO_SENSE_FALLING_EDGE);//
	    if(result != ADI_GPIO_SUCCESS)
	    {
	    	DEBUG_PRINT("Failed in function Init_GPIO_PD06_INT/adi_gpio_SetPinIntEdgeSense : %d\n\n",result);
	  		  break;
	    }


	}while(0);

	return result;

}


void Register_Callback_GPIO_PD06_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam )
{
	ADI_GPIO_RESULT result;
	 //�Ǽ�IRQ�ص�����
	result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_3,
										ADI_GPIO_PIN_6,
										handler,
										pCBParam);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Register_Callback_GPIO_PD06_INT : %d\n\n",result);
	}
}

void Enable_GPIO_PD06_INT(bool enable)
{
	ADI_GPIO_RESULT result;

	//ʹ���ж�
	result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_3,
											ADI_GPIO_PIN_6,
											enable);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Enable_GPIO_PD06_INT : %d\n\n",result);
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
		 //����IRQ���ֽ�
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_3,
				ADI_GPIO_PIN_ASSIGN_BYTE_1,
				ADI_GPIO_PIN_ASSIGN_PDH_PINT3);
		if(result != ADI_GPIO_SUCCESS)
		{
			  DEBUG_PRINT("Failed in function Init_GPIO_PD09_INT\adi_gpio_PinInterruptAssignment : %d\n\n",result);
			  break;
		}

		//����������ź��жϷ�ʽ
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

	//�Ǽ�IRQ�ص�����
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

	//ʹ���ж�
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

/**************************************GPIO PG08***********************************/

ADI_GPIO_RESULT Set_GPIO_PG08_IODirection( ADI_GPIO_DIRECTION Direction )
{
	ADI_GPIO_RESULT result;
	result = adi_gpio_SetDirection(
				ADI_GPIO_PORT_G,
				ADI_GPIO_PIN_8,
				Direction);

	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Set_GPIO_PD06_IODirection failed : %d\n\n", result);
	}

	return result;
}

ADI_GPIO_RESULT Init_GPIO_PG08_INT( void )
{
	ADI_GPIO_RESULT result;
	do
	{
		 //����IRQ���ֽ�
		result = adi_gpio_PinInterruptAssignment(ADI_GPIO_PIN_INTERRUPT_5,
				ADI_GPIO_PIN_ASSIGN_BYTE_1,
				ADI_GPIO_PIN_ASSIGN_PGH_PINT5);
	    if(result != ADI_GPIO_SUCCESS)
	    {
	    	DEBUG_PRINT("Failed in function Init_GPIO_PG08_INT/adi_gpio_PinInterruptAssignment : %d\n\n",result);
	  		  break;
	    }

	    //����������ź��жϷ�ʽ
	    result = adi_gpio_SetPinIntEdgeSense(ADI_GPIO_PIN_INTERRUPT_3,
	    		ADI_GPIO_PIN_8,
	    		ADI_GPIO_SENSE_FALLING_EDGE);
	    if(result != ADI_GPIO_SUCCESS)
	    {
	    	DEBUG_PRINT("Failed in function Init_GPIO_PG08_INT/adi_gpio_SetPinIntEdgeSense : %d\n\n",result);
	  		  break;
	    }


	}while(0);

	return result;
}

void Register_Callback_GPIO_PG08_INT( ADI_GPIO_CALLBACK handler, void *const pCBParam )
{
	ADI_GPIO_RESULT result;
	 //�Ǽ�IRQ�ص�����
	result = adi_gpio_RegisterCallback(ADI_GPIO_PIN_INTERRUPT_5,
										ADI_GPIO_PIN_8,
										handler,
										pCBParam);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Register_Callback_GPIO_PG08_INT : %d\n\n",result);
	}
}

void Enable_GPIO_PG08_INT(bool enable)
{
	ADI_GPIO_RESULT result;

	//ʹ���ж�
	result = adi_gpio_EnablePinInterruptMask(ADI_GPIO_PIN_INTERRUPT_5,
											ADI_GPIO_PIN_8,
											enable);
	if(result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("Failed in function Enable_GPIO_PG08_INT : %d\n\n",result);
	}

}

