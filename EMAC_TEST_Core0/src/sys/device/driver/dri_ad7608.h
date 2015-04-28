/*
 * dri_ad7608.h
 *
 *  Created on: 2015-3-17
 *      Author: Administrator
 */

#ifndef DRI_AD7608_H_
#define DRI_AD7608_H_

#include <cdefbf609.h>
#include <ccblkfn.h>
#include <services/gpio/adi_gpio.h>

#if 0
/*
 * AD7608 Control pin
 */
#define RESET_HIGH()    *pREG_PORTD_DATA_SET = ADI_GPIO_PIN_3
#define RESET_LOW()	    *pREG_PORTD_DATA_CLR = ADI_GPIO_PIN_3
#define CONVERT_HIGH() 	*pREG_PORTD_DATA_SET = ADI_GPIO_PIN_9
#define CONVERT_LOW() 	*pREG_PORTD_DATA_CLR = ADI_GPIO_PIN_9


/*
 * AD7608���ݲɼ���ɵı�־�������ź��жϺ������ڴ˺����п���SPI����AD���ݵĶ�ȡ
 */
typedef void (*AD7608_Busy_ISR_Handler)(ADI_GPIO_PIN_INTERRUPT const ePinInt,
										const uint32_t event,
										void *pArg);

ADI_GPIO_RESULT Init_Busy_IO_IRQ(void);

uint32_t Init_AD7608_IO(void);

void Register_Buzy_IO_Callback( AD7608_Busy_ISR_Handler handler );

void Enable_Buzy_IO_Interrupt(bool enable);

/*
 * ����AD7608���ص�������8ͨ��18λ���з��������ڴ���λ��ʽ����һ��18-Bit�Ļ����������ͣ�
 */
typedef struct
{
	signed       ad : 18;
	unsigned     pad: 14;
} AD7608_DATA;

#else

typedef struct
{
	signed       ad : 18;
	unsigned     pad: 14;
} AD7608_DATA;

/*
 * AD7608���ݲɼ���ɵı�־�������ź��жϺ������ڴ˺����п���SPI����AD���ݵĶ�ȡ
 */
typedef void (*AD7608_Busy_ISR_Handler)(ADI_GPIO_PIN_INTERRUPT const ePinInt,
										const uint32_t event,
										void *pArg);

ADI_GPIO_RESULT Init_Busy_IO_IRQ(void);

uint32_t Init_AD7608_IO(void);


void Register_Buzy_IO_Callback( AD7608_Busy_ISR_Handler handler );

void Enable_Buzy_IO_Interrupt(bool enable);

void Start_AD7608(void);

#endif



#endif /* DEV_AD7608_H_ */
