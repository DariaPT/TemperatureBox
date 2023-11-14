#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "math.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "stm32f10x_i2c.h"

#include "stm32f10x_tim.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bme280.h"


ADC_InitTypeDef ADC_InitStructure;
GPIO_InitTypeDef InitStruct;

TIM_TimeBaseInitTypeDef timer1;
TIM_OCInitTypeDef timer1Pwm;

uint8_t res = 0;
uint8_t i = 0;

void _delay_ms(uint32_t t)
{
	while(t--){}
}

void send_bytes_array_to_usb(uint8_t *data, uint32_t dataSize)
{
	for (int i = 0; i < dataSize; i++)
	{
		USB_Send_Data(data[i]);
	}
}


int usb_printf(char *format, ...)
{
	char printBuf[256] = { 0 };

	va_list ap ;

	va_start(ap, format); /* ��������� ��������� ��������� �������� (����� �������� ����� ������� ������������) */

	vsprintf(printBuf, format, ap);
	send_bytes_array_to_usb((uint8_t *)printBuf, strlen(printBuf));

	va_end(ap);

	return 0;
}

void TIM1_IRQHandler()
{
	//res = (i-63)*(i-63)/17;
	res = 50*(1 - exp(-0.02*i));
	i++;


	USB_Send_Data(res);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void USB_Init_Function()
{
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
}

int main(void)
{
	USB_Init_Function();

    __enable_irq();

    bme280_init();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1, ENABLE);

    GPIO_InitTypeDef gpioStruct;
    GPIO_StructInit(&gpioStruct);
    gpioStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioStruct.GPIO_Pin = GPIO_Pin_7;
    gpioStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpioStruct);

	TIM_TimeBaseStructInit(&timer1);
	timer1.TIM_Prescaler = 720;
	timer1.TIM_Period = 250;
	TIM_TimeBaseInit(TIM1, &timer1);

	TIM_OCStructInit(&timer1Pwm);
	timer1Pwm.TIM_Pulse = 50;
	timer1Pwm.TIM_OCMode = TIM_OCMode_PWM1;
	timer1Pwm.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM1, &timer1Pwm);

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	NVIC_EnableIRQ(TIM1_IRQn);


    while (1)
    {
    	int8_t currentTemperatureInCelcius = bme280_get_temp_in_celcius();

		USB_Send_Data(currentTemperatureInCelcius);

    	_delay_ms(10000000);
    }
}
