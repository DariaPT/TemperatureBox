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

uint16_t previousState;
GPIO_InitTypeDef port;
TIM_TimeBaseInitTypeDef timer;
TIM_OCInitTypeDef timerPWM;
uint16_t buttonPreviousState;

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


void USB_Init_Function()
{
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
}

void TIM3_IRQHandler()
{
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

int main(void)
{
	USB_Init_Function();

    __enable_irq();

    bme280_init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_1;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &port);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 720; // 1 tick per 0.01 ms
    timer.TIM_Period = 255;
    TIM_TimeBaseInit(TIM3, &timer);

    TIM_OCStructInit(&timerPWM);
    timerPWM.TIM_Pulse = 50; // 0.5 ms
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC4Init(TIM3, &timerPWM);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);


    while (1)
    {
    	int8_t currentTemperatureInCelcius = bme280_get_temp_in_celcius();


    	static uint8_t counter = 0;
        TIM3->CCR4 = counter;
        if(++counter >= 255)
        {
        	counter = 0;
        }

		for (int var = 0; var < 256; ++var)
		{
			TIM3->CCR4 = var;
			_delay_ms(10000);
		}

		for (int var = 255; var >= 0; var--)
		{
			TIM3->CCR4 = var;
			_delay_ms(10000);
		}

    	USB_Send_Data(currentTemperatureInCelcius);

    }
}
