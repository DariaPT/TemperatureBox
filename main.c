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
#include "custom_pwm.h"


ADC_InitTypeDef ADC_InitStructure;
GPIO_InitTypeDef InitStruct;

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

int main(void)
{
	USB_Init_Function();

    __enable_irq();

    bme280_init();

    custom_pwm_init();

    while (1)
    {
    	int8_t currentTemperatureInCelcius = bme280_get_temp_in_celcius();

    	uint8_t newPwmValue =
    			get_remaped_value_for_zero_min_border(currentTemperatureInCelcius, 40, 255);

    	CUSTOM_PWM_SET_NEW_VALUE(newPwmValue);

//		for (int var = 0; var < 256; ++var)
//		{
//			CUSTOM_PWM_SET_NEW_VALUE(var);
//			_delay_ms(10000);
//		}
//
//		for (int var = 255; var >= 0; var--)
//		{
//			CUSTOM_PWM_SET_NEW_VALUE(var);
//			_delay_ms(10000);
//		}

    	USB_Send_Data(currentTemperatureInCelcius);
    	_delay_ms(1000000);
    }
}
