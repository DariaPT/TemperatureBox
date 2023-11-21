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
	t = t * 7200;
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
	va_list ap ;

	va_start(ap, format); /* пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ (пїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ) */
	char printBuf[256] = { 0 };
	vsprintf(printBuf, format, ap);
	va_end(ap);

	send_bytes_array_to_usb((uint8_t *)printBuf, strlen(printBuf));

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
	double T_st = 40;
	double x = 0.0;
	double Error = 0.0;

	double dt = 40;
	double Kp = 4000; //4000 - колебания

	double Ki = 0;
	USB_Init_Function();
    __enable_irq();

    bme280_init();

    custom_pwm_init();

    while (1)
    {
    	double currentTemperatureInCelcius = bme280_get_float_temp();
    	uint16_t currentTemperatureInCelciusX100 = currentTemperatureInCelcius * 100;

    	x = (T_st - currentTemperatureInCelcius)/T_st;
    	Error += x;
    	double newPwmValue = Kp * x + Ki * dt * Error;
    	if (newPwmValue < 0) newPwmValue = 0;
    	if (newPwmValue > 200) newPwmValue= 190;

    	CUSTOM_PWM_SET_NEW_VALUE((uint8_t)newPwmValue);

    	send_bytes_array_to_usb((uint8_t*)&currentTemperatureInCelciusX100, 2);

    	_delay_ms(100);
    }
}
