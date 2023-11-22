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

#include <string.h>

#include "FreeRTOS/Include/FreeRTOS.h"
#include "FreeRTOS/Include/queue.h"
#include "FreeRTOS/Include/task.h"

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

void USB_Init_Function()
{
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
}

void TaskSensorPoller(void *pvParameters)
{
	const double Kp = 4000;
	const double Kd = 0;
	const double Ki = 50;

	const double T_st = 50;

	double errorSum = 0.0;

    bme280_init();

    custom_pwm_init();

	while(1)
	{
		double currentTemperatureInCelcius = bme280_get_float_temp();
		uint16_t currentTemperatureInCelciusX100 = currentTemperatureInCelcius * 100;

		double currentError = T_st - currentTemperatureInCelcius;
		errorSum += currentError;
		double dError = (-1) * currentError;

		double newPwmValue = Kp * currentError + Kd * dError + Ki * errorSum;

		if (newPwmValue < 0) newPwmValue = 0;
		if (newPwmValue > 200) newPwmValue = 199;

		CUSTOM_PWM_SET_NEW_VALUE((uint16_t)newPwmValue);

		send_bytes_array_to_usb((uint8_t*)&currentTemperatureInCelciusX100, 2);

		vTaskDelay(500);
	}
}
int main(void)
{
	USB_Init_Function();
    __enable_irq();

    xTaskCreate(
    		TaskSensorPoller,
    		(signed char *) "TaskSensorPoller",
    		configMINIMAL_STACK_SIZE,
    		NULL,
    		2,
    		(TaskHandle_t *)NULL);

    vTaskStartScheduler();

    while (1)
    {

    }
}

void vApplicationTickHook(void)
{

}
