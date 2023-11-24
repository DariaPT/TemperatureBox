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
xQueueHandle temperatureQueue;

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
	while(1)
	{
		double currentTemperatureInCelcius = bmp280_get_float_temp();
		uint16_t currentTemperatureInCelciusX100 = currentTemperatureInCelcius * 100;

		xQueueSendToBack(temperatureQueue, (void *)&currentTemperatureInCelcius, 0);

		vTaskDelay(100);
	}
}

void PidRegulator(void *pvParameters)
{
	const double T_st = 50;
	const double Kp = 105; // 108 XENM PF[JLBN
	const double Kd = 0;
	const double Ki = 0.2;
	double I_prev = 0;
	double prevError = 0;

	while(1)
	{
		double newTemperatureInCelcius = 0;
		xQueueReceive(temperatureQueue, &newTemperatureInCelcius, portMAX_DELAY );
		double currentError = T_st - newTemperatureInCelcius;
		double P_part = Kp * currentError;
		double I_part = I_prev + Ki * currentError;
		double D_part = Kd * (currentError - prevError);
		double newPwmValue = P_part + I_part + D_part;
		if (newPwmValue < 0) newPwmValue = 0;
		if (newPwmValue > 200) newPwmValue = 199;

		CUSTOM_PWM_SET_NEW_VALUE((uint16_t)newPwmValue);

		prevError = currentError;
		I_prev = I_part;
		uint16_t currentTemperatureInCelciusX100 = newTemperatureInCelcius * 100;
		send_bytes_array_to_usb((uint8_t*)&currentTemperatureInCelciusX100, 2);
	}
}

int main(void)
{
	USB_Init_Function();
    __enable_irq();
    bmp280_init();
    custom_pwm_init();
    temperatureQueue = xQueueCreate(1, sizeof(double));

    xTaskCreate(
    		TaskSensorPoller,
    		(signed char *) "TaskSensorPoller",
    		configMINIMAL_STACK_SIZE,
    		NULL,
    		2,
    		(TaskHandle_t *)NULL);
    xTaskCreate(
    		PidRegulator,
    		(signed char *) "PidRegulator",
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
