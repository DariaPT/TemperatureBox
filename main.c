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

#include "FreeRTOS/Include/semphr.h"

#include "custom_pwm.h"

#define USE_DHT22_SENSOR 1
#define USE_DHT11_SENSOR 0

#if USE_DHT22_SENSOR == 1 || USE_DHT11_SENSOR == 1
#include "dh11_driver.h"
#else
#include "bme280.h"
#endif

#define PUMP_WORK_TIME_MS 15000

#define START_MEASUREMENT_INPUT_PORT_RCC RCC_APB2Periph_GPIOA
#define START_MEASUREMENT_INPUT_PORT GPIOA
#define START_MEASUREMENT_INPUT_PIN GPIO_Pin_0

#define VALVES_OUTPUT_PORT GPIOA
#define VALVES_OUTPUT_PIN GPIO_Pin_1

#define PUMP_OUTPUT_PORT GPIOA
#define PUMP_OUTPUT_PIN GPIO_Pin_2

#define OUTPUT_SIGNAL_OUTPUT_PORT GPIOA
#define OUTPUT_SIGNAL_OUTPUT_PIN GPIO_Pin_3

#define PUMP_START() GPIO_SetBits(PUMP_OUTPUT_PORT, PUMP_OUTPUT_PIN)
#define PUMP_STOP() GPIO_ResetBits(PUMP_OUTPUT_PORT, PUMP_OUTPUT_PIN)

#define OPEN_VALVES() GPIO_SetBits(VALVES_OUTPUT_PORT, VALVES_OUTPUT_PIN)
#define CLOSE_VALVES() GPIO_ResetBits(VALVES_OUTPUT_PORT, VALVES_OUTPUT_PIN)

#define SET_OUTPUT_SIGNAL() GPIO_SetBits(OUTPUT_SIGNAL_OUTPUT_PORT, OUTPUT_SIGNAL_OUTPUT_PIN)
#define RESET_OUTPUT_SIGNAL() GPIO_ResetBits(OUTPUT_SIGNAL_OUTPUT_PORT, OUTPUT_SIGNAL_OUTPUT_PIN)

ADC_InitTypeDef ADC_InitStructure;
GPIO_InitTypeDef InitStruct;
xQueueHandle temperatureQueue;

xSemaphoreHandle readyToMeasureSemaphoreHandle;

volatile bool isReadyToHeat = false;

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

void MainTask(void *pvParameters)
{
	bool isMeasurementStarted = false;
	isReadyToHeat = false;
	while(1)
	{
		if(!isReadyToHeat && GPIO_ReadInputDataBit(START_MEASUREMENT_INPUT_PORT, START_MEASUREMENT_INPUT_PIN) == 1)
		{
			// Запустить измерение
			send_bytes_array_to_usb("READY TO MEASURE\r\n", sizeof("READY TO MEASURE"));

			OPEN_VALVES();
			PUMP_START();

			vTaskDelay(PUMP_WORK_TIME_MS);

			CLOSE_VALVES();
			PUMP_STOP();

			isReadyToHeat = true;
			//xSemaphoreGive(readyToMeasureSemaphoreHandle);
		}
		vTaskDelay(100);
	}
}

void TaskSensorPoller(void *pvParameters)
{
	u8 Rh,RhDec,Temp,TempDec,ChkSum;

	int16_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;

	uint16_t temperatureU16;
	uint8_t Presence = 0;

	while(1)
	{
		if(isReadyToHeat)
		{
			//xSemaphoreTake(readyToMeasureSemaphoreHandle, portMAX_DELAY);

#if USE_DHT11_SENSOR
			DHT11Read(&Rh,&RhDec,&Temp,&TempDec,&ChkSum);
			double currentTemperatureInCelcius = (double)Temp;
#elif USE_DHT22_SENSOR

			DHT22_Start();
			Presence = DHT22_Check_Response();
			Rh_byte1 = DHT22_Read();
			Rh_byte2 = DHT22_Read();
			Temp_byte1 = DHT22_Read();
			Temp_byte2 = DHT22_Read();

			if(Presence == -1 || Rh_byte1 == -1 ||
					Rh_byte2 == -1 || Temp_byte1 == -1 || Temp_byte2 == -1)
			{
				continue;
			}

			temperatureU16 = ((uint8_t)Temp_byte1 << 8) | (uint8_t)Temp_byte2;

			double currentTemperatureInCelcius = temperatureU16 / 10.0;
#else
			double currentTemperatureInCelcius = bmp280_get_float_temp();
#endif


			xQueueSendToBack(temperatureQueue, (void *)&currentTemperatureInCelcius, 0);
		}

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

		if(isReadyToHeat)
		{
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

			if(newTemperatureInCelcius >= 50)
			{
				CUSTOM_PWM_SET_NEW_VALUE(0); // Отключаем нагрев.

				SET_OUTPUT_SIGNAL();

				vTaskDelay(30000);

				RESET_OUTPUT_SIGNAL();

				isReadyToHeat = false;

			}
		}
	}
}

void init_gpio()
{
	RCC_AHBPeriphClockCmd(START_MEASUREMENT_INPUT_PORT_RCC, ENABLE);
	//RCC_AHBPeriphClockCmd(OUTPUT_SIGNAL_OUTPUT_RCC, ENABLE);

	GPIO_InitTypeDef gpioStruct;
	// Входной сигнал
	gpioStruct.GPIO_Mode = GPIO_Mode_IPD;
	gpioStruct.GPIO_Pin = START_MEASUREMENT_INPUT_PIN;
	gpioStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(START_MEASUREMENT_INPUT_PORT, &gpioStruct);

	// Клапаны
	gpioStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioStruct.GPIO_Pin = VALVES_OUTPUT_PIN;
	gpioStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(VALVES_OUTPUT_PORT, &gpioStruct);

	// Насос
	gpioStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioStruct.GPIO_Pin = PUMP_OUTPUT_PIN;
	gpioStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(PUMP_OUTPUT_PORT, &gpioStruct);

	// Выходной сигнал
	gpioStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioStruct.GPIO_Pin = OUTPUT_SIGNAL_OUTPUT_PIN;
	gpioStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(OUTPUT_SIGNAL_OUTPUT_PORT, &gpioStruct);

	GPIO_Write(GPIOA, 0);
}

int main(void)
{
	USB_Init_Function();
    __enable_irq();

#if USE_DHT22_SENSOR || USE_DHT11_SENSOR
	DHT11initTIM2();
#else
	bmp280_init();
#endif


    custom_pwm_init();
    temperatureQueue = xQueueCreate(1, sizeof(double));

    init_gpio();

    vSemaphoreCreateBinary(readyToMeasureSemaphoreHandle);

    xTaskCreate(
    		MainTask,
    		(signed char *) "MainTask",
    		configMINIMAL_STACK_SIZE,
    		NULL,
    		2,
    		(TaskHandle_t *)NULL);

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
