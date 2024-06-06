/**
  *****************************************************************************
  * @title   dht11.c
  * @author  muhittin_kaplan
  * @date    13 May 2014
  * @brief   DHT11 with stm32f4
  *******************************************************************************
  */
////// The above comment is automatically generated by CoIDE ///////////////////

#include "dh11_driver.h"
#include "math.h"

#include "stdbool.h"

#define GPIO_PORT	GPIOA
#define GPIO_PIN	GPIO_Pin_5
#define GPIO_RCC	RCC_APB2Periph_GPIOA

GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

void DHT11initTIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 72000000-1;//1us
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;		//1us counter
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
}

void DHT11initGPIOasOutput(void){

	 /* GPIOD Periph clock enable */
	RCC_APB2PeriphClockCmd(GPIO_RCC, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

}


void DHT11initGPIOasInput(void){

	 /* GPIOD Periph clock enable */
	RCC_APB2PeriphClockCmd(GPIO_RCC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

}

void DHT11_delay_us(int us){

	TIM2->CNT = 0;
	while((TIM2->CNT) <= us);
}

bool wait_for_pin_to_go_to_level(GPIO_TypeDef* gpioX, uint16_t gpioPin, bool desiredValue, uint16_t timeoutUs)
{
	TIM2->CNT = 0;
	while (GPIO_ReadInputDataBit(gpioX,gpioPin) != desiredValue)   // wait for the pin to go low. Wait for 80 us
	{
		uint16_t timCnt = TIM2->CNT;
		if(timCnt >= timeoutUs)
		{
			return false;
		}
	}
	return true;
}


int16_t DHT22_Check_Response (void)
{
	DHT11initGPIOasInput();   // set as input

	uint8_t Response = 0;
	DHT11_delay_us(40);  // wait for 40us

	if (!GPIO_ReadInputDataBit(GPIO_PORT,GPIO_PIN)) // if the pin is low
	{
		DHT11_delay_us(80);   // wait for 80us

		if (GPIO_ReadInputDataBit(GPIO_PORT,GPIO_PIN)) Response = 1;  // if the pin is high, response is ok
		else Response = -1;
	}

	if(!wait_for_pin_to_go_to_level(GPIO_PORT, GPIO_PIN, false, 80))
		return -1;

	return Response;
}

int16_t DHT22_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		// 50
		if(!wait_for_pin_to_go_to_level(GPIO_PORT, GPIO_PIN, true, 60))
			return -1;

		DHT11_delay_us(40);   // wait for 40 us

		if (!GPIO_ReadInputDataBit(GPIO_PORT,GPIO_PIN))   // if the pin is low
		{
			i &= ~(1<<(7-j));   // write 0
		}
		else
		{
			i |= (1<<(7-j));  // if the pin is high, write 1
		}

		// wait for 70-40 = 30
		if(!wait_for_pin_to_go_to_level(GPIO_PORT, GPIO_PIN, false, 40))
			return -1;
	}

	return i;
}

void DHT22_Start (void)
{
	DHT11initGPIOasOutput(); // set the pin as output
	GPIO_ResetBits(GPIO_PORT,GPIO_PIN); // pull the pin low
	DHT11_delay_us(1200);   // wait for > 1ms
	GPIO_SetBits(GPIO_PORT,GPIO_PIN);// pull the pin high
	DHT11_delay_us(20);   // wait for 30us

	DHT11initGPIOasInput();   // set as input
}
