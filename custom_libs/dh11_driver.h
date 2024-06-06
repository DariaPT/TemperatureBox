/**
  *****************************************************************************
  * @title   dht11.h
  * @author  muhittin_kaplan
  * @date    13 May 2014
  * @brief   DHT11 with stm32f4
  *******************************************************************************
  */
////// The above comment is automatically generated by CoIDE ///////////////////

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "stdio.h"
#include "math.h"

int16_t DHT22_Check_Response (void);
int16_t DHT22_Read (void);
void DHT22_Start (void);
