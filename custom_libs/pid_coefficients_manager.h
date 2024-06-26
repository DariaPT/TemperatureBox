#ifndef __PID_COEFFICIENTS_MANAGER_H__
#define __PID_COEFFICIENTS_MANAGER_H__

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#include "misc.h"
#include "stdio.h"
#include "math.h"

struct PidCoefficients
{
	float P;
	float I;
	float D;
	float T_st;
}
__attribute__((packed));

void callback_on_new_raw_coef_received(uint8_t *rawCoefBuf);

#endif
