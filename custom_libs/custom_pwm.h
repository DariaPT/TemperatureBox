#ifndef __CUSTOM_PWM_H__
#define __CUSTOM_PWM_H__

#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#define CUSTOM_PWM_SET_NEW_VALUE(_newValue_) TIM3->CCR4 = (_newValue_)

void custom_pwm_init();

#endif
