#ifndef __BME280_h__
#define __BME280_h__

#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"



void bme280_init();
int8_t bme280_get_temp_in_celcius();
double bme280_get_float_temp();
int32_t get_remaped_value_for_zero_min_border(
		int8_t curTempInCelc,
		int8_t maxTempValue,
		int32_t maxRemapValue);

#endif
