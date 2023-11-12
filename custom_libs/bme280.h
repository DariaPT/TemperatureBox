#ifndef __BME280_h__
#define __BME280_h__

#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"



void bme280_init();
int8_t bme280_get_temp_in_celcius();

#endif
