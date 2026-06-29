#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32h7xx_hal.h"

void bmp280_init(I2C_HandleTypeDef *hi2c);
void bmp280_calibrate_sea_level_pressure(I2C_HandleTypeDef *hi2c, float known_altitude_m);
float bmp280_read_altitude(I2C_HandleTypeDef *hi2c);

#endif
