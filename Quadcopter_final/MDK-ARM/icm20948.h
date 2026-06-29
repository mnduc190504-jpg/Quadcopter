#ifndef ICM20948_H
#define ICM20948_H

#include "stm32h7xx_hal.h"

#define ICM20948_ADDR       (0x68 << 1)
#define REG_BANK_SEL        0x7F

extern volatile float GyroRoll, GyroPitch, GyroYaw;
// Khai báo hàm chu?n
uint8_t ICM20948_Init(I2C_HandleTypeDef *hi2c);
void    ICM20948_Calibrate(I2C_HandleTypeDef *hi2c);
void    ICM20948_Process(I2C_HandleTypeDef *hi2c);

#endif

