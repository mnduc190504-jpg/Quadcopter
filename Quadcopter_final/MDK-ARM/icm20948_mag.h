#ifndef ICM20948_MAG_H
#define ICM20948_MAG_H

#include "main.h"

extern volatile float GyroRoll, GyroPitch, GyroYaw;
extern volatile float accel_roll, accel_pitch, accel_yaw;
extern float Mag_Delta;       
extern float Yaw_Drift;

/* Bi?n Debug */
extern int16_t mag_x, mag_y, mag_z;

/* Register Banks & Addresses (D?a theo snippet b?n g?i) */
#define ICM20948_ADDR         (0x68 << 1)
#define AK09916_I2C_ADDR      0x0C

#define REG_BANK_SEL          0x7F
#define USER_BANK_0           0x00
#define USER_BANK_2           0x20
#define USER_BANK_3           0x30

#define REG_WHO_AM_I          0x00
#define REG_USER_CTRL         0x03
#define REG_PWR_MGMT_1        0x06
#define REG_PWR_MGMT_2        0x07
#define REG_ACCEL_XOUT_H      0x2D
#define REG_EXT_SLV_SENS_DATA_00  0x3B

#define AK09916_CNTL2         0x31

/* Function Prototypes */
uint8_t ICM20948_Init_mag(I2C_HandleTypeDef *hi2c);
void    ICM20948_Calibrate_mag(I2C_HandleTypeDef *hi2c);
void    ICM20948_Process_mag(I2C_HandleTypeDef *hi2c);

#endif
