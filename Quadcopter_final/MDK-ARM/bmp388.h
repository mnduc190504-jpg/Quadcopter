#ifndef BMP388_H
#define BMP388_H

#include "stm32h7xx_hal.h"

/* Ð?a ch? I2C */
#define BMP388_I2C_ADDR_76      (0x76 << 1)

/* Thanh ghi */
#define BMP388_REG_CMD          0x7E
#define BMP388_CMD_SOFT_RESET   0xB6
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_REG_DATA_0       0x04
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CONFIG       0x1F
#define BMP388_REG_CALIB_DATA   0x31

typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin;
} BMP388_CalibData;

/* Khai báo hàm */
uint8_t BMP388_Init(I2C_HandleTypeDef *hi2c);
uint8_t    BMP388_Read_Data(I2C_HandleTypeDef *hi2c, float *press, float *temp);
void    BMP388_Set_Ground_Level(I2C_HandleTypeDef *hi2c);
void    BMP388_Set_Ground_Level_ukf(I2C_HandleTypeDef *hi2c);
float   BMP388_Get_Relative_Altitude(float current_pressure);
float   BMP388_Get_Relative_Altitude_ukf(float current_pressure);


#endif
