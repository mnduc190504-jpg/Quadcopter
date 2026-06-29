#include "bmp280.h"
#include <math.h>
#include <stdio.h>

#define BMP280_ADDR        (0x76 << 1)
#define REG_PRESS_MSB      0xF7
#define REG_TEMP_MSB       0xFA
#define REG_CTRL_MEAS      0xF4
#define REG_CONFIG         0xF5
#define REG_CALIB          0x88

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

static bmp280_calib_t calib;
static int32_t t_fine;
static float sea_level_pressure = 101325.0f;

void bmp280_init(I2C_HandleTypeDef *hi2c) {
    uint8_t config[2];

    // C?u hình config register
    config[0] = REG_CONFIG;
    config[1] = 0x00;
    HAL_I2C_Master_Transmit(hi2c, BMP280_ADDR, config, 2, HAL_MAX_DELAY);

    // Ð?c h? s? hi?u ch?nh
    uint8_t calib_data[24];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, REG_CALIB, 1, calib_data, 24, HAL_MAX_DELAY);

    calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];
}

static int32_t compensate_temp(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12)
                    * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

static int64_t compensate_press(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return p >> 8;  // Tr? v? don v? Pa
}

void bmp280_calibrate_sea_level_pressure(I2C_HandleTypeDef *hi2c, float known_altitude_m) {
    uint8_t ctrl_meas[2] = { REG_CTRL_MEAS, 0x25 };
    HAL_I2C_Master_Transmit(hi2c, BMP280_ADDR, ctrl_meas, 2, HAL_MAX_DELAY);
    HAL_Delay(20);

    uint8_t data[6];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, REG_PRESS_MSB, 1, data, 6, HAL_MAX_DELAY);

    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

    compensate_temp(adc_T);
    int64_t pressure = compensate_press(adc_P);

    sea_level_pressure = (float)pressure / powf(1.0f - (known_altitude_m / 44330.0f), 5.255f);
}

float bmp280_read_altitude(I2C_HandleTypeDef *hi2c) {
    uint8_t ctrl_meas[2] = { REG_CTRL_MEAS, 0x25 };
    HAL_I2C_Master_Transmit(hi2c, BMP280_ADDR, ctrl_meas, 2, HAL_MAX_DELAY);
    HAL_Delay(20);

    uint8_t data[6];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, REG_PRESS_MSB, 1, data, 6, HAL_MAX_DELAY);

    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

    compensate_temp(adc_T);
    int64_t pressure_pa = compensate_press(adc_P);

    float altitude = 44330.0f * (1.0f - powf(pressure_pa / sea_level_pressure, 0.1903f));
    return altitude;
}
