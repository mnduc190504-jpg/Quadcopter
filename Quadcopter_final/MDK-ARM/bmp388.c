
#include "BMP388.h"
#include <math.h>
#include "main.h"

static BMP388_CalibData calib;
static float P_ground = 0;
static float P_ground_ukf = 0;


static float altitude_lpf = 0;
static float altitude_lpf_ukf = 0;
const float alpha_z = 0.20f; 

static void BMP388_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR_76, reg, 1, data, len, 20);
}

static void BMP388_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, BMP388_I2C_ADDR_76, reg, 1, &data, 1, 20);
}

static void BMP388_Read_Calibration(I2C_HandleTypeDef *hi2c) {
    uint8_t reg_calib[21];
    BMP388_ReadRegs(hi2c, BMP388_REG_CALIB_DATA, reg_calib, 21);

    calib.par_t1 = (float)((uint16_t)(reg_calib[1] << 8 | reg_calib[0])) / powf(2.0f, -8.0f);
    calib.par_t2 = (float)((uint16_t)(reg_calib[3] << 8 | reg_calib[2])) / powf(2.0f, 30.0f);
    calib.par_t3 = (float)((int8_t)reg_calib[4]) / powf(2.0f, 48.0f);
    calib.par_p1 = ((float)((int16_t)(reg_calib[6] << 8 | reg_calib[5])) - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    calib.par_p2 = ((float)((int16_t)(reg_calib[8] << 8 | reg_calib[7])) - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    calib.par_p3 = (float)((int8_t)reg_calib[9]) / powf(2.0f, 32.0f);
    calib.par_p4 = (float)((int8_t)reg_calib[10]) / powf(2.0f, 37.0f);
    calib.par_p5 = (float)((uint16_t)(reg_calib[12] << 8 | reg_calib[11])) / powf(2.0f, -3.0f);
    calib.par_p6 = (float)((uint16_t)(reg_calib[14] << 8 | reg_calib[13])) / powf(2.0f, 6.0f);
    calib.par_p7 = (float)((int8_t)reg_calib[15]) / powf(2.0f, 8.0f);
    calib.par_p8 = (float)((int8_t)reg_calib[16]) / powf(2.0f, 15.0f);
    calib.par_p9 = (float)((int16_t)(reg_calib[18] << 8 | reg_calib[17])) / powf(2.0f, 48.0f);
    calib.par_p10 = (float)((int8_t)reg_calib[19]) / powf(2.0f, 48.0f);
    calib.par_p11 = (float)((int8_t)reg_calib[20]) / powf(2.0f, 65.0f);
}

uint8_t BMP388_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id;

    BMP388_WriteReg(hi2c, 0x7E, 0xB6); // Soft Reset
    HAL_Delay(100); 

    BMP388_ReadRegs(hi2c, 0x00, &chip_id, 1);
    if (chip_id != 0x50) return 0;

    BMP388_Read_Calibration(hi2c);

    // 1. OSR: High Resolution (Áp su?t x8, Nhi?t d? x1) - Th?i gian do ~21.5ms
    BMP388_WriteReg(hi2c, 0x1C, 0x03); 
//	  BMP388_WriteReg(hi2c, 0x1C, 0x0C); 
	  HAL_Delay(10);

    // 2. ODR: 25Hz 
    BMP388_WriteReg(hi2c, 0x1D, 0x03); 
//    BMP388_WriteReg(hi2c, 0x1D, 0x04); 
	  HAL_Delay(10);

    // 3. IIR Filter 
    //BMP388_WriteReg(hi2c, 0x1F, 0x08);
    BMP388_WriteReg(hi2c, 0x1F, 0x04);
    HAL_Delay(10);		

    // 4. Kích ho?t Normal Mode
    BMP388_WriteReg(hi2c, 0x1B, 0x33);
    HAL_Delay(1000);
    
    return 1;
}

uint8_t BMP388_Read_Data(I2C_HandleTypeDef *hi2c, float *press, float *temp) {
    uint8_t status;
    BMP388_ReadRegs(hi2c, 0x03, &status, 1);
    
    if (!(status & 0x20)) return 0; 

    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR_76, 0x04, 1, raw, 6, 20) != HAL_OK) return 0;

    uint32_t raw_p = (uint32_t)((raw[2] << 16) | (raw[1] << 8) | raw[0]);
    uint32_t raw_t = (uint32_t)((raw[5] << 16) | (raw[4] << 8) | raw[3]);

    if (raw_p == 0) return 0;

    // Bù nhi?t d?
    float p1 = (float)raw_t - calib.par_t1;
    float p2 = (float)(p1 * calib.par_t2);
    calib.t_lin = p2 + (p1 * p1) * calib.par_t3;
    *temp = calib.t_lin;

    // Bù áp su?t[cite: 1]
    float d1, d2, d3, d4, o1, o2;
    d1 = calib.par_p6 * calib.t_lin;
    d2 = calib.par_p7 * (calib.t_lin * calib.t_lin);
    d3 = calib.par_p8 * (calib.t_lin * calib.t_lin * calib.t_lin);
    o1 = calib.par_p5 + d1 + d2 + d3;
    d1 = calib.par_p2 * calib.t_lin;
    d2 = calib.par_p3 * (calib.t_lin * calib.t_lin);
    d3 = calib.par_p4 * (calib.t_lin * calib.t_lin * calib.t_lin);
    o2 = (float)raw_p * (calib.par_p1 + d1 + d2 + d3);
    d1 = (float)raw_p * (float)raw_p;
    d2 = calib.par_p9 + calib.par_p10 * calib.t_lin;
    d3 = d1 * d2;
    d4 = d3 + ((float)raw_p * (float)raw_p * (float)raw_p) * calib.par_p11;
    *press = o1 + o2 + d4;
    
    return 1;
}

void BMP388_Set_Ground_Level(I2C_HandleTypeDef *hi2c) {
    float p_sum = 0, temp_p, temp_t;
    int count = 0;
    uint32_t timeout = HAL_GetTick();
    
    while (count < 100 && (HAL_GetTick() - timeout < 3000)) {
        if (BMP388_Read_Data(hi2c, &temp_p, &temp_t)) {
            p_sum += temp_p;
            count++;
        }
        HAL_Delay(20);
    }
    if (count > 0) P_ground = p_sum / (float)count;
    else P_ground = 101325.0f;
    
    altitude_lpf = 0; 
}
void BMP388_Set_Ground_Level_ukf(I2C_HandleTypeDef *hi2c) {
    float p_sum = 0, temp_p, temp_t;
    int count = 0;
    uint32_t timeout = HAL_GetTick();
    
    while (count < 100 && (HAL_GetTick() - timeout < 3000)) {
        if (BMP388_Read_Data(hi2c, &temp_p, &temp_t)) {
            p_sum += temp_p;
            count++;
        }
        HAL_Delay(20);
    }
    if (count > 0) P_ground_ukf = p_sum / (float)count;
    else P_ground_ukf = 101325.0f;
    
    altitude_lpf_ukf = 0; 
}

float BMP388_Get_Relative_Altitude(float current_pressure) {
    if (P_ground == 0) return 0;
    float alt_raw = 44330.0f * (1.0f - powf((current_pressure / P_ground), 0.190295f));
    
    // LPF
    altitude_lpf = (altitude_lpf * (1.0f - alpha_z)) + (alt_raw * alpha_z);
	
	  if (fabsf(altitude_lpf) < 0.03f) {
        return 0.0f;
    }
    
    return altitude_lpf;
}
float BMP388_Get_Relative_Altitude_ukf(float current_pressure) {
    if (P_ground_ukf == 0) return 0;
    float alt_raw = 44330.0f * (1.0f - powf((current_pressure / P_ground_ukf), 0.190295f));
    
    // LPF
    altitude_lpf_ukf = (altitude_lpf_ukf * (1.0f - alpha_z)) + (alt_raw * alpha_z);
	
	  if (fabsf(altitude_lpf_ukf) < 0.03f) {
        return 0.0f;
    }
    
    return altitude_lpf_ukf;
}
