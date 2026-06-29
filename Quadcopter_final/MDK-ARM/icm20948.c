#include "icm20948.h"
#include "main.h"  
#include <math.h>

static float GyroX_Offset = 0.0f;
static float GyroY_Offset = 0.0f;
static float Roll_Offset = 0.0f;
static float Pitch_Offset = 0.0f;
static float Yaw_Gyro_Offset = 0.0f; //offset cho Yaw
volatile float GyroRoll, GyroPitch, GyroYaw;
static uint32_t last_tick = 0;


uint8_t ICM20948_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t val = 0;
    // Bank 0, Reset va Wake up
    val = 0x00; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &val, 1, 10);
    val = 0x80; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x06, 1, &val, 1, 10); 
    HAL_Delay(50);
    val = 0x01; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x06, 1, &val, 1, 10); 

    // Bank 2 cau hinh Scale
    val = 0x20; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &val, 1, 10);
    val = 0x07; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x01, 1, &val, 1, 10); // Gyro +/-2000dps (16.4 LSB/dps)
    val = 0x07; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x14, 1, &val, 1, 10); // Accel +/-16g (2048 LSB/g)
    
    // Quay ve Bank 0
    val = 0x00; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &val, 1, 10);
    
    last_tick = DWT->CYCCNT;
    return 1;
}

void ICM20948_Calibrate(I2C_HandleTypeDef *hi2c) {
    float r_sum = 0, p_sum = 0, y_gyro_sum = 0;
	  float gx_sum = 0, gy_sum = 0;
    int samples = 1000; 
    
    Roll_Offset = 0; Pitch_Offset = 0; Yaw_Gyro_Offset = 0;
    
    for(int i = 0; i < samples; i++) {
        uint8_t raw[12];
        if(HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, 0x2D, 1, raw, 12, 10) == HAL_OK) {
            float ax = (int16_t)(raw[0] << 8 | raw[1]) / 2048.0f;
            float ay = (int16_t)(raw[2] << 8 | raw[3]) / 2048.0f;
            float az = (int16_t)(raw[4] << 8 | raw[5]) / 2048.0f;
					  float gx_raw = (int16_t)(raw[6] << 8 | raw[7]) / 16.4f;
            float gy_raw = (int16_t)(raw[8] << 8 | raw[9]) / 16.4f;
            float gz_raw = (int16_t)(raw[10] << 8 | raw[11]) / 16.4f;

            r_sum += atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
            p_sum += atan2f(ay, az) * 57.29578f;
            y_gyro_sum += gz_raw;
					  
					  gx_sum += gx_raw; // Thêm bi?n gx_sum, gy_sum vào hàm
            gy_sum += gy_raw;
        }
        HAL_Delay(1); 
    }
    Roll_Offset = r_sum / (float)samples;
    Pitch_Offset = p_sum / (float)samples;
    Yaw_Gyro_Offset = y_gyro_sum / (float)samples; 
		GyroX_Offset = gx_sum / (float)samples; 
    GyroY_Offset = gy_sum / (float)samples;
}

void ICM20948_Process(I2C_HandleTypeDef *hi2c) {
    uint8_t raw[12];
	  static float ax_f = 0, ay_f = 0, az_f = 1.0f; 
	  static float gx_f = 0, gy_f = 0;
    const float alpha = 0.05f;
	  const float alpha_gyro = 0.15f; 
    if(HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, 0x2D, 1, raw, 12, 10) != HAL_OK) return;

    // Tinh Delta Time (dt)
    uint32_t now = DWT->CYCCNT;
    float dt = (float)(now - last_tick) / (float)SystemCoreClock;
    last_tick = now;
    
    if (dt <= 0 || dt > 0.05f) dt = 0.001f;

    // raw
    float ax_raw = (int16_t)(raw[0] << 8 | raw[1]) / 2048.0f;
    float ay_raw = (int16_t)(raw[2] << 8 | raw[3]) / 2048.0f;
    float az_raw = (int16_t)(raw[4] << 8 | raw[5]) / 2048.0f;

    // Bo loc thong thap
    ax_f = ax_f + alpha * (ax_raw - ax_f);
    ay_f = ay_f + alpha * (ay_raw - ay_f);
    az_f = az_f + alpha * (az_raw - az_f); 
    
//    float gx_raw = (int16_t)(raw[6] << 8 | raw[7]) / 16.4f;
//    float gy_raw = (int16_t)(raw[8] << 8 | raw[9]) / 16.4f;
    float gz_raw = (int16_t)(raw[10] << 8 | raw[11]) / 16.4f;
//float gx_fixed = ((int16_t)(raw[6] << 8 | raw[7]) / 16.4f) - GyroX_Offset;
//float gy_fixed = ((int16_t)(raw[8] << 8 | raw[9]) / 16.4f) - GyroY_Offset;
// raw
float gx_raw_corr = ((int16_t)(raw[6] << 8 | raw[7]) / 16.4f) - GyroX_Offset;
float gy_raw_corr = ((int16_t)(raw[8] << 8 | raw[9]) / 16.4f) - GyroY_Offset;

// Bo loc
gx_f = gx_f + alpha_gyro * (gx_raw_corr - gx_f);
gy_f = gy_f + alpha_gyro * (gy_raw_corr - gy_f);


float gx_fixed = gx_f;
float gy_fixed = gy_f;


    // Tinh goc 
//    float roll_acc  = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 57.29578f; 
//    float pitch_acc = atan2f(ay_f, az_f) * 57.29578f;
float roll_acc_final  = (atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 57.29578f) - Roll_Offset;
float pitch_acc_final = (atan2f(ay_f, az_f) * 57.29578f) - Pitch_Offset;

    // Roll, Pitch 
//    Roll  = 0.995f * (Roll + gy_raw * dt) + 0.005f * (roll_acc - Roll_Offset);
//    Pitch = 0.995f * (Pitch + gx_raw * dt) + 0.005f * (pitch_acc - Pitch_Offset);
Roll  = 0.998f * (Roll + gy_fixed * dt) + 0.002f * roll_acc_final;
Pitch = 0.998f * (Pitch + gx_fixed * dt) + 0.002f * pitch_acc_final;
    
    // --- FIX YAW ---
    static float gz_lpf = 0;

    float gz_current = gz_raw - Yaw_Gyro_Offset;



    gz_lpf = (gz_lpf * 0.9f) + (gz_current * 0.1f);


    float gz_final = 0;
    if (fabsf(gz_lpf) > 0.15f) {
        gz_final = gz_lpf;
    } else {
        gz_final = 0.0f;
    }

    Yaw += gz_final * dt;
    // -----------------------

//    GyroRoll = gy_raw * 0.017453f; 
//    GyroPitch = gx_raw * 0.017453f; 
		GyroRoll  = gy_fixed * 0.017453f; 
    GyroPitch = gx_fixed * 0.017453f;
    GyroYaw = gz_final * 0.017453f; 
}
