#include "ICM20948_mag.h"
#include <math.h>

static float mx_f = 0, my_f = 0;
const float alpha_mag = 0.01f;
int16_t mag_x = 0, mag_y = 0, mag_z = 0;

extern float Mag_Delta = 0.0f;       
extern float Yaw_Drift = 0.0f;       
static float Mag_Yaw_Initial = 0.0f; 
static float MagZ_Offset = 0.0f;
volatile float accel_roll, accel_pitch, accel_yaw;
static float GyroX_Offset = 0, GyroY_Offset = 0, Yaw_Gyro_Offset = 0;
static float AccelX_Offset = 0, AccelY_Offset = 0, AccelZ_Offset = 0;
static float Roll_Offset = 0, Pitch_Offset = 0;
static uint32_t last_tick = 0;


static void SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank) {
    static uint8_t current_bank = 0xFF;
    if (bank != current_bank) {
        HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_BANK_SEL, 1, &bank, 1, 10);
        current_bank = bank;
    }
}

/* 1. KHOI TAO */
uint8_t ICM20948_Init_mag(I2C_HandleTypeDef *hi2c) {
    uint8_t val;
    HAL_Delay(100);

    // Check WHO_AM_I
    SelectBank(hi2c, USER_BANK_0);
    if(HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, REG_WHO_AM_I, 1, &val, 1, 10) != HAL_OK) return 0;
    if(val != 0xEA) return 0;

    // Reset device
    val = 0x80; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_PWR_MGMT_1, 1, &val, 1, 10);
    HAL_Delay(100);
    
    // Wake up
    val = 0x01; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_PWR_MGMT_1, 1, &val, 1, 10);
    HAL_Delay(50);
    
    // Enable sensors
    val = 0x00; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_PWR_MGMT_2, 1, &val, 1, 10);
    HAL_Delay(50);
    
    // Enable I2C Master
    val = 0x20; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, REG_USER_CTRL, 1, &val, 1, 10);
    HAL_Delay(10);
    
    // Config Gyro/Accel (Bank 2)
    SelectBank(hi2c, USER_BANK_2);
    val = 0x27; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x01, 1, &val, 1, 10); // GYRO_CONFIG_1
    HAL_Delay(10);
    val = 0x27; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x14, 1, &val, 1, 10); // ACCEL_CONFIG
    HAL_Delay(10);
    
    // Config Magnetometer (Bank 3)
    SelectBank(hi2c, USER_BANK_3);
    val = 0x07; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x01, 1, &val, 1, 10); // I2C_MST_CTRL
    val = 0x01; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x02, 1, &val, 1, 10); // Delay control
    HAL_Delay(10);
    
    // Set AK09916 Continuous Mode
    val = AK09916_I2C_ADDR; HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x03, 1, &val, 1, 10); // SLV0_ADDR
    val = AK09916_CNTL2;    HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x04, 1, &val, 1, 10); // SLV0_REG
    val = 0x08;             HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x06, 1, &val, 1, 10); // SLV0_DO (Mode 2)
    val = 0x81;             HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x05, 1, &val, 1, 10); // SLV0_CTRL
    HAL_Delay(100); 
    
    // Read Mode Setup (9 bytes t? ST1)
    val = 0x8C;             HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x03, 1, &val, 1, 10);
    val = 0x10;             HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x04, 1, &val, 1, 10); // Start from ST1
    val = 0x89;             HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, 0x05, 1, &val, 1, 10); // 9 bytes
    HAL_Delay(10);
    
    SelectBank(hi2c, USER_BANK_0);
    last_tick = DWT->CYCCNT;
    return 1;
}

void ICM20948_Calibrate_mag(I2C_HandleTypeDef *hi2c) {
    float r_sum = 0, p_sum = 0, y_gyro_sum = 0, gx_sum = 0, gy_sum = 0;
    float mx_sum = 0, my_sum = 0, mz_sum = 0; 
	  // === THÊM BI?N LUU T?NG GIA T?C ===
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    int samples = 1000; 
    
    SelectBank(hi2c, USER_BANK_0);
    

for(int i = 0; i < samples + 100; i++) {
        uint8_t raw[23]; 
        if(HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, 0x2D, 1, raw, 23, 10) == HAL_OK) {
            if (i < 100) continue; 

            // --- ACCEL / GYRO ---
            float ax = (int16_t)(raw[0] << 8 | raw[1]) / 2048.0f;
            float ay = (int16_t)(raw[2] << 8 | raw[3]) / 2048.0f;
            float az = (int16_t)(raw[4] << 8 | raw[5]) / 2048.0f;
					  // === C?NG D?N GIA T?C ===
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
					
            gx_sum += (int16_t)(raw[6] << 8 | raw[7]) / 16.4f;
            gy_sum += (int16_t)(raw[8] << 8 | raw[9]) / 16.4f;
            y_gyro_sum += (int16_t)(raw[10] << 8 | raw[11]) / 16.4f;
            r_sum += atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
            p_sum += atan2f(ay, az) * 57.29578f;

            // --- MAGNETOMETER ---
            int16_t mx_raw = (int16_t)(raw[16] << 8 | raw[15]);
            int16_t my_raw = (int16_t)(raw[18] << 8 | raw[17]);
					  int16_t mz_raw = (int16_t)(raw[20] << 8 | raw[19]);
            
            
            mx_sum += (float)mx_raw - (-106.0f);
					  my_sum += ((float)my_raw - (72.0f));
					  mz_sum += (float)mz_raw - (74.0f);
        }
        HAL_Delay(1);
    }

    // 1. Tinh Offset cho Accel/Gyro
    Roll_Offset = r_sum / 1000.0f; 
    Pitch_Offset = p_sum / 1000.0f;
    Yaw_Gyro_Offset = y_gyro_sum / 1000.0f;
    GyroX_Offset = gx_sum / 1000.0f; 
    GyroY_Offset = gy_sum / 1000.0f;

    // 2. Tinh trung binh Mag 
    float mx_avg = mx_sum / 1000.0f;
    float my_avg = my_sum / 1000.0f;
		float mz_avg = mz_sum / 1000.0f;
		
		// === TÍNH OFFSET CHO GIA T?C ===
    AccelX_Offset = ax_sum / 1000.0f;
    AccelY_Offset = ay_sum / 1000.0f;
    // Tr?c Z n?m im ph?i b?ng 1G, nên ph?n du ra chính là Offset
    AccelZ_Offset = (az_sum / 1000.0f) - 1.0f;
		
    mx_f = mx_avg; 
    my_f = my_avg;
		MagZ_Offset = mz_avg;

    Mag_Yaw_Initial = atan2f(mx_avg, -my_avg) * 57.29578f;
    
		Yaw = 0.0f;
}

void ICM20948_Process_mag(I2C_HandleTypeDef *hi2c) {
    // 0x2D-0x32: Accel(6) | 0x33-0x38: Gyro(6) | 0x39-0x3A: Temp(2) | 0x3B-0x43: Mag(9)
    uint8_t raw_all[23];
    static float ax_f = 0, ay_f = 0, az_f = 1.0f, gx_f = 0, gy_f = 0, gz_lpf = 0;

    SelectBank(hi2c, USER_BANK_0);
    
    if(HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, 0x2D, 1, raw_all, 23, 10) != HAL_OK) return;

    // 1. Tinh dt
    uint32_t now = DWT->CYCCNT;
    float dt = (float)(now - last_tick) / (float)SystemCoreClock;
    last_tick = now;

    if (dt <= 0 || dt > 0.05f) dt = 0.002f; 

    // 2. Gan du lieu Magnetometer 
    mag_x = (int16_t)(raw_all[16] << 8 | raw_all[15]);
    mag_y = (int16_t)(raw_all[18] << 8 | raw_all[17]);
    mag_z = (int16_t)(raw_all[20] << 8 | raw_all[19]);

    // 3. Xu ly Accel/Gyro 
    float ax_raw = (int16_t)(raw_all[0] << 8 | raw_all[1]) / 2048.0f;
    float ay_raw = (int16_t)(raw_all[2] << 8 | raw_all[3]) / 2048.0f;
    float az_raw = (int16_t)(raw_all[4] << 8 | raw_all[5]) / 2048.0f;

    ax_f += 0.1f * (ax_raw - ax_f);
    ay_f += 0.1f * (ay_raw - ay_f);
    az_f += 0.1f * (az_raw - az_f);
		accel_roll = (ay_raw - AccelY_Offset) * 9.80665f;
		accel_pitch = (ax_raw - AccelX_Offset) * 9.80665f;
		accel_yaw = (az_raw - AccelZ_Offset) * 9.80665f;

    float gx_raw_corr = ((int16_t)(raw_all[6] << 8 | raw_all[7]) / 16.4f) - GyroX_Offset;
    float gy_raw_corr = ((int16_t)(raw_all[8] << 8 | raw_all[9]) / 16.4f) - GyroY_Offset;
    float gz_raw      = ((int16_t)(raw_all[10] << 8 | raw_all[11]) / 16.4f) - Yaw_Gyro_Offset;
		
		// === THÊM: XU?T RAW RA CHO UKF (Nhân 0.017453f d? d?i sang Rad/s) ===
    GyroPitch = gx_raw_corr * 0.017453f;
    GyroRoll  = gy_raw_corr * 0.017453f;
    GyroYaw   = gz_raw * 0.017453f;

    gx_f += 0.15f * (gx_raw_corr - gx_f);
    gy_f += 0.15f * (gy_raw_corr - gy_f);

    // 4. Complementary Filter 
    float r_acc = (atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 57.29578f) - Roll_Offset;
//    float p_acc = (atan2f(ay_f, az_f) * 57.29578f) - Pitch_Offset;
    float p_acc = atan2f(ay_f, sqrtf(ax_f*ax_f + az_f*az_f)) * 57.29578f - Pitch_Offset;

//    Roll  = 0.998f * (Roll + gy_f * dt) + 0.002f * r_acc;
//    Pitch = 0.998f * (Pitch + gx_f * dt) + 0.002f * p_acc;
float acc_mag = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
float acc_trust = (fabsf(acc_mag - 1.0f) < 0.15f) ? 0.002f : 0.0001f;

Roll  = (1.0f - acc_trust) * (Roll + gy_f * dt) + acc_trust * r_acc;
Pitch = (1.0f - acc_trust) * (Pitch + gx_f * dt) + acc_trust * p_acc;

    // Tich phan Yaw
    gz_lpf = (gz_lpf * 0.9f) + (gz_raw * 0.1f);
    float gz_final = (fabsf(gz_lpf) > 0.15f) ? gz_lpf : 0.0f;
    Yaw += gz_final * dt;
		
    // 1. Scale
    float mx_c = ((float)mag_x - (-106.0f)) * 1.0f;
    float my_c = ((float)mag_y - (72.0f)) * 1.0f;
    float mz_c = (float)mag_z - MagZ_Offset;

    // 2. Bo loc thong thap
    mx_f += alpha_mag * (mx_c - mx_f);
    my_f += alpha_mag * (my_c - my_f);

    // 3. --- Thuat toan bu nghieng ---
    float r_rad = Roll * 0.017453f;
    float p_rad = Pitch * 0.017453f;

    float mx_h = mx_f * cosf(p_rad) + mz_c * sinf(p_rad);
    float my_h = mx_f * sinf(r_rad) * sinf(p_rad) + my_f * cosf(r_rad) - mz_c * sinf(r_rad) * cosf(p_rad);

    //float Mag_Yaw_Current = atan2f(mx_h, -my_h) * 57.29578f;
		float current_raw_yaw = atan2f(mx_h, -my_h) * 57.29578f;

    //deadband
    static float last_stable_yaw = 0;
    if (fabsf(current_raw_yaw - last_stable_yaw) > 0.5f) { 
        last_stable_yaw = current_raw_yaw;
    }
    float Mag_Yaw_Current = last_stable_yaw;

    Mag_Delta = Mag_Yaw_Current - Mag_Yaw_Initial;
    if (Mag_Delta > 180.0f)  Mag_Delta -= 360.0f;
    if (Mag_Delta < -180.0f) Mag_Delta += 360.0f;

    float Yaw_Wrapped = fmodf(Yaw, 360.0f);
    if (Yaw_Wrapped > 180.0f)  Yaw_Wrapped -= 360.0f;
    if (Yaw_Wrapped < -180.0f) Yaw_Wrapped += 360.0f;

    Yaw_Drift = Yaw_Wrapped - Mag_Delta;
    if (Yaw_Drift > 180.0f)  Yaw_Drift -= 360.0f;
    if (Yaw_Drift < -180.0f) Yaw_Drift += 360.0f;
		
		Yaw -= Yaw_Drift * 0.002f;
		
    // Export Rad/s
//    GyroRoll = gy_f * 0.017453f; GyroPitch = gx_f * 0.017453f; GyroYaw = gz_final * 0.017453f;
}

