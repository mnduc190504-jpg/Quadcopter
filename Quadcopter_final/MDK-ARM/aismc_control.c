#include "aismc_control.h"
#include <math.h>

// --- 1. Constant ---  
#define GRAVITY   9.81f
#define ARM_L     0.23f     // Chieu dai canh tay don (m) 
//#define IX        0.1f    // Mo-men quan tinh truc X (kg.m^2) 
//#define IY        0.1f    // Mo-men quán tính truc Y (kg.m^2)
//#define IZ        0.15f    // Mo-men quán tính truc Z (kg.m^2)
#define IX        0.08f    // Mo-men quan tinh truc X (kg.m^2) 
#define IY        0.08f    // Mo-men quán tính truc Y (kg.m^2)
#define IZ        0.12f    // Mo-men quán tính truc Z (kg.m^2)
static const float b_coeff = 313e-6f;   
//static const float d_coeff = 1.5e-5f;
static const float d_coeff = 4.0e-5f;

// --- 2. He so dk ---

//static const float K1[3] = {6.5f, 6.5f, 1.0f};   
//static const float K2[3] = {1.0f, 1.0f, 0.2f};    
//static const float C1[3] = {1.2f, 1.2f, 0.8f};      
//static const float C2[3] = {1.2f, 1.2f, 0.8f};      
//static const float G_ADAPT[3] = {3.0f, 3.0f, 1.5f};
//static const float G_MAX = 20.0f;
//static const float G_MAX_yaw = 10.0f;
//12052026
//static const float K1[3] = {5.5f, 5.5f, 3.5f};   
//static const float K2[3] = {1.0f, 1.0f, 1.0f};    
//static const float C1[3] = {0.85f, 0.85f, 1.5f};      
//static const float C2[3] = {0.85f, 0.85f, 1.5f};      
//static const float G_ADAPT[3] = {3.0f, 3.0f, 3.0f};
//static const float G_MAX = 22.0f;
//static const float G_MAX_yaw = 18.0f;
//bam tot voi vong ngoai
//static const float K1[3] = {6.5f, 6.5f, 3.5f};   
//static const float K2[3] = {1.1f, 1.1f, 1.0f};    
//static const float C1[3] = {0.55f, 0.55f, 1.5f};      
//static const float C2[3] = {1.05f, 1.05f, 1.5f};      
//static const float G_ADAPT[3] = {3.0f, 3.0f, 3.0f};
//static const float G_MAX = 22.0f;
//static const float G_MAX_yaw = 18.0f;
/////////////////////////
static const float K1[3] = {5.5f, 5.5f, 3.5f};   
static const float K2[3] = {1.0f, 1.0f, 1.0f};    
static const float C1[3] = {0.85f, 0.85f, 1.5f};      
static const float C2[3] = {0.85f, 0.85f, 1.5f};      
static const float G_ADAPT[3] = {6.5f, 6.5f, 3.0f};
static const float G_MAX = 12.0f;
static const float G_MAX_yaw = 18.0f;

//static const float ARM_FACTOR = 1.537f; 

float clamp(float v, float min, float max) {
    if(v < min) return min;
    if(v > max) return max;
    return v;
}

void AISMC_Init(AISMC_HandleTypeDef *hadapt, float sample_time) {
    hadapt->dt = sample_time;
    for(int i=0; i<3; i++) {
        hadapt->int_e[i] = 0; 
        hadapt->Gamma_hat[i] = 0; 
    }
}

AISMC_Output AISMC_Update(AISMC_HandleTypeDef *hadapt, float cur_ang[3], float des_ang[3], float cur_gyro[3], float current_throttle) {
    AISMC_Output out;
    float dt = hadapt->dt;

    if (current_throttle <= 0.0f) {
        for(int i=0; i<3; i++) {
            hadapt->int_e[i] = 0.0f;     
            hadapt->Gamma_hat[i] = 0.0f; 
        }
        out.u2 = 0.0f; out.u3 = 0.0f; out.u4 = 0.0f;
        return out; 
    }

    float e[3], s[3], U[3];
    for(int i=0; i<3; i++) {
        e[i] = cur_ang[i] - des_ang[i]; 

        // Anti-windup 
        hadapt->int_e[i] += e[i] * dt;
        hadapt->int_e[i] = clamp(hadapt->int_e[i], -5.0f, 5.0f); 
        
        float de = cur_gyro[i]; 
        
        // Mat truot s
        s[i] = de + K1[i] * e[i] + K2[i] * hadapt->int_e[i]; 
        
//        float sigma = 0.05f; // Leakage factor
			  float sigma = 0.03f;
        hadapt->Gamma_hat[i] += (G_ADAPT[i] * s[i] - sigma * hadapt->Gamma_hat[i]) * dt;
        
        float current_G_MAX = (i == 2) ? G_MAX_yaw : G_MAX;
        hadapt->Gamma_hat[i] = clamp(hadapt->Gamma_hat[i], -current_G_MAX, current_G_MAX);
        
        U[i] = -hadapt->Gamma_hat[i] - C1[i] * tanhf(s[i]/1.0f) - C2[i] * s[i];
    }

    // u2, u3, u4...
    float a1 = (IY - IZ) / IX, a3 = (IZ - IX) / IY, a5 = (IX - IY) / IZ;
    float p = cur_gyro[0], q = cur_gyro[1], r = cur_gyro[2];

    out.u2 = IX * (-a1 * q * 0 - K1[0] * p - K2[0] * e[0] + U[0]); // Roll
    out.u3 = IY * (-a3 * p * 0 - K1[1] * q - K2[1] * e[1] + U[1]); // Pitch
    out.u4 = IZ * (-a5 * p * q - K1[2] * r - K2[2] * e[2] + U[2]); // Yaw
    
    return out;
}

void Actuator_Write(float u1, AISMC_Output ctrl) {
    float F[4];  
	  float f_t = 1.0f / (4.0f * b_coeff);
    // factor_roll_pitch: 1 / (2*sqrt(2)*b*l)
    float f_rp = 1.0f / (2.8284f * b_coeff * ARM_L);
    // factor_yaw: 1 / 4d
    float f_y = 1.0f / (4.0f * d_coeff);
    
    // --- 8. MOTOR MIXING ---
    // FR (1), RL (2), FL (3), RR (4)
    F[0] = f_t*u1 - f_rp*ctrl.u2 + f_rp*ctrl.u3 - f_y*ctrl.u4*1.0f; // Front-Right
    F[1] = f_t*u1 + f_rp*ctrl.u2 - f_rp*ctrl.u3 - f_y*ctrl.u4*1.0f; // Rear-Left
    F[2] = f_t*u1 + f_rp*ctrl.u2 + f_rp*ctrl.u3 + f_y*ctrl.u4*1.0f; // Front-Left
    F[3] = f_t*u1 - f_rp*ctrl.u2 - f_rp*ctrl.u3 + f_y*ctrl.u4*1.0f; // Rear-Right

    for(int i=0; i<4; i++) {
        if(F[i]<0) F[i] = 0;
        
        // --- 9. MAPPING sang PWM
        uint16_t pwm = (uint16_t)(1100 + 5.3f * sqrtf(F[i]));
        
        pwm = (uint16_t)clamp(pwm, 1100, 1950);
        log_data.m[i] = pwm;
        
        if(i==0) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
        else if(i==1) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
        else if(i==2) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);
        else if(i==3) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);
    }
}

void Actuator_Stop(void) {
    for(int i=1; i<=4; i++) __HAL_TIM_SET_COMPARE(&htim2, (i==1?TIM_CHANNEL_1:(i==2?TIM_CHANNEL_2:(i==3?TIM_CHANNEL_3:TIM_CHANNEL_4))), 1000);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
