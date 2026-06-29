#ifndef AISMC_CONTROL_H
#define AISMC_CONTROL_H

#include "main.h"

typedef struct {
    float int_e[3];      // Tich phan sai so
    float Gamma_hat[3];  // Bien thich nghi
    float prev_ang[3];   // Goc vong truoc
    float dt;            
} AISMC_HandleTypeDef;


typedef struct {
    float u2, u3, u4;
} AISMC_Output;

 
extern TIM_HandleTypeDef htim2;


void AISMC_Init(AISMC_HandleTypeDef *hadapt, float sample_time);
AISMC_Output AISMC_Update(AISMC_HandleTypeDef *hadapt, float cur_ang[3], float des_ang[3], float cur_gyro[3], float current_throttle);
void Actuator_Write(float u1, AISMC_Output ctrl);
void Actuator_Stop(void);


float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float clamp(float v, float min, float max);

#endif
