#ifndef AISMC_OUTER_H
#define AISMC_OUTER_H

#include "main.h"

typedef struct {
    float int_e[3];      // Tich phan sai so [x, y, z]
    float Gamma_hat[3];  // Bien thich nghi [x, y, z]
    float prev_pos[3];   // Vi tri lan trc tinh van toc
    float dt; 
    float filtered_vel[3];	
} AISMC_Outer_HandleTypeDef;

typedef struct {
    float u1;      
    float phi_d;   
    float theta_d; 
} AISMC_Outer_Output;

void AISMC_Outer_Init(AISMC_Outer_HandleTypeDef *houter, float sample_time);
AISMC_Outer_Output AISMC_Outer_Update(AISMC_Outer_HandleTypeDef *houter, float cur_pos[3], float cur_v[3], float des_pos[3], float cur_yaw);
void AISMC_Outer_Reset(AISMC_Outer_HandleTypeDef *houter, float cur_pos[3]);
#endif
