#ifndef UKF_FUSION_H
#define UKF_FUSION_H

#include "ICM20948_mag.h"
#include "arm_math.h"
#include <string.h>
#include <math.h>
#include "main.h"
// #include "main.h" // Nh? include main.h n?u Anchor_t du?c d?nh nghia ? dó

// ---------------------------------------------------------
// Ð?NH NGHIA KÍCH THU?C B? L?C
// ---------------------------------------------------------
#define STATE_DIM 16      // [px, py, pz, vx, vy, vz, qw, qx, qy, qz, bax, bay, baz, bgx, bgy, bgz]
#define MEAS_DIM 4        // [D0, D1, D2, bmp_z]
#define SIGMA_POINTS (2 * STATE_DIM + 1)

// Thông s? UT (Unscented Transform)
#define UKF_ALPHA 1.0f
#define UKF_KAPPA 0.0f
#define UKF_BETA  2.0f
#define GRAVITY   9.80665f

// ---------------------------------------------------------
// C?U TRÚC UKF
// ---------------------------------------------------------
typedef struct {
    float32_t state_data[STATE_DIM];                  
    float32_t P_data[STATE_DIM * STATE_DIM];          
    float32_t Q_data[STATE_DIM * STATE_DIM];          
    float32_t R_data[MEAS_DIM * MEAS_DIM];            
    float32_t X_sigmas_data[STATE_DIM * SIGMA_POINTS];

    float32_t Wm[SIGMA_POINTS];
    float32_t Wc[SIGMA_POINTS];
    float32_t lambda;

    arm_matrix_instance_f32 X;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 R;
} UKF_FilterTypeDef;

// ---------------------------------------------------------
// KHAI BÁO HÀM (Ðã b? tham s? accel/gyro do dùng extern)
// ---------------------------------------------------------
void UKF_Init(UKF_FilterTypeDef *ukf);
void UKF_Predict(UKF_FilterTypeDef *ukf, float dt);
void UKF_Update(UKF_FilterTypeDef *ukf, float d0, float d1, float d2, float bmp_z);

#endif // UKF_FUSION_H
