#include "ukf_fusion.h"

#define ATTR_DTCM __attribute__((section(".dtcmram")))

static float32_t imu_ax_snap = 0.0f;
static float32_t imu_ay_snap = 0.0f;
static float32_t imu_az_snap = 0.0f;
static float32_t imu_gx_snap = 0.0f;
static float32_t imu_gy_snap = 0.0f;
static float32_t imu_gz_snap = 0.0f;

static uint8_t custom_cholesky_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) {
    uint16_t n = pSrc->numRows;
    if (pSrc->numRows != pSrc->numCols || pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols) return 1;

    float32_t *pA = pSrc->pData;
    float32_t *pL = pDst->pData;
    memset(pL, 0, n * n * sizeof(float32_t));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            float32_t sum = 0.0f;
            for (int k = 0; k < j; k++) sum += pL[i * n + k] * pL[j * n + k];

            if (i == j) {
                float32_t val = pA[i * n + i] - sum;
                if (val <= 0.0f) return 1; 
                pL[i * n + j] = sqrtf(val);
            } else {
                pL[i * n + j] = (1.0f / pL[j * n + j]) * (pA[i * n + j] - sum);
            }
        }
    }
    return 0;
}


static void IMU_Kinematics(float32_t *state_in, float32_t *state_out, float dt) {
    memcpy(state_out, state_in, sizeof(float32_t) * STATE_DIM);

//    float px = state_in[0], py = state_in[1], pz = state_in[2];
//    float vx = state_in[3], vy = state_in[4], vz = state_in[5];
//    float qw = state_in[6], qx = state_in[7], qy = state_in[8], qz = state_in[9];
//    float bax = state_in[10], bay = state_in[11], baz = state_in[12];
//    float bgx = state_in[13], bgy = state_in[14], bgz = state_in[15];


//    float ax = accel_pitch - bax; 
//    float ay = accel_roll  - bay; 
//    float az = accel_yaw   - baz; 
//    
//    float gx = GyroPitch - bgx;
//    float gy = GyroRoll  - bgy;
//    float gz = GyroYaw   - bgz;
	////////////////////////////////////////////
   	float px = state_in[0], py = state_in[1], pz = state_in[2];
    float vx = state_in[3], vy = state_in[4], vz = state_in[5];
    float qw = state_in[6], qx = state_in[7], qy = state_in[8], qz = state_in[9];
    float bax = state_in[10], bay = state_in[11], baz = state_in[12];
    float bgx = state_in[13], bgy = state_in[14], bgz = state_in[15];

    // ?? ÐÃ S?A: Thay d?i các bi?n Global IMU thành các bi?n Snapshot dóng bang
    float ax = imu_ax_snap - bax;
    float ay = imu_ay_snap - bay;
    float az = imu_az_snap - baz;
    
    float gx = imu_gx_snap - bgx;
    float gy = imu_gy_snap - bgy;
    float gz = imu_gz_snap - bgz;

    // Ma tran quay (DCM) Body sang World Frame
    float R11 = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
    float R12 = 2.0f*qx*qy - 2.0f*qz*qw;
    float R13 = 2.0f*qx*qz + 2.0f*qy*qw;
    float R21 = 2.0f*qx*qy + 2.0f*qz*qw;
    float R22 = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;
    float R23 = 2.0f*qy*qz - 2.0f*qx*qw;
    float R31 = 2.0f*qx*qz - 2.0f*qy*qw;
    float R32 = 2.0f*qy*qz + 2.0f*qx*qw;
    float R33 = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

    // World Frame
    float aw_x = R11*ax + R12*ay + R13*az;
    float aw_y = R21*ax + R22*ay + R23*az;
    float aw_z = (R31*ax + R32*ay + R33*az) - GRAVITY;

    // Vi tri va van toc
    state_out[0] = px + vx*dt + 0.5f*aw_x*dt*dt;
    state_out[1] = py + vy*dt + 0.5f*aw_y*dt*dt;
    state_out[2] = pz + vz*dt + 0.5f*aw_z*dt*dt;

    state_out[3] = vx + aw_x*dt;
    state_out[4] = vy + aw_y*dt;
    state_out[5] = vz + aw_z*dt;

    // Quaternion
    state_out[6] = qw + 0.5f * (-qx*gx - qy*gy - qz*gz) * dt;
    state_out[7] = qx + 0.5f * ( qw*gx + qy*gz - qz*gy) * dt;
    state_out[8] = qy + 0.5f * ( qw*gy - qx*gz + qz*gx) * dt;
    state_out[9] = qz + 0.5f * ( qw*gz + qx*gy - qy*gx) * dt;
}


static void Observation_Model(float32_t *state, float32_t *z_pred) {
    float32_t px = state[0], py = state[1], pz = state[2];
    float32_t dx, dy, dz;

    // Tinh cho BS0
    dx = px - anchors[0].x; dy = py - anchors[0].y; dz = pz - anchors[0].z;
    z_pred[0] = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // Tinh cho BS1
    dx = px - anchors[1].x; dy = py - anchors[1].y; dz = pz - anchors[1].z;
    z_pred[1] = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // Tinh cho BS2
    dx = px - anchors[2].x; dy = py - anchors[2].y; dz = pz - anchors[2].z;
    z_pred[2] = sqrtf(dx*dx + dy*dy + dz*dz);
    
    z_pred[3] = pz; 
}

// ----------------------------------------------------------------------------
// KH?I T?O B? L?C
// ----------------------------------------------------------------------------
//void UKF_Init(UKF_FilterTypeDef *ukf) {
//    arm_mat_init_f32(&(ukf->X), STATE_DIM, 1, ukf->state_data);
//    arm_mat_init_f32(&(ukf->P), STATE_DIM, STATE_DIM, ukf->P_data);
//    arm_mat_init_f32(&(ukf->Q), STATE_DIM, STATE_DIM, ukf->Q_data);
//    arm_mat_init_f32(&(ukf->R), MEAS_DIM, MEAS_DIM, ukf->R_data);

//    memset(ukf->state_data, 0, sizeof(ukf->state_data));
//    memset(ukf->P_data, 0, sizeof(ukf->P_data));
//    
//    ukf->state_data[6] = 1.0f; // qw = 1

//    ukf->lambda = UKF_ALPHA * UKF_ALPHA * (STATE_DIM + UKF_KAPPA) - STATE_DIM;
//    float c = STATE_DIM + ukf->lambda;
//    
//    ukf->Wm[0] = ukf->lambda / c;
//    ukf->Wc[0] = (ukf->lambda / c) + (1.0f - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
//    for (int i = 1; i < SIGMA_POINTS; i++) {
//        ukf->Wm[i] = 0.5f / c;
//        ukf->Wc[i] = 0.5f / c;
//    }

//    for(int i=0; i<STATE_DIM; i++) ukf->P_data[i*STATE_DIM + i] = 0.1f;
//    for(int i=0; i<STATE_DIM; i++) ukf->Q_data[i*STATE_DIM + i] = 0.001f;
//    
////    ukf->R_data[0*MEAS_DIM + 0] = 0.02f; 
////    ukf->R_data[1*MEAS_DIM + 1] = 0.02f; 
////    ukf->R_data[2*MEAS_DIM + 2] = 0.02f; 
////    ukf->R_data[3*MEAS_DIM + 3] = 0.05f; 
//		
//		ukf->R_data[0*MEAS_DIM + 0] = 0.05f; 
//    ukf->R_data[1*MEAS_DIM + 1] = 0.05f; 
//    ukf->R_data[2*MEAS_DIM + 2] = 0.05f; 
//    ukf->R_data[3*MEAS_DIM + 3] = 0.002f; 
//}
////////////////////////////////////////////////
//kha on
//void UKF_Init(UKF_FilterTypeDef *ukf) {
//    arm_mat_init_f32(&(ukf->X), STATE_DIM, 1, ukf->state_data);
//    arm_mat_init_f32(&(ukf->P), STATE_DIM, STATE_DIM, ukf->P_data);
//    arm_mat_init_f32(&(ukf->Q), STATE_DIM, STATE_DIM, ukf->Q_data);
//    arm_mat_init_f32(&(ukf->R), MEAS_DIM, MEAS_DIM, ukf->R_data);

//    memset(ukf->state_data, 0, sizeof(ukf->state_data));
//    memset(ukf->P_data, 0, sizeof(ukf->P_data));
//    
//    ukf->state_data[6] = 1.0f; // qw = 1

//    ukf->lambda = UKF_ALPHA * UKF_ALPHA * (STATE_DIM + UKF_KAPPA) - STATE_DIM;
//    float c = STATE_DIM + ukf->lambda;
//    
//    ukf->Wm[0] = ukf->lambda / c;
//    ukf->Wc[0] = (ukf->lambda / c) + (1.0f - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
//    for (int i = 1; i < SIGMA_POINTS; i++) {
//        ukf->Wm[i] = 0.5f / c;
//        ukf->Wc[i] = 0.5f / c;
//    }

//    // === ÐÃ S?A: PHÂN TÁCH MA TR?N P VÀ Q ===
//    // 1. T?a d? (0-2) và V?n t?c (3-5): Cho phép UWB c?p nh?t bình thu?ng
//    for(int i = 0; i < 6; i++) {
//        ukf->P_data[i*STATE_DIM + i] = 0.1f;
//        ukf->Q_data[i*STATE_DIM + i] = 0.001f;
//    }

//    // 2. KHÓA C?NG Quaternion (6-9) và IMU Bias (10-15)
//    for(int i = 6; i < 16; i++) {
//        ukf->P_data[i*STATE_DIM + i] = 1e-5f;  // Ép nhi?u c?c nh?
//        ukf->Q_data[i*STATE_DIM + i] = 1e-6f;  // Ép nhi?u c?c nh?
//    }
//    // ========================================
//    
//    ukf->R_data[0*MEAS_DIM + 0] = 0.05f; 
//    ukf->R_data[1*MEAS_DIM + 1] = 0.05f; 
//    ukf->R_data[2*MEAS_DIM + 2] = 0.05f; 
//    ukf->R_data[3*MEAS_DIM + 3] = 0.002f; 
//}
///////////////////////////
//on hon nua roi
void UKF_Init(UKF_FilterTypeDef *ukf) {
    arm_mat_init_f32(&(ukf->X), STATE_DIM, 1, ukf->state_data);
    arm_mat_init_f32(&(ukf->P), STATE_DIM, STATE_DIM, ukf->P_data);
    arm_mat_init_f32(&(ukf->Q), STATE_DIM, STATE_DIM, ukf->Q_data);
    arm_mat_init_f32(&(ukf->R), MEAS_DIM, MEAS_DIM, ukf->R_data);

    memset(ukf->state_data, 0, sizeof(ukf->state_data));
    memset(ukf->P_data, 0, sizeof(ukf->P_data));
    
    ukf->state_data[6] = 1.0f; // qw = 1

    ukf->lambda = UKF_ALPHA * UKF_ALPHA * (STATE_DIM + UKF_KAPPA) - STATE_DIM;
    float c = STATE_DIM + ukf->lambda;
    
    ukf->Wm[0] = ukf->lambda / c;
    ukf->Wc[0] = (ukf->lambda / c) + (1.0f - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
    for (int i = 1; i < SIGMA_POINTS; i++) {
        ukf->Wm[i] = 0.5f / c;
        ukf->Wc[i] = 0.5f / c;
    }

    for(int i = 0; i < 6; i++) {
        ukf->P_data[i*STATE_DIM + i] = 0.1f;
        ukf->Q_data[i*STATE_DIM + i] = 0.001f;
    }

    for(int i = 6; i < 16; i++) {
        ukf->P_data[i*STATE_DIM + i] = 1e-5f;  
        ukf->Q_data[i*STATE_DIM + i] = 1e-6f;  
    }
    
    ukf->R_data[0*MEAS_DIM + 0] = 0.3f; 
    ukf->R_data[1*MEAS_DIM + 1] = 0.3f; 
    ukf->R_data[2*MEAS_DIM + 2] = 0.3f; 
    ukf->R_data[3*MEAS_DIM + 3] = 0.002f; 
}

void UKF_Predict(UKF_FilterTypeDef *ukf, float dt) {
	  imu_ax_snap = accel_pitch;
    imu_ay_snap = accel_roll;
    imu_az_snap = accel_yaw;
    imu_gx_snap = GyroPitch;
    imu_gy_snap = GyroRoll;
    imu_gz_snap = GyroYaw;
	  
    ATTR_DTCM static float32_t P_chol_data[STATE_DIM * STATE_DIM];
    arm_matrix_instance_f32 P_chol;
    arm_mat_init_f32(&P_chol, STATE_DIM, STATE_DIM, P_chol_data);
    
    if(custom_cholesky_f32(&(ukf->P), &P_chol) != 0) {
        for (int i = 0; i < STATE_DIM; i++) {
            for (int j = 0; j < STATE_DIM; j++) {
                if (i == j) {
                    ukf->P_data[i * STATE_DIM + j] = 0.1f;
                } else {
                    ukf->P_data[i * STATE_DIM + j] = 0.0f;
                }
            }
        }
        if(custom_cholesky_f32(&(ukf->P), &P_chol) != 0) return;
    }

    float gamma = sqrtf(STATE_DIM + ukf->lambda);
    for(int i = 0; i < STATE_DIM * STATE_DIM; i++) P_chol_data[i] *= gamma;

    ATTR_DTCM static float32_t X_sigmas_old[STATE_DIM * SIGMA_POINTS];
    for (int j = 0; j < STATE_DIM; j++) X_sigmas_old[j * SIGMA_POINTS + 0] = ukf->state_data[j];
    
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            X_sigmas_old[j * SIGMA_POINTS + (i + 1)] = ukf->state_data[j] + P_chol_data[j * STATE_DIM + i];
            X_sigmas_old[j * SIGMA_POINTS + (i + 1 + STATE_DIM)] = ukf->state_data[j] - P_chol_data[j * STATE_DIM + i];
        }
    }

    for (int i = 0; i < SIGMA_POINTS; i++) {
        float32_t state_in[STATE_DIM], state_out[STATE_DIM];
        for(int j = 0; j < STATE_DIM; j++) state_in[j] = X_sigmas_old[j * SIGMA_POINTS + i];
        
        IMU_Kinematics(state_in, state_out, dt);
        
        for(int j = 0; j < STATE_DIM; j++) ukf->X_sigmas_data[j * SIGMA_POINTS + i] = state_out[j];
    }

    memset(ukf->state_data, 0, sizeof(ukf->state_data));
    for (int i = 0; i < SIGMA_POINTS; i++) {
        for (int j = 0; j < STATE_DIM; j++) ukf->state_data[j] += ukf->Wm[i] * ukf->X_sigmas_data[j * SIGMA_POINTS + i];
    }

    float qw = ukf->state_data[6], qx = ukf->state_data[7], qy = ukf->state_data[8], qz = ukf->state_data[9];
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0f) {
        ukf->state_data[6] /= norm; ukf->state_data[7] /= norm; ukf->state_data[8] /= norm; ukf->state_data[9] /= norm;
    }

    memset(ukf->P_data, 0, sizeof(ukf->P_data));
    for (int i = 0; i < SIGMA_POINTS; i++) {
        float32_t x_diff[STATE_DIM];
        for (int j = 0; j < STATE_DIM; j++) x_diff[j] = ukf->X_sigmas_data[j * SIGMA_POINTS + i] - ukf->state_data[j];
        for (int row = 0; row < STATE_DIM; row++) {
            for (int col = 0; col < STATE_DIM; col++) {
                ukf->P_data[row * STATE_DIM + col] += ukf->Wc[i] * x_diff[row] * x_diff[col];
            }
        }
    }
    arm_mat_add_f32(&(ukf->P), &(ukf->Q), &(ukf->P));
    
//    drone_x = ukf->state_data[0];
//    drone_y = ukf->state_data[1];
//    drone_z_ukf = ukf->state_data[2];
//    vx = ukf->state_data[3];
//    vy = ukf->state_data[4];
//    vz_ukf = ukf->state_data[5];
}


void UKF_Update(UKF_FilterTypeDef *ukf, float d0, float d1, float d2, float bmp_z) {
    float32_t z_meas_data[MEAS_DIM] = {d0, d1, d2, bmp_z};
    ATTR_DTCM static float32_t Z_sigmas_data[MEAS_DIM * SIGMA_POINTS] = {0};
    ATTR_DTCM static float32_t Z_pred_data[MEAS_DIM] = {0};
    ATTR_DTCM static float32_t S_data[MEAS_DIM * MEAS_DIM] = {0};
    ATTR_DTCM static float32_t Pxz_data[STATE_DIM * MEAS_DIM] = {0};
    ATTR_DTCM static float32_t K_data[STATE_DIM * MEAS_DIM] = {0};
    ATTR_DTCM static float32_t S_inv_data[MEAS_DIM * MEAS_DIM] = {0};
        
    // Reset bien static
    memset(Z_sigmas_data, 0, sizeof(Z_sigmas_data));
    memset(Z_pred_data, 0, sizeof(Z_pred_data));
    memset(S_data, 0, sizeof(S_data));
    memset(Pxz_data, 0, sizeof(Pxz_data));
    memset(K_data, 0, sizeof(K_data));
    
    arm_matrix_instance_f32 Z_meas, Z_pred, S, Pxz, K, S_inv;
    arm_mat_init_f32(&Z_meas, MEAS_DIM, 1, z_meas_data);
    arm_mat_init_f32(&Z_pred, MEAS_DIM, 1, Z_pred_data);
    arm_mat_init_f32(&S, MEAS_DIM, MEAS_DIM, S_data);
    arm_mat_init_f32(&Pxz, STATE_DIM, MEAS_DIM, Pxz_data);
    arm_mat_init_f32(&K, STATE_DIM, MEAS_DIM, K_data);
    arm_mat_init_f32(&S_inv, MEAS_DIM, MEAS_DIM, S_inv_data);

    for (int i = 0; i < SIGMA_POINTS; i++) {
        float32_t current_x_sigma[STATE_DIM], current_z_sigma[MEAS_DIM];
        for(int j = 0; j < STATE_DIM; j++) current_x_sigma[j] = ukf->X_sigmas_data[j * SIGMA_POINTS + i];
        Observation_Model(current_x_sigma, current_z_sigma);
        for(int j = 0; j < MEAS_DIM; j++) Z_sigmas_data[j * SIGMA_POINTS + i] = current_z_sigma[j];
    }

    for (int i = 0; i < SIGMA_POINTS; i++) {
        for (int j = 0; j < MEAS_DIM; j++) Z_pred_data[j] += ukf->Wm[i] * Z_sigmas_data[j * SIGMA_POINTS + i];
    }

    memcpy(S_data, ukf->R_data, sizeof(float32_t) * MEAS_DIM * MEAS_DIM);
    for (int i = 0; i < SIGMA_POINTS; i++) {
        float32_t z_diff[MEAS_DIM], x_diff[STATE_DIM];
        for (int j = 0; j < MEAS_DIM; j++) z_diff[j] = Z_sigmas_data[j * SIGMA_POINTS + i] - Z_pred_data[j];
        for (int j = 0; j < STATE_DIM; j++) x_diff[j] = ukf->X_sigmas_data[j * SIGMA_POINTS + i] - ukf->state_data[j];
        for(int row = 0; row < MEAS_DIM; row++) {
            for(int col = 0; col < MEAS_DIM; col++) S_data[row * MEAS_DIM + col] += ukf->Wc[i] * z_diff[row] * z_diff[col];
        }
        for(int row = 0; row < STATE_DIM; row++) {
            for(int col = 0; col < MEAS_DIM; col++) Pxz_data[row * MEAS_DIM + col] += ukf->Wc[i] * x_diff[row] * z_diff[col];
        }
    }

    if (arm_mat_inverse_f32(&S, &S_inv) != ARM_MATH_SUCCESS) return; 
    arm_mat_mult_f32(&Pxz, &S_inv, &K);

    ATTR_DTCM static float32_t z_diff_data[MEAS_DIM], K_times_zdiff_data[STATE_DIM];
    arm_matrix_instance_f32 Z_diff_mat, K_times_zdiff;
    arm_mat_init_f32(&Z_diff_mat, MEAS_DIM, 1, z_diff_data);
    arm_mat_init_f32(&K_times_zdiff, STATE_DIM, 1, K_times_zdiff_data);

    arm_mat_sub_f32(&Z_meas, &Z_pred, &Z_diff_mat);
    arm_mat_mult_f32(&K, &Z_diff_mat, &K_times_zdiff);
    arm_mat_add_f32(&(ukf->X), &K_times_zdiff, &(ukf->X)); 

    float qw = ukf->state_data[6], qx = ukf->state_data[7], qy = ukf->state_data[8], qz = ukf->state_data[9];
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0f) {
        ukf->state_data[6] /= norm; ukf->state_data[7] /= norm; ukf->state_data[8] /= norm; ukf->state_data[9] /= norm;
    }

    ATTR_DTCM static float32_t K_times_S_data[STATE_DIM * MEAS_DIM], K_T_data[MEAS_DIM * STATE_DIM], K_S_KT_data[STATE_DIM * STATE_DIM];
    arm_matrix_instance_f32 K_times_S, K_T, K_S_KT;
    arm_mat_init_f32(&K_times_S, STATE_DIM, MEAS_DIM, K_times_S_data);
    arm_mat_init_f32(&K_T, MEAS_DIM, STATE_DIM, K_T_data);
    arm_mat_init_f32(&K_S_KT, STATE_DIM, STATE_DIM, K_S_KT_data);

    arm_mat_mult_f32(&K, &S, &K_times_S);
    arm_mat_trans_f32(&K, &K_T);
    arm_mat_mult_f32(&K_times_S, &K_T, &K_S_KT);
    arm_mat_sub_f32(&(ukf->P), &K_S_KT, &(ukf->P));
    
//    drone_x = ukf->state_data[0];
//    drone_y = ukf->state_data[1];
//    drone_z_ukf = ukf->state_data[2];
//    vx = ukf->state_data[3];
//    vy = ukf->state_data[4];
//    vz_ukf = ukf->state_data[5];
		
}
