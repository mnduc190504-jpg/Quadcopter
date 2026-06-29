#include "aismc_outer.h"
#include <math.h>

#define M_DRONE     1.55f    //
#define G_CONST     9.81f   //
#define SCALE_XY    0.6f    // 

//static const float K1_OUT[3] = {1.2f, 1.2f, 2.0f};  //
//static const float K2_OUT[3] = {0.2f, 0.2f, 0.4f};  //
//static const float C1_OUT[3] = {1.0f, 1.0f, 1.2f};  //
//static const float C2_OUT[3] = {0.6f, 0.6f, 0.8f};  //
//static const float ALPHA_OUT[3] = {0.5f, 0.5f, 1.0f}; // 
//static const float GAMMA_OUT[3] = {12.0f, 12.0f, 8.0f}; //
//static const float G_MAX_OUT[3] = {10.0f, 10.0f, 20.0f}; //
//lan dang on
//static const float K1_OUT[3] = {1.4f, 1.4f, 1.5f};  //
//static const float K2_OUT[3] = {0.4f, 0.4f, 0.5f};  //
//static const float C1_OUT[3] = {0.8f, 0.8f, 0.8f};  //
//static const float C2_OUT[3] = {0.4f, 0.4f, 0.5f};  //
//static const float ALPHA_OUT[3] = {0.5f, 0.5f, 1.5f}; // 
//static const float GAMMA_OUT[3] = {12.0f, 12.0f, 10.0f}; //
//static const float G_MAX_OUT[3] = {16.0f, 16.0f, 6.0f}; 
/////////////////////////////////
//3g10 08052026
//static const float K1_OUT[3] = {2.0f, 2.0f, 1.5f};  //
//static const float K2_OUT[3] = {0.6f, 0.6f, 0.5f};  //
//static const float C1_OUT[3] = {0.8f, 0.8f, 0.8f};  //
//static const float C2_OUT[3] = {0.4f, 0.4f, 0.5f};  //
//static const float ALPHA_OUT[3] = {0.5f, 0.5f, 1.5f}; // 
//static const float GAMMA_OUT[3] = {12.0f, 12.0f, 10.0f}; //
//static const float G_MAX_OUT[3] = {16.0f, 16.0f, 6.0f}; 
/////////////////////////////////
//3g30 08052026
//static const float K1_OUT[3] = {1.2f, 1.2f, 1.5f};  //
//static const float K2_OUT[3] = {0.6f, 0.6f, 0.5f};  //
//static const float C1_OUT[3] = {0.4f, 0.4f, 0.8f};  //
//static const float C2_OUT[3] = {0.4f, 0.4f, 0.5f};  //
//static const float ALPHA_OUT[3] = {0.5f, 0.5f, 1.5f}; // 
//static const float GAMMA_OUT[3] = {12.0f, 12.0f, 10.0f}; //
//static const float G_MAX_OUT[3] = {16.0f, 16.0f, 6.0f}; 
/////////////////////////////////
static const float K1_OUT[3] = {1.0f, 1.0f, 1.5f};  //
static const float K2_OUT[3] = {0.3f, 0.3f, 0.5f};  //
static const float C1_OUT[3] = {0.2f, 0.2f, 0.8f};  //
static const float C2_OUT[3] = {0.4f, 0.4f, 0.5f};  //
static const float ALPHA_OUT[3] = {0.5f, 0.5f, 1.5f}; // 
static const float GAMMA_OUT[3] = {11.0f, 11.0f, 10.0f}; //
static const float G_MAX_OUT[3] = {8.0f, 8.0f, 6.0f}; 

void AISMC_Outer_Init(AISMC_Outer_HandleTypeDef *houter, float sample_time) {
    houter->dt = sample_time;
    for(int i=0; i<3; i++) {
        houter->int_e[i] = 0.0f;
        houter->Gamma_hat[i] = 0.0f;
        houter->prev_pos[i] = 0.0f;
			  houter->filtered_vel[i] = 0.0f;
    }
}

AISMC_Outer_Output AISMC_Outer_Update(AISMC_Outer_HandleTypeDef *houter, float cur_pos[3], float cur_v[3], float des_pos[3], float cur_yaw) {
    AISMC_Outer_Output out;
    float e[3], de[3], s[3], virtual_u[3];
    float dt = houter->dt;

//    for(int i=0; i<3; i++) {
//        // 1. Tinh van toc va vi tri
//        float vel = (cur_pos[i] - houter->prev_pos[i]) / dt;
//        houter->prev_pos[i] = cur_pos[i];

//        // 2. Sai so va tich phan
//        e[i] = cur_pos[i] - des_pos[i];
//        houter->int_e[i] += e[i] * dt;
//        de[i] = vel - 0.0f; 

//        // 3. Sliding Surface
//        s[i] = de[i] + K1_OUT[i] * e[i] + K2_OUT[i] * houter->int_e[i];

//        // 4. Luat thich nghi
//        houter->Gamma_hat[i] += ALPHA_OUT[i] * (1.0f / fmaxf(GAMMA_OUT[i], 1e-6f)) * s[i] * dt;
//        houter->Gamma_hat[i] = fmaxf(fminf(houter->Gamma_hat[i], G_MAX_OUT[i]), -G_MAX_OUT[i]);

//        // 5. Tinh virtual control Ux, Uy, Uz
//        float U_nonlin = -houter->Gamma_hat[i] - C1_OUT[i] * tanhf(s[i]) - C2_OUT[i] * s[i];
//        virtual_u[i] = 0.0f - K1_OUT[i] * de[i] - K2_OUT[i] * e[i] + U_nonlin; // ddxd = 0
//    }
////////////////////////////////////////
    for(int i=0; i<3; i++) {
        // 1. Tinh van toc va vi tri
//        float raw_vel = (cur_pos[i] - houter->prev_pos[i]) / dt;
//        houter->prev_pos[i] = cur_pos[i];
//        
//        houter->filtered_vel[i] = (houter->filtered_vel[i] * 0.7f) + (raw_vel * 0.3f); 
//        float vel = houter->filtered_vel[i];
			////////////////////////
//        float vel = (cur_pos[i] - houter->prev_pos[i]) / dt;
//        houter->prev_pos[i] = cur_pos[i];
//			if(i == 0) vx = vel;
//			else if(i == 1) vy = vel;
//			else if(i == 2) vz = vel; 
			//////////////////////
			float vel;
			if(i== 0 || i == 1)
			{
			  vel = cur_v[i];
        houter->prev_pos[i] = cur_pos[i];
			}
			else
			{
	      vel = (cur_pos[i] - houter->prev_pos[i]) / dt;
        houter->prev_pos[i] = cur_pos[i];
				vz = vel;
			}
//////////////////////////////

        // 2. Sai so va tich phan 
        e[i] = cur_pos[i] - des_pos[i];
        
        houter->int_e[i] += e[i] * dt;
        houter->int_e[i] = fmaxf(fminf(houter->int_e[i], 8.0f), -8.0f); 

        de[i] = vel - 0.0f; 

        // 3. Sliding Surface
        s[i] = de[i] + K1_OUT[i] * e[i] + K2_OUT[i] * houter->int_e[i];

        // 4. Luat thich nghi 
        float sigma_out = 0.05f; // Leakage factor 
        
        
        float adapt_rate = ALPHA_OUT[i] * (1.0f / fmaxf(GAMMA_OUT[i], 1e-6f)); 
        
        
        houter->Gamma_hat[i] += (adapt_rate * s[i] - sigma_out * houter->Gamma_hat[i]) * dt;
        
        // Gioi han
        houter->Gamma_hat[i] = fmaxf(fminf(houter->Gamma_hat[i], G_MAX_OUT[i]), -G_MAX_OUT[i]);

        // 5. Tinh virtual control Ux, Uy, Uz
        //float U_nonlin = -houter->Gamma_hat[i] - C1_OUT[i] * tanhf(s[i]) - C2_OUT[i] * s[i];
				float U_nonlin = -houter->Gamma_hat[i] - C1_OUT[i] * tanhf(s[i] / 2.0f) - C2_OUT[i] * s[i];
        virtual_u[i] = 0.0f - K1_OUT[i] * de[i] - K2_OUT[i] * e[i] + U_nonlin; // ddxd = 0
				
//				// ADD
//        if (i < 2 && fabs(e[i]) <= 0.1f) {
//            //virtual_u[i] = 0.0f;     
//            houter->int_e[i] = houter->int_e[i]; 
//        }
    }

    // --- CONVERTER BLOCK ---
    // Gioi han virtual control
    virtual_u[0] = fmaxf(fminf(virtual_u[0], 3.0f * G_CONST), -3.0f * G_CONST);
    virtual_u[1] = fmaxf(fminf(virtual_u[1], 3.0f * G_CONST), -3.0f * G_CONST);
   // virtual_u[2] = fmaxf(fminf(virtual_u[2], 2.0f * G_CONST), -2.0f * G_CONST);
		 virtual_u[2] = fmaxf(fminf(virtual_u[2], 3.0f), -3.0f);

//    // Tinh u1 (Throttle)
//    float u1_static = M_DRONE * (G_CONST + virtual_u[2]);
//    u1_static = fmaxf(fminf(out.u1, 22.0f), 0.0f); 
//		out.u1 = M_DRONE * virtual_u[2];

        // 2. Tinh TONG LUC DAY (Total Thrust) de lam mau so tinh goc X, Y
    // Tong luc = Trong luc + Luc bu sai so
    float u1_total = M_DRONE * (G_CONST + virtual_u[2]);
    
    // Khoa day mau so: Ep luc nang luon toi thieu bang 50% luc Hover
    float min_hover_force = M_DRONE * G_CONST * 0.5f; 
    float den = fmaxf(u1_total, min_hover_force);
		
    float cp = cosf(cur_yaw);
    float sp = sinf(cur_yaw);

    // Tinh Roll/Pitch mong muon
    float theta_lin = SCALE_XY * (M_DRONE / den) * (cp * virtual_u[0] + sp * virtual_u[1]);
    float phi_lin   = SCALE_XY * (M_DRONE / den) * (sp * virtual_u[0] - cp * virtual_u[1]);

    out.theta_d = atanf(theta_lin);
    out.phi_d   = atanf(phi_lin);

    // Saturation goc toi da 0.6 rad
    out.theta_d = fmaxf(fminf(out.theta_d, 0.4f), -0.4f);
    out.phi_d   = fmaxf(fminf(out.phi_d, 0.4f), -0.4f);
		
		  // Tinh XUAT TIN HIEU TRUC Z (Chi lay phan luc bu sai so)
    out.u1 = M_DRONE * virtual_u[2]/(cosf(out.theta_d)*cosf(out.phi_d));

    return out;
}

void AISMC_Outer_Reset(AISMC_Outer_HandleTypeDef *houter, float cur_pos[3]) {
    for(int i=0; i<3; i++) {
        // Xóa s?ch trí nh? tích phân và lu?t thích nghi
        houter->int_e[i] = 0.0f;
        houter->Gamma_hat[i] = 0.0f;
        
        // C?c k? quan tr?ng: Reset v? trí cu b?ng v? trí hi?n t?i
        // N?u không có dòng này, v?n t?c (de) s? b? gi?t lên hàng ch?c m/s trong chu k? d?u tiên
        houter->prev_pos[i] = cur_pos[i]; 
        houter->filtered_vel[i] = 0.0f;
    }
}

