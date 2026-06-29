/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "bdma.h"
#include "dcmi.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "madgwick.h"
#include "string.h"
#include "bmp280.h"
#include "stdio.h"
#include "icm20948.h"
#include <stdlib.h>
#include <math.h>
#include "aismc_control.h"
#include "aismc_outer.h"
#include <stdbool.h>
#include "icm20948_mag.h"
#include "BMP388.h"
#include "ukf_fusion.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Khai bao
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
// Extern cho DMA (n?u s? d?ng Callback)
// extern DMA_HandleTypeDef hdma_usart3_rx; 


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_COUNT 20
#define MEDIAN_SIZE 7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t ukf_update_flag = 0;
// Khai báo Handle toàn cục cho Task UKF Predict
osThreadId_t ukfPredictTaskHandle;
// Biến bộ lọc và Mutex bảo vệ
UKF_FilterTypeDef my_ukf;
osMutexId_t ukf_mutex;

volatile float u1_auto_base = 0.0f;    // Giá trị ga tự động tăng dần
volatile bool takeoff_complete = false; // Cờ xác nhận đã cất cánh xong
volatile float target_z = 0.7f;        // Độ cao mục tiêu để ghim ga (ví dụ 0.5 mét)
volatile float ramp_speed = 0.025f;
///////////
volatile float u1_hold = 0.0f;
volatile bool callback = false;
volatile float Roll_SP_outer = 0.0f;
volatile float Pitch_SP_outer = 0.0f;
//debug tu ke
volatile float reference = 0.0f;
volatile float relative = 0.0f;
volatile float current = 0.0f;
////////////////////////
volatile bool stop = false;
//ID task
osThreadId_t fcTaskHandle; 
//////////////////////
volatile uint16_t pi4_rx_size = 0;
volatile float u1_outer = 0.0f;
volatile float Roll_SP_offset, Pitch_SP_offset, Yaw_SP_offset = 0.0f;
volatile int cmd = 0x00;
volatile int drone_status = 0;
volatile bool posit;
float UWB_Process_Time = 0;
volatile float uwb_medians[3] = {0};
volatile float inner_time = 0.0f; 
volatile float outer_time = 0.0f; 
volatile FlightDebug_t log_data; 
char s_msg[256];               
///////////////////////////////
#define DEG_TO_RAD 0.01745329f
//Position
AISMC_Outer_HandleTypeDef h_aismc_outer;
volatile float drone_z_ukf = 0.0f;
volatile float drone_x = 0.0f; 
volatile float drone_y = 0.0f; 
volatile float drone_z = 0.0f;
volatile float vx = 0.0f; 
volatile float vy = 0.0f; 
volatile float vz = 0.0f;
volatile float vz_ukf = 0.0f;
volatile float Roll = 0.0f;  
volatile float Pitch = 0.0f; 
volatile float Yaw = 0.0f;
volatile float drone_x_SP = 0.0f; 
volatile float drone_y_SP = 0.0f; 
volatile float drone_z_SP = 0.0f; 
volatile float Roll_SP = 0.0f; 
volatile float Pitch_SP = 0.0f; 
volatile float Yaw_SP = 0.0f; 
volatile float Throttle = 0.0f; 

volatile float drone_x_SP_callback = 0.0f; 
volatile float drone_y_SP_callback = 0.0f; 
volatile float drone_z_SP_callback = 0.0f; 

//typedef struct
//{
//    float x;
//    float y;
//    float z;
//} Anchor_t;
//Anchor_t anchors[3] =
//{
//    {0.0f, 0.0f, 0.67f},   // BS0
//    {4.2f, 0.0f, 0.67f},   // BS1
//    {0.0f, 6.0f, 0.67f}    // BS2
//};
Anchor_t anchors[3] =
{
    {0.0f, 0.0f, 0.67f},   // BS0
    {1.5f, 0.0f, 0.67f},   // BS1
    {0.0f, 2.1f, 0.67f}    // BS2
};
volatile float distances[3];
volatile uint8_t newDataReady = 0;
////////////////
//uwb
#define UART3_RX_BUFFER_SIZE 256
#define PACKET_SIZE 35

#define HEADER0 0xAA
#define HEADER1 0x25
#define HEADER2 0x01
uint8_t uart3_rx_buffer[UART3_RX_BUFFER_SIZE];
//////////////////
//microzone
#define SBUS_FRAME_SIZE 25
#define SBUS_DMA_BUF_SIZE 64 

__attribute__((section(".RAM_D3"))) __attribute__((aligned(32))) 
uint8_t sbus_rx_buffer[SBUS_DMA_BUF_SIZE];
uint8_t sbus_failsafe = 0;
uint8_t sbus_lostframe = 0;
uint16_t sbus_channels[18];
char msg_out[100];
volatile uint8_t sbus_new_frame = 0;
/////////////////////
//pi 4
#define PI4_DMA_BUF_SIZE 64

__attribute__((section(".RAM_D3"))) __attribute__((aligned(32))) 
volatile uint8_t pi4_rx_buffer[PI4_DMA_BUF_SIZE];
/////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us) {
    uint32_t count = us * (SystemCoreClock / 1000000) / 5; // S? 5 t�y thu?c v�o d�ng chip v� t?i uu h�a
    while(count--);
}
int get_random_int(int min, int max) {
    return (rand() % (max - min + 1)) + min;
}

uint8_t calculate_position_xy_plane(volatile float *x, volatile float *y)
{
    // 1. Lấy tọa độ X, Y của 3 Anchor (Mặc định các Anchor đặt cùng độ cao, coi z1=z2=z3=0)
    float x1 = anchors[0].x; float y1 = anchors[0].y;
    float x2 = anchors[1].x; float y2 = anchors[1].y;
    float x3 = anchors[2].x; float y3 = anchors[2].y;

    // Khoảng cách 3D (r thô) đọc từ UWB
    float r1 = distances[0];
    float r2 = distances[1];
    float r3 = distances[2];

    // 2. Tính hệ số tuyến tính (Triệt tiêu Z do Z của 3 anchor bằng nhau)
    // Phương trình 1: Ax + By = D
    float A = 2.0f * (x2 - x1);
    float B = 2.0f * (y2 - y1);
    float D = (r1 * r1) - (r2 * r2) - (x1 * x1) + (x2 * x2) - (y1 * y1) + (y2 * y2);

    // Phương trình 2: Ex + Fy = H
    float E = 2.0f * (x3 - x1);
    float F = 2.0f * (y3 - y1);
    float H_term = (r1 * r1) - (r3 * r3) - (x1 * x1) + (x3 * x3) - (y1 * y1) + (y3 * y3); 

    // Tính mẫu số (Định thức Cramer cho X-Y)
    float den = A * F - E * B;

    // 3. Kiểm tra định thức
    if (fabsf(den) < 1e-6f) {
        return 0; // Lỗi hình học: Các Anchor X-Y thẳng hàng
    }

    // 4. Giải X, Y trực tiếp 
    *x = (D * F - H_term * B) / den;
    *y = (A * H_term - E * D) / den;

    return 1; // Tính thành công
}

//qsort
int compareFloat(const void *a, const void *b) {
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

void parseData(uint8_t *data, uint16_t len, char *msg)
{
    static float samples[3][MEDIAN_SIZE]; 
    static int idx[3] = {0, 0, 0};       
    static int filled[3] = {0, 0, 0};    

    for (int i = 0; i <= (int)len - PACKET_SIZE; ) 
    {
        if (data[i] == HEADER0 && data[i+1] == HEADER1 && data[i+2] == HEADER2)
        {
            uint8_t *packet = &data[i];
            for (int j = 0; j < 3; j++)
            {
                int offset = 3 + j * 4;
                uint32_t raw = packet[offset] | (packet[offset+1] << 8) | 
                               (packet[offset+2] << 16) | (packet[offset+3] << 24);

                if (raw > 0)
                {
                    // 1. Đẩy mẫu thô vào cửa sổ trượt
                    samples[j][idx[j]] = raw / 1000.0f;
                    idx[j] = (idx[j] + 1) % MEDIAN_SIZE;
                    if (filled[j] < MEDIAN_SIZE) filled[j]++;

                    // 2. Lọc trung vị ngay lập tức khi đủ 5 mẫu khởi tạo
                    if (filled[j] == MEDIAN_SIZE) 
                    {
                        float temp[MEDIAN_SIZE];
                        memcpy(temp, samples[j], sizeof(temp));
                        qsort(temp, MEDIAN_SIZE, sizeof(float), compareFloat);
                        
                        // Lấy giá trị chính giữa (đã lọc rác) gán vào khoảng cách
                        distances[j] = temp[MEDIAN_SIZE / 2]; 
                    }
                }
            }
            i += PACKET_SIZE; 
        }
        else 
        {
            i++; 
        }
    }
    
    // 3. Tính toán X, Y bằng khoảng cách ĐÃ LỌC SẠCH (Bỏ qua Z)
    if (filled[0] == MEDIAN_SIZE && filled[1] == MEDIAN_SIZE && filled[2] == MEDIAN_SIZE)
    {
        // Gọi hàm giải tuyến tính 2D (Triệt tiêu Z)
        if(calculate_position_xy_plane(&drone_x, &drone_y)) {
            posit = true;
        } else {
            posit = false;
        }
    }
}

//void Position_Task(void *argument)
//{
//    AISMC_Outer_Init(&h_aismc_outer, 0.025f);
//	
//    uint16_t old_pos = 0;
//    uint16_t new_pos;
//    char msg[128];
//    uint32_t start_tick; 
//	  float pressure = 101325.0f; 
//    float temperature = 25.0f;
//	  float raw_baro_ukf = 0.0f;
//	
//	  //KHAI BÁO BIẾN THỜI GIAN REAL-TIME ===
//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = pdMS_TO_TICKS(25);
//	  /////////////////////////////////////////
//	
//	  osDelay(2000);
//	  if (BMP388_Init(&hi2c1)) {
//			 //HAL_UART_Transmit(&huart2, (uint8_t*)"BMP388 OK\r\n", 18, 100);
//      BMP388_Set_Ground_Level_ukf(&hi2c1); // Reset độ cao về 0 lúc bắt đầu
//			BMP388_Set_Ground_Level(&hi2c1);
//		}
//		
//		xLastWakeTime = xTaskGetTickCount();

//    for (;;)
//    {
//			
//      
//        if (BMP388_Read_Data(&hi2c1, &pressure, &temperature)) {			
//          drone_z = BMP388_Get_Relative_Altitude(pressure);
//					raw_baro_ukf = BMP388_Get_Relative_Altitude_ukf(pressure);
//				}
//			
//        new_pos = UART3_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

//        if (new_pos != old_pos)
//        {
//            start_tick = DWT->CYCCNT;

//            // 1. Parse data UWB
//            if (new_pos > old_pos) {
//                parseData(&uart3_rx_buffer[old_pos], new_pos - old_pos, msg);
//            } else {
//                parseData(&uart3_rx_buffer[old_pos], UART3_RX_BUFFER_SIZE - old_pos, msg);
//                if (new_pos > 0) parseData(&uart3_rx_buffer[0], new_pos, msg);
//            }
//						
//						// === ADD UKF ===
//            if (osMutexAcquire(ukf_mutex, osWaitForever) == osOK) {
//                UKF_Update(&my_ukf, distances[0], distances[1], distances[2], raw_baro_ukf);
//                osMutexRelease(ukf_mutex);
//            }
//            // =============================================================
//    
//            // 2. Outer Loop
//            float current_p[3] = {drone_x, drone_y, drone_z};
//						float current_v[3] = {vx, vy, vz};
//            float target_p[3]  = {drone_x_SP, drone_y_SP, drone_z_SP};
//            
//            // Yaw dung tinh goc nghieng phu hop
//            AISMC_Outer_Output res = AISMC_Outer_Update(&h_aismc_outer, current_p, current_v, target_p, Yaw * 0.017453f);
//            
//            // 3. output outer loop
//            u1_outer = res.u1;
//            Roll_SP_outer  = - res.phi_d;
//            Pitch_SP_outer =  res.theta_d;
////						Roll_SP_outer  = 0.0f;
////            Pitch_SP_outer = 0.0f;
//            

//            outer_time = (float)(DWT->CYCCNT - start_tick) * 1000000.0f / (float)SystemCoreClock;
//            old_pos = new_pos;
//        }
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//    }
//}
void Position_Task(void *argument)
{
    AISMC_Outer_Init(&h_aismc_outer, 0.025f);
	
    uint16_t old_pos = 0;
    uint16_t new_pos;
    char msg[128];
    uint32_t start_tick; 
    float pressure = 101325.0f; 
    float temperature = 25.0f;
    float raw_baro_ukf = 0.0f;
	
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Nhịp phẳng 40Hz
	
    osDelay(2000);
    if (BMP388_Init(&hi2c1)) {
        BMP388_Set_Ground_Level_ukf(&hi2c1); 
        BMP388_Set_Ground_Level(&hi2c1);
    }
		
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        start_tick = DWT->CYCCNT;

        
        if (BMP388_Read_Data(&hi2c1, &pressure, &temperature)) {			
            drone_z = BMP388_Get_Relative_Altitude(pressure);
            raw_baro_ukf = BMP388_Get_Relative_Altitude_ukf(pressure);
        }
			
        // 🔥 ĐÃ SỬA: Làm sạch D-Cache vùng DMA UWB trước khi CPU vào đọc cấu trúc mảng
        SCB_InvalidateDCache_by_Addr((uint32_t*)uart3_rx_buffer, UART3_RX_BUFFER_SIZE);

        // --- KHỐI NHẬN DỮ LIỆU VÀ UPDATE UKF (Chỉ chạy khi có gói UWB mới) ---
        new_pos = UART3_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
        if (new_pos != old_pos)
        {
            if (new_pos > old_pos) {
                parseData(&uart3_rx_buffer[old_pos], new_pos - old_pos, msg);
            } else {
                parseData(&uart3_rx_buffer[old_pos], UART3_RX_BUFFER_SIZE - old_pos, msg);
                if (new_pos > 0) parseData(&uart3_rx_buffer[0], new_pos, msg);
            }
						
            if (osMutexAcquire(ukf_mutex, osWaitForever) == osOK) {
                UKF_Update(&my_ukf, distances[0], distances[1], distances[2], raw_baro_ukf);
                ukf_update_flag = 1; 
                
                // Đồng bộ ngay dữ liệu mốc Update ra toàn cục dưới sự bảo vệ của Mutex
                drone_x     = my_ukf.state_data[0];
                drone_y     = my_ukf.state_data[1];
                drone_z_ukf = my_ukf.state_data[2];
                vx          = my_ukf.state_data[3];
                vy          = my_ukf.state_data[4];
                vz_ukf      = my_ukf.state_data[5];
                
                osMutexRelease(ukf_mutex);
            }
            old_pos = new_pos;
        }
			
        // --- CHUẨN BỊ MẢNG ĐẦU VÀO ĐỒNG BỘ CHO SMC OUTER ---
        float current_p[3], current_v[3];
        if (osMutexAcquire(ukf_mutex, osWaitForever) == osOK) {
            current_p[0] = drone_x;
            current_p[1] = drone_y;
            current_v[0] = vx;
            current_v[1] = vy;
            osMutexRelease(ukf_mutex);
        }

        current_p[2] = drone_z;  // Ăn Baro thô chuẩn ý bác
        current_v[2] = vz;       // Trục Z tự sai phân trong aismc_outer.c
        
        float target_p[3] = {drone_x_SP, drone_y_SP, drone_z_SP};
        
        AISMC_Outer_Output res = AISMC_Outer_Update(&h_aismc_outer, current_p, current_v, target_p, Yaw * 0.017453f);
        
        u1_outer       = res.u1;
        Roll_SP_outer  = -res.phi_d;
        Pitch_SP_outer =  res.theta_d;

        outer_time = (float)(DWT->CYCCNT - start_tick) * 1000000.0f / (float)SystemCoreClock;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void StartPositionTask_init(void)
{
    const osThreadAttr_t posTask_attr =
    {
        .name = "Position_Task",
        .priority = osPriorityNormal,
        .stack_size = 1024
    };

    osThreadNew(Position_Task, NULL, &posTask_attr);
}
void UWB_Task(void *argument)
{
    uint16_t old_pos = 0;
    uint16_t new_pos;
    char msg[128];
    uint32_t start_tick; 

    for (;;)
    {
        new_pos = UART3_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        if (new_pos != old_pos)
        {
            //Bat dau do
            start_tick = DWT->CYCCNT;

            if (new_pos > old_pos)
            {
                parseData(&uart3_rx_buffer[old_pos], new_pos - old_pos, msg);
            }
            else
            {
                parseData(&uart3_rx_buffer[old_pos], UART3_RX_BUFFER_SIZE - old_pos, msg);
                if (new_pos > 0)
                {
                    parseData(&uart3_rx_buffer[0], new_pos, msg);
                }
            }

            // --- End do ---
            UWB_Process_Time = (float)(DWT->CYCCNT - start_tick) * 1000000.0f / (float)SystemCoreClock;

            old_pos = new_pos;
        }

        osDelay(2); 
    }
}
void StartUWBTask_init(void)
{
    const osThreadAttr_t uwbTask_attr = {
        .name = "UWB_Task",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 1024
    };

    osThreadNew(UWB_Task, NULL, &uwbTask_attr);
}
uint8_t decode_sbus(uint8_t *buf)
{
    uint8_t flags = buf[23];

    sbus_failsafe = (flags >> 3) & 0x01;
    sbus_lostframe = (flags >> 2) & 0x01;

    if (sbus_failsafe || sbus_lostframe)
        return 0;   

    // decode 16 channels 
    sbus_channels[0]  = (uint16_t)((buf[1]  | buf[2] << 8) & 0x07FF);
    sbus_channels[1]  = (uint16_t)((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    sbus_channels[2]  = (uint16_t)((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    sbus_channels[3]  = (uint16_t)((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    sbus_channels[4]  = (uint16_t)((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    sbus_channels[5]  = (uint16_t)((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    sbus_channels[6]  = (uint16_t)((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    sbus_channels[7]  = (uint16_t)((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    // ...

    return 1;   
}



void ESC_Calibration()
{
  // B1: Gui Max PWM
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 2000);

  HAL_Delay(2000);  

  // B2: Gui Min PWM
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);

  HAL_Delay(2000);  
}

void StartUart2Task(void *argument) {
    HAL_UART_Transmit(&huart3, (uint8_t*)"UART Debug Start\r\n", 18, 100);

    for (;;) {
//			int len = sprintf(s_msg, "IMU:%.4f,%.4f,%.4f|SP:%.2f,%.2f,%.2f|ERR:%.2f,%.2f,%.2f|U:%.2f,%.2f,%.2f,%.2f|M:%d,%d,%d,%d |Time: %.2f\r\n",
//                log_data.roll, log_data.pitch, log_data.yaw,
//                log_data.roll_d, log_data.pitch_d, log_data.yaw_d,
//                log_data.roll - log_data.roll_d, log_data.pitch - log_data.pitch_d, log_data.yaw - log_data.yaw_d,
//                log_data.u1, log_data.u2, log_data.u3, log_data.u4,
//                log_data.m[0], log_data.m[1], log_data.m[2], log_data.m[3],
//			inner_time);


//        HAL_UART_Transmit(&huart3, (uint8_t*)s_msg, len, 20);
							//debug
//char msg[192]; // Tăng kích thước buffer lên một chút cho an toàn
//snprintf(msg, sizeof(msg),
//	"R=%.4f P=%.4f Y=%.4f GR=%.4f GP=%.4f GY=%.4f \r\n mag_x:%d mag_y:%d mag_z:%d\r\n MD=%.2f Drift=%.2f | Time: %.2f\r\n",
//         Roll,
//         Pitch,
//         Yaw,
//         GyroRoll,
//         GyroPitch,
//         GyroYaw,
//         mag_x,
//         mag_y,
//         mag_z, 
//         Mag_Delta,
//         Yaw_Drift,
//         IMU_Time);

//HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//			        char msg[64];
//        snprintf(msg, sizeof(msg),
//                 "TIme=%.4f us\r\n",
//                 UWB_Process_Time);

//        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//				
				
//				if(posit == true)
//				{
//					        char msg[64];
//				int lenMsg = sprintf(msg, "x: %.3f | y: %.3f | z: %.3f\r\n BS0: %.3f | BS1: %.3f | BS2: %.3f \r\n", 
//                     drone_x, drone_y, drone_z, distances[0], distances[1], distances[2]);
//HAL_UART_Transmit(&huart3, (uint8_t*)msg, lenMsg, 10);
//					    HAL_UART_Transmit(&huart3,
//        (uint8_t*)"Co giao diem \r\n",
//        18, HAL_MAX_DELAY);
//				}
//				else if(posit == false)
//				{
//					    HAL_UART_Transmit(&huart3,
//        (uint8_t*)"Khong co giao diem \r\n",
//        25, HAL_MAX_DELAY);
//				}
//////////////////////
//char msg[100];
//				int lenMsg = sprintf(msg, "x: %.3f | y: %.3f | z: %.3f\r\n BS0: %.3f | BS1: %.3f | BS2: %.3f \r\n", 
//                     drone_x, drone_y, drone_z, distances[0], distances[1], distances[2]);
//HAL_UART_Transmit(&huart2, (uint8_t*)msg, lenMsg, 10);
/////////////////////////
char msg[128];
int lenMsg = sprintf(msg, "x: %.3f y: %.3f z: %.3f | accroll: %.3f accpitch: %.3f accyaw: %.3f | In: %.0f Out: %.0f\r\n", 
                     drone_x, drone_y, drone_z, accel_roll, accel_pitch, accel_yaw, inner_time, outer_time);
HAL_UART_Transmit(&huart2, (uint8_t*)msg, lenMsg, HAL_MAX_DELAY);

/////////////////////
//char msg[100];

//int len = sprintf(msg, "CMD: 0x%02X\r\nX: %.2f | Y: %.2f | Z: %.2f\r\n", 
//                  (int)cmd, drone_x_SP_callback, drone_y_SP_callback, drone_z_SP_callback);


//HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 100);
        osDelay(1000); 
    }
}
void StartUartTask_init()
{
	  const osThreadAttr_t uart2Task_attributes = {
    .name = "uart2Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024 * 4
  };
  osThreadNew(StartUart2Task, NULL, &uart2Task_attributes);
}



void StartICM20948Task(void *argument) {

    //Khoi tao cam bien
//    ICM20948_Init(&hi2c4);
	 ICM20948_Init_mag(&hi2c4);
        osDelay(500); 

//        ICM20948_Calibrate(&hi2c4);
	 ICM20948_Calibrate_mag(&hi2c4);
	  osDelay(3000); 

    for(;;) {
        uint32_t start = DWT->CYCCNT;
        
        //Loc du lieu
//        ICM20948_Process(&hi2c4);
			 ICM20948_Process_mag(&hi2c4);
			
        
        inner_time = (float)(DWT->CYCCNT - start) * 1000000.0f / (float)SystemCoreClock;
        
        osDelay(2); 
    }
}

void StartICM20948Task_init(void)
{
  const osThreadAttr_t icm20948Task_attr = {
    .name = "ICM20948Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  osThreadNew(StartICM20948Task, NULL, &icm20948Task_attr);
}
////stm send to pi
uint8_t CalcCRC(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0x00;  

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];  
    }

    return crc;  
}

//void SendPose(void)
//{
//    uint8_t tx_buf[86]; 

//    tx_buf[0] = 0xFF; // Header
//    tx_buf[1] = 0x00; // CMD
//    tx_buf[2] = 81;   // Length: 57 + 24 = 81

//    memcpy(&tx_buf[3],  (void *)&drone_x, 4);
//    memcpy(&tx_buf[7],  (void *)&drone_y, 4);
//    memcpy(&tx_buf[11], (void *)&drone_z, 4);
//    memcpy(&tx_buf[15], (void *)&Roll, 4);
//    memcpy(&tx_buf[19], (void *)&Pitch, 4);
//    memcpy(&tx_buf[23], (void *)&Yaw, 4);
//    memcpy(&tx_buf[27], (void *)&drone_x_SP, 4);
//    memcpy(&tx_buf[31], (void *)&drone_y_SP, 4);
//    memcpy(&tx_buf[35], (void *)&drone_z_SP, 4);
//    memcpy(&tx_buf[39], (void *)&log_data.roll_d, 4);
//    memcpy(&tx_buf[43], (void *)&log_data.pitch_d, 4);
//    memcpy(&tx_buf[47], (void *)&Yaw_SP, 4);
//    memcpy(&tx_buf[51], (void *)&Throttle, 4);

//    // ------
//    memcpy(&tx_buf[55], (void *)&log_data.m[1], 4);
//    memcpy(&tx_buf[59], (void *)&log_data.m[2], 4);
//    memcpy(&tx_buf[63], (void *)&log_data.m[3], 4);
//    memcpy(&tx_buf[67], (void *)&log_data.m[4], 4);
//    memcpy(&tx_buf[71], (void *)&inner_time, 4);
//    memcpy(&tx_buf[75], (void *)&outer_time, 4);

//    // ------
//    memcpy(&tx_buf[79], (void *)&drone_status, 4);
//    memcpy(&tx_buf[83], (void *)&posit, 1);

//    // CRC va End byte
//    tx_buf[84] = CalcCRC(&tx_buf[1], 83); // CRC byte 1 to 83
//    tx_buf[85] = 0xFE;

//    HAL_UART_Transmit(&huart2, tx_buf, 86, HAL_MAX_DELAY);
//}
void SendPose(void)
{
    uint8_t tx_buf[98]; 

    tx_buf[0] = 0xFF; // Header
    tx_buf[1] = 0x00; // CMD
    tx_buf[2] = 93;   // Length: 81 + 12 = 93

    memcpy(&tx_buf[3],  (void *)&drone_x, 4);
    memcpy(&tx_buf[7],  (void *)&drone_y, 4);
    memcpy(&tx_buf[11], (void *)&drone_z, 4);
    
    // --- ADD vx, vy, vz ---
    memcpy(&tx_buf[15], (void *)&vx, 4);
    memcpy(&tx_buf[19], (void *)&vy, 4);
    memcpy(&tx_buf[23], (void *)&vz, 4);

    // Dịch toàn bộ các chỉ số phía sau cộng thêm 12
    memcpy(&tx_buf[27], (void *)&Roll, 4);
    memcpy(&tx_buf[31], (void *)&Pitch, 4);
    memcpy(&tx_buf[35], (void *)&Yaw, 4);
    memcpy(&tx_buf[39], (void *)&drone_x_SP, 4);
    memcpy(&tx_buf[43], (void *)&drone_y_SP, 4);
    memcpy(&tx_buf[47], (void *)&drone_z_SP, 4);
    memcpy(&tx_buf[51], (void *)&log_data.roll_d, 4);
    memcpy(&tx_buf[55], (void *)&log_data.pitch_d, 4);
    memcpy(&tx_buf[59], (void *)&Yaw_SP, 4);
    memcpy(&tx_buf[63], (void *)&Throttle, 4);

    // ------
    memcpy(&tx_buf[67], (void *)&log_data.m[1], 4);
    memcpy(&tx_buf[71], (void *)&log_data.m[2], 4);
    memcpy(&tx_buf[75], (void *)&log_data.m[3], 4);
    memcpy(&tx_buf[79], (void *)&log_data.m[4], 4);
    memcpy(&tx_buf[83], (void *)&inner_time, 4);
    memcpy(&tx_buf[87], (void *)&outer_time, 4);

    // ------
    memcpy(&tx_buf[91], (void *)&drone_status, 4);
    memcpy(&tx_buf[95], (void *)&posit, 1);

    // CRC và End byte đã được dịch chỉ số
    // Tính CRC từ byte 1 đến byte 95 (tổng cộng 95 byte)
    tx_buf[96] = CalcCRC(&tx_buf[1], 95); 
    tx_buf[97] = 0xFE;

    HAL_UART_Transmit(&huart2, tx_buf, 98, HAL_MAX_DELAY);
}
void StartPoseTask(void *argument)
{
    uint8_t temp_buffer[34];
    uint16_t current_dma_pos = 0;
    static uint16_t last_dma_pos = 0;
	
    HAL_UART_Transmit(&huart2,
        (uint8_t*)"Pose task ready\r\n",
        18, HAL_MAX_DELAY);

    for (;;)
    {
			  // Xử lý Cache cho H7 trước khi đọc RAM
        SCB_InvalidateDCache_by_Addr((uint32_t*)pi4_rx_buffer, PI4_DMA_BUF_SIZE);

        // Lấy vị trí ghi hiện tại của DMA
        current_dma_pos = PI4_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

        // Chỉ xử lý nếu DMA có dịch chuyển (có dữ liệu mới)
        if (current_dma_pos != last_dma_pos) 
        {
            // Quét ngược từ vị trí current_pos về trước để tìm Header 0xFF của gói tin mới nhất
            // Cách này không dùng 'while', chỉ dùng 'for' để giới hạn phạm vi quét
            for (int i = 0; i < PI4_DMA_BUF_SIZE; i++) 
            {
                // Tính toán vị trí kiểm tra (quét lùi từ current_pos)
                int check_idx = (current_dma_pos - i - 34 + PI4_DMA_BUF_SIZE * 2) % PI4_DMA_BUF_SIZE;

                if (pi4_rx_buffer[check_idx] == 0xFF) 
                {
                    // Copy ra buffer tạm để check wrap-around (vòng mảng)
                    for (int j = 0; j < 34; j++) {
                        temp_buffer[j] = pi4_rx_buffer[(check_idx + j) % PI4_DMA_BUF_SIZE];
                    }

                    // Kiểm tra Footer và CRC
                    if (temp_buffer[33] == 0xFE) 
                    {
                        uint8_t crc_calc = CalcCRC(&temp_buffer[1], 31);
                        if (crc_calc == temp_buffer[32]) 
                        {
                            // --- CẬP NHẬT TRẠNG THÁI ---
                            cmd = temp_buffer[1];
                            posit = temp_buffer[31];

                            switch (cmd) {
                                case 0x01:
																	{
																		//if(Throttle <= 0.0f){BMP388_Set_Ground_Level(&hi2c1);}
                                    memcpy((void *)&Roll_SP_offset,  &temp_buffer[15], 4);
                                    memcpy((void *)&Pitch_SP_offset, &temp_buffer[19], 4);
                                    memcpy((void *)&Yaw_SP_offset,   &temp_buffer[23], 4);
                                    drone_status = 0x01;
																    stop = false;
																	 }
                                    break;
                                case 0x02:
																	{
																		//if(Throttle <= 0.0f){BMP388_Set_Ground_Level(&hi2c1);}
																		if (drone_status != 0x02) 
                                    {
                                        // 1. Chốt ga nền từ chế độ trước đó
                                        if(drone_status == 0x01) { u1_hold = Throttle; }
                                        if(drone_status == 0x03) { u1_hold = u1_auto_base; }
                                        
                                        // 3. Tẩy não vòng ngoài (chỉ 1 lần duy nhất)
                                        float current_pos_reset[3] = {drone_x, drone_y, drone_z};
                                        AISMC_Outer_Reset(&h_aismc_outer, current_pos_reset);
                                        
                                        // 4. Cập nhật trạng thái
                                        drone_status = 0x02;
                                    }
                                    drone_x_SP = drone_x;
                                    drone_y_SP = drone_y;
                                    drone_z_SP = drone_z;
																    stop = false;
																		
																	}
                                    break;
                                case 0x03:
																	{
																		if(drone_status == 0x01) { u1_auto_base = Throttle; }
                                    if(drone_status == 0x02) { u1_auto_base = u1_hold; }
																		//if(Throttle <= 0.0f){BMP388_Set_Ground_Level(&hi2c1);}
                                    memcpy((void *)&drone_x_SP, &temp_buffer[3], 4);
                                    memcpy((void *)&drone_y_SP, &temp_buffer[7], 4);
                                    memcpy((void *)&drone_z_SP, &temp_buffer[11], 4);
																		if (drone_status != 0x03) 
                                    {
                                        float current_pos_reset_auto[3] = {drone_x, drone_y, drone_z};
                                        AISMC_Outer_Reset(&h_aismc_outer, current_pos_reset_auto);
                                        
                                        drone_status = 0x03;
                                    }
																    stop = false;														
																	}
                                    break;
                                case 0x04:
																	{
																		//BMP388_Set_Ground_Level(&hi2c1); // Reset độ cao về 0
                                    drone_z_SP = 0.1f;
                                    drone_status = 0x04;
																    stop = true;
																	}
                                    break;
																case 0x05:
																	{
																		if(Throttle < 0.1f)
																		{
																			if (BMP388_Init(&hi2c1)) {
																				BMP388_Set_Ground_Level(&hi2c1); 
																			}
																			//UKF_Init(&my_ukf);
																		}
																	}
																	  break;
                            }
                            // Tìm thấy gói mới nhất và hợp lệ rồi thì thoát vòng for quét
                            break; 
                        }
                    }
                }
            }
            // Cập nhật lại vị trí đã đọc
            last_dma_pos = current_dma_pos;
        }

        // Kiểm tra lỗi Overrun phần cứng định kỳ để tránh treo UART
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
        }
        SendPose(); 				

				osDelay(500);
    }
}
void StartPoseTask_init(void)
{
  const osThreadAttr_t poseTask_attr = {
    .name = "PoseTask",
    .priority = (osPriority_t) osPriorityBelowNormal,
    .stack_size = 1024
  };

  osThreadNew(StartPoseTask, NULL, &poseTask_attr);
}



void StartFlightControlTask(void *argument) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
    // 1. Chỉ khai báo biến ở đây, CHƯA gán giá trị
    TickType_t xLastWakeTime;
    
    // Đổi sang macro chuẩn để an toàn tuyệt đối
    const TickType_t xFrequency = pdMS_TO_TICKS(5);

    ICM20948_Init_mag(&hi2c4);
	
	
	  // --- CALIB ---
    ICM20948_Calibrate_mag(&hi2c4);
	
	// === ADD UKF===
//    UKF_Init(&my_ukf);
//    const osMutexAttr_t ukf_mutex_attr = { .name = "ukf_mutex" };
//    ukf_mutex = osMutexNew(&ukf_mutex_attr);
    // ===============================================
    
    // 2. Khoi tao bdk
    AISMC_HandleTypeDef hquad;
    AISMC_Init(&hquad, 0.005f); 
    AISMC_Output ctrl = {0, 0, 0};
    
    osDelay(2000); 
		
		float des_ang[3];
		float u1;
		xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        // --- Start ---
        uint32_t start_tick = DWT->CYCCNT;

        // B1: IMU
        ICM20948_Process_mag(&hi2c4);
			

        // B2: 
			  float cur_ang[3] = { Roll * 0.017453f, Pitch * 0.017453f, Yaw * 0.017453f };
				float cur_gyro[3] = { GyroRoll, GyroPitch, GyroYaw };
//			////////chay test

//        float des_ang[3] = {0.0f, 0.0f, 0.0f
////            mapf(sbus_channels[0], 696, 1400, -0.4f, 0.4f) - 0.05f, 
////            mapf(sbus_channels[1], 720, 1398, -0.4f, 0.4f) - 0.07f, 
////            mapf(sbus_channels[3], 600, 1334, -0.5f, 0.5f) + 0.05f
//        };
//        float u1 = mapf(sbus_channels[2], 200, 1652, 0.0f, 32.0f);
//				if(u1 > 24.0f) u1 = 24.0f;
				
			///////////////////////////////////////////////////////////////////
				switch (drone_status){
					case 0x01:
					{
						//tay cam
//            des_ang[0] = mapf(sbus_channels[0], 696, 1400, -0.22f, 0.22f);
//            des_ang[1] = mapf(sbus_channels[1], 720, 1398, -0.22f, 0.22f);
//            des_ang[2] = mapf(sbus_channels[3], 600, 1334, -0.5f, 0.5f);
						//set cung
						des_ang[0] = 0.0f;
            des_ang[1] = 0.0f;
            des_ang[2] = 0.0f;
						Roll_SP = des_ang[0];
						Pitch_SP = des_ang[1];
						Yaw_SP = des_ang[2];
						u1 = mapf(sbus_channels[2], 200, 1652, 0.0f, 32.0f);
						if(u1 > 26.0f) u1 = 26.0f;
					}break;
					case 0x02:
					{
						des_ang[0] = Roll_SP_outer;
						des_ang[1] = Pitch_SP_outer;
						Roll_SP = Roll_SP_outer;
						Pitch_SP = Pitch_SP_outer;
						des_ang[2] = Yaw_SP;
						u1 = u1_hold + u1_outer;
						if(u1>25.0f) u1 = 25.0f;
//						u1 = mapf(sbus_channels[2], 200, 1652, 0.0f, 32.0f);
	//					if(u1 > 26.0f) u1 = 26.0f;
					}break;
					case 0x03:
					{
						des_ang[0] = Roll_SP_outer;
						des_ang[1] = Pitch_SP_outer;
						Roll_SP = Roll_SP_outer;
						Pitch_SP = Pitch_SP_outer;
						des_ang[2] = Yaw_SP;

//						// --- LOGIC CẤT CÁNH TỰ ĐỘNG ---
//						if (!takeoff_complete) {
//							// 1. Tăng ga dần dần cho đến khi đạt độ cao target_z
//							u1_auto_base += ramp_speed; 
//							// Giới hạn ga tối đa trong lúc cất cánh để tránh vọt quá mạnh
//							if (u1_auto_base > 23.0f) u1_auto_base = 23.0f; 
//							// 2. Kiểm tra nếu đã đạt độ cao mục tiêu
//							if (drone_z >= target_z) {
//								takeoff_complete = true; // Ghim giá trị ga tại đây
//							}
//						}
						// 3. Tính toán u1 cuối cùng
						u1 = u1_auto_base + u1_outer;
						// Safety Limit
						if(u1 > 27.0f) u1 = 27.0f;
					}break;
					default:
					{
						u1 = 0.0f;
						des_ang[0] = 0.0f;
						des_ang[1] = 0.0f;
						des_ang[2] = 0.0f;
						Roll_SP = des_ang[0];
						Pitch_SP = des_ang[1];
						Yaw_SP = des_ang[2];
					}
				}


        // B3: BDK  &&(drone_status >= 0x01)&&(drone_status <= 0x03)
        if ((mapf(sbus_channels[2], 200, 1652, 0.0f, 32.0f) >= 0.1f)&&(drone_status >= 0x01)&&(drone_status <= 0x03)&&(stop == false)) { 
            ctrl = AISMC_Update(&hquad, cur_ang, des_ang, cur_gyro, u1);
            Actuator_Write(u1, ctrl);
					Throttle = u1;
        } else 
				{
					  Throttle -= 0.15f;
						ctrl = AISMC_Update(&hquad, cur_ang, des_ang, cur_gyro, Throttle);
            Actuator_Write(Throttle, ctrl);
					  if(Throttle < 0.0f)
						{
							//BMP388_Set_Ground_Level(&hi2c1);
							Throttle = 0.0f;
							Actuator_Stop();
							
							// === RESET CÁC BIẾN CẤT CÁNH TỰ ĐỘNG Ở ĐÂY ===
                takeoff_complete = false;
                u1_auto_base = 0.0f;
                u1_hold = 0.0f;
						}
            
				}

        // B4: debug
				log_data.roll_d = Roll_SP * 57.29578f; 
        log_data.pitch_d = Pitch_SP * 57.29578f;
//        log_data.roll = cur_ang[0]; log_data.pitch = cur_ang[1]; log_data.yaw = cur_ang[2];
//        log_data.roll_d = des_ang[0]; log_data.pitch_d = des_ang[1]; log_data.yaw_d = des_ang[2];
//        log_data.u1 = u1; log_data.u2 = ctrl.u2; log_data.u3 = ctrl.u3; log_data.u4 = ctrl.u4;
////        log_data.m[0] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
////        log_data.m[1] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
////        log_data.m[2] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
////        log_data.m[3] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
				
							// === ADD UKF: ===
//        if (osMutexAcquire(ukf_mutex, 0) == osOK) {
//            UKF_Predict(&my_ukf, 0.01f);
//            osMutexRelease(ukf_mutex);
//        }
        // =========================================
				
        inner_time = (float)(DWT->CYCCNT - start_tick) * 1000000.0f / (float)SystemCoreClock;

        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void FlightControl_InitTask(void)
{
    const osThreadAttr_t fc_attr = {
        .name = "FlightControl",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 2048 
    };
    fcTaskHandle = osThreadNew(StartFlightControlTask, NULL, &fc_attr);
}

//void StartUKFPredictTask(void *argument) {
//    UKF_Init(&my_ukf);
//    const osMutexAttr_t ukf_mutex_attr = { .name = "ukf_mutex" };
//    ukf_mutex = osMutexNew(&ukf_mutex_attr);

//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

//    osDelay(4000); 

//    xLastWakeTime = xTaskGetTickCount();

//    for(;;) {
//        uint32_t start_tick = DWT->CYCCNT;

//        if (osMutexAcquire(ukf_mutex, osWaitForever) == osOK) {
//            UKF_Predict(&my_ukf, 0.01f);
//            osMutexRelease(ukf_mutex);
//        }

////        ukf_predict_time = (float)(DWT->CYCCNT - start_tick) * 1000000.0f / (float)SystemCoreClock;

//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//    }
//}
void StartUKFPredictTask(void *argument) {
    UKF_Init(&my_ukf);
    const osMutexAttr_t ukf_mutex_attr = { .name = "ukf_mutex" };
    ukf_mutex = osMutexNew(&ukf_mutex_attr);

    static UKF_FilterTypeDef shadow_ukf;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    osDelay(4000); 

    osMutexAcquire(ukf_mutex, osWaitForever);
    memcpy(&shadow_ukf, &my_ukf, sizeof(UKF_FilterTypeDef));
    osMutexRelease(ukf_mutex);

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        // Đồng bộ nhanh từ gốc vào shadow làm mốc tính chu kỳ mới
        osMutexAcquire(ukf_mutex, osWaitForever);
        memcpy(&shadow_ukf, &my_ukf, sizeof(UKF_FilterTypeDef));
        osMutexRelease(ukf_mutex);

        // Chạy toán nặng 5ms tự do (Thời gian thực thi IMU Kinematics ăn theo biến Snapshot siêu an toàn)
        UKF_Predict(&shadow_ukf, 0.01f);

        osMutexAcquire(ukf_mutex, osWaitForever);
        
        if (ukf_update_flag) {
            ukf_update_flag = 0; 
        } else {
            memcpy(&my_ukf, &shadow_ukf, sizeof(UKF_FilterTypeDef));
        }

        // Đồng bộ chuẩn xác kết quả ra biến toàn cục
        drone_x     = my_ukf.state_data[0];
        drone_y     = my_ukf.state_data[1];
        drone_z_ukf = my_ukf.state_data[2];
        vx          = my_ukf.state_data[3];
        vy          = my_ukf.state_data[4];
        vz_ukf      = my_ukf.state_data[5];
        
        osMutexRelease(ukf_mutex);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void UKFPredict_InitTask(void)
{
    const osThreadAttr_t ukf_pred_attr = {
        .name = "UKF_Predict_Task",
        .priority = (osPriority_t) osPriorityNormal, 
        .stack_size = 4096   
    };
    
    // Tạo Task độc lập
    ukfPredictTaskHandle = osThreadNew(StartUKFPredictTask, NULL, &ukf_pred_attr);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	ESC_Calibration();
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1050);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1050);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1050);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1050);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, sbus_rx_buffer, SBUS_DMA_BUF_SIZE);
__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

HAL_UART_Receive_DMA(&huart2, (uint8_t *)pi4_rx_buffer, PI4_DMA_BUF_SIZE);

HAL_UART_Receive_DMA(&huart3,
                     uart3_rx_buffer,
                     UART3_RX_BUFFER_SIZE);
										 
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  //MX_FREERTOS_Init();
  StartPositionTask_init();
	FlightControl_InitTask();
	StartPoseTask_init();
	UKFPredict_InitTask();
	//StartICM20948Task_init();
	//StartUartTask_init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART4) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

        if (Size >= SBUS_FRAME_SIZE) {
            for (int i = 0; i <= (Size - SBUS_FRAME_SIZE); i++) {
                if (sbus_rx_buffer[i] == 0x0F && sbus_rx_buffer[i + 24] == 0x00) {
                    if (decode_sbus(&sbus_rx_buffer[i])) {
                        sbus_new_frame = 1; 
                    }
                    break;
                }
            }
        }
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, sbus_rx_buffer, SBUS_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT); 
    }
//		else if (huart->Instance == USART2) {
//			SCB_InvalidateDCache_by_Addr((uint32_t*)pi4_rx_buffer, PI4_DMA_BUF_SIZE);
//    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
//								    if (Size >= 34) {
//        for (int i = 0; i <= (Size - 34); i++) {
//            // 1. Check Header 0xFF va Footer 0xFE
//            if (pi4_rx_buffer[i] == 0xFF && pi4_rx_buffer[i + 33] == 0xFE) {
//                
//                // 2. Check CRC 
//                uint8_t crc_calc = CalcCRC((uint8_t *)&pi4_rx_buffer[i + 1], 31);
//                if (crc_calc == pi4_rx_buffer[i + 32]) {
//                    
//                    cmd = pi4_rx_buffer[i + 1];

//                    // 3. Parse Data
//                    //memcpy((void *)&drone_status, &pi4_rx_buffer[i + 27], 4);
//                    posit = pi4_rx_buffer[i + 31];
//                                    
//                    switch (cmd) {
//                        case 0x01:
//													{
//                            memcpy((void *)&Roll_SP_offset,  (void *)&pi4_rx_buffer[i + 15], 4);
//                            memcpy((void *)&Pitch_SP_offset, (void *)&pi4_rx_buffer[i + 19], 4);
//                            memcpy((void *)&Yaw_SP_offset,   (void *)&pi4_rx_buffer[i + 23], 4);
//                            drone_status = 0x01;
//                            stop = false;
//													}
//                            break;
//                        case 0x02:
//													{
//                            drone_status = 0x02;
//                            drone_x_SP = drone_x;
//                            drone_y_SP = drone_y;
//                            drone_z_SP = drone_z;
//                            stop = false;
//													}
//                            break;
//                        case 0x03:
//													{
//                            drone_status = 0x03;
//                            memcpy((void *)&drone_x_SP, (void *)&pi4_rx_buffer[i + 3], 4);
//                            memcpy((void *)&drone_y_SP, (void *)&pi4_rx_buffer[i + 7], 4);
//                            memcpy((void *)&drone_z_SP, (void *)&pi4_rx_buffer[i + 11], 4);
//                            stop = false;
//													}
//                            break;
//                        case 0x04:
//													{
//                            drone_status = 0x04;
//												    drone_x_SP = drone_x;
//                            drone_y_SP = drone_y;
//                            drone_z_SP = 0.1f;
//                            stop = true; 
//													}
//                            break;
//												default:
//													{
//														drone_status = 0xFF;
//													}
//                    }
//                }
//                break; 
//            }
//        }
//    }
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)pi4_rx_buffer, PI4_DMA_BUF_SIZE);
//    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
