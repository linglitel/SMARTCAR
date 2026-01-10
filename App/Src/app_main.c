//
// Created by linglitel on 2026/1/10.
//

#include "app_main.h"

#include "app_motor_controller.h"
#include "bsp_current.h"
#include "bsp_temp.h"
#include "bsp_voltage.h"

volatile uint16_t adc1_dma_buffer[5];
volatile uint16_t adc3_dma_buffer[4];

volatile uint8_t system_ready = 0;
volatile uint8_t system_init = 0;

Chip_Temp_t chip_temp;

Motor_Current_t Motor_Current_1;
Motor_Current_t Motor_Current_2;
Motor_Current_t Motor_Current_3;
Motor_Current_t Motor_Current_4;

Motor_Voltage_t Motor_Voltage_1;
Motor_Voltage_t Motor_Voltage_2;
Motor_Voltage_t Motor_Voltage_3;
Motor_Voltage_t Motor_Voltage_4;

Motor_Controller_t Motor_Controller_1;
Motor_Controller_t Motor_Controller_2;
Motor_Controller_t Motor_Controller_3;
Motor_Controller_t Motor_Controller_4;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim20;

static uint8_t task_divider = 0;

void app_init() {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(100);
    Motor_Current_Init(&Motor_Current_1, &hadc1, &adc1_dma_buffer[0], 0.005f, 50.0f, 1.65f);
    Motor_Current_Init(&Motor_Current_2, &hadc1, &adc1_dma_buffer[1], 0.005f, 50.0f, 1.65f);
    Motor_Current_Init(&Motor_Current_3, &hadc1, &adc1_dma_buffer[2], 0.005f, 50.0f, 1.65f);
    Motor_Current_Init(&Motor_Current_4, &hadc1, &adc1_dma_buffer[3], 0.005f, 50.0f, 1.65f);
    HAL_Delay(100);
    Motor_Voltage_Init(&Motor_Voltage_1, &hadc3, &adc3_dma_buffer[0], 100000.0f, 6800.0f);
    Motor_Voltage_Init(&Motor_Voltage_2, &hadc3, &adc3_dma_buffer[1], 100000.0f, 6800.0f);
    Motor_Voltage_Init(&Motor_Voltage_3, &hadc3, &adc3_dma_buffer[2], 100000.0f, 6800.0f);
    Motor_Voltage_Init(&Motor_Voltage_4, &hadc3, &adc3_dma_buffer[3], 100000.0f, 6800.0f);
    Chip_Temp_Init(&chip_temp, &adc1_dma_buffer[4]);
    HAL_Delay(100);
    Motor_Controller_Init(&Motor_Controller_1, &htim1,TIM_CHANNEL_1,TIM_CHANNEL_2, &htim2, 10.0f, 0.5f, 0.0f, 4250.0f,
                          2000.0f, 0.5f, 0.0f, 0.0f, 10000.0f, 5.0f);
    Motor_Controller_Init(&Motor_Controller_2, &htim1,TIM_CHANNEL_3,TIM_CHANNEL_4, &htim3, 10.0f, 0.5f, 0.0f, 4250.0f,
                          2000.0f, 0.5f, 0.0f, 0.0f, 10000.0f, 5.0f);
    Motor_Controller_Init(&Motor_Controller_3, &htim8,TIM_CHANNEL_1,TIM_CHANNEL_2, &htim4, 10.0f, 0.5f, 0.0f, 4250.0f,
                          2000.0f, 0.5f, 0.0f, 0.0f, 10000.0f, 5.0f);
    Motor_Controller_Init(&Motor_Controller_4, &htim8,TIM_CHANNEL_3,TIM_CHANNEL_4, &htim5, 10.0f, 0.5f, 0.0f, 4250.0f,
                          2000.0f, 0.5f, 0.0f, 0.0f, 10000.0f, 5.0f);
    system_init = 1;
}

void app_main() {
    if (!system_init) {
        Error_Handler();
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc1_dma_buffer, 5);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *) adc3_dma_buffer, 5);
    HAL_Delay(100);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_Delay(100);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    system_ready = 1;
}

void App_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!system_ready) {
        return;
    }
    if (htim->Instance == TIM7) {
        // --- 1. 高频任务：电流采样与闭环控制 (每次中断都做，1kHz) ---
        // A. 更新电流 (从DMA取最新值 -> 滤波 -> 存入结构体)
        Motor_Current_Update(&Motor_Current_1);
        Motor_Current_Update(&Motor_Current_2);
        Motor_Current_Update(&Motor_Current_3);
        Motor_Current_Update(&Motor_Current_4);

        Motor_Controller_Isr(&Motor_Controller_1);
        Motor_Controller_Isr(&Motor_Controller_2);
        Motor_Controller_Isr(&Motor_Controller_3);
        Motor_Controller_Isr(&Motor_Controller_4);

        // --- 2. 低频任务：电压和温度 (每10次中断做一次，即100ms/10Hz) ---
        // 电压和温度变化很慢，不需要每毫秒都算，省CPU资源
        task_divider++;
        if (task_divider >= 100) {
            task_divider = 0;
            Motor_Voltage_Update(&Motor_Voltage_1);
            Motor_Voltage_Update(&Motor_Voltage_2);
            Motor_Voltage_Update(&Motor_Voltage_3);
            Motor_Voltage_Update(&Motor_Voltage_4);
            Chip_Temp_Update(&chip_temp);

            // C. 检查保护逻辑
            if (chip_temp.temperature_C > 85.0f) {
                // 紧急停机
                HAL_TIM_Base_Stop_IT(htim); // 停止中断
                // Set_PWM_All(0);
            }
        }
    }
}
