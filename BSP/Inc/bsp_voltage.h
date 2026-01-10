//
// Created by linglitel on 2026/1/10.
//

#ifndef SMARTCAR_BSP_VOLTAGE_H
#define SMARTCAR_BSP_VOLTAGE_H

#include "main.h"

typedef struct {
    // --- 硬件资源 ---
    ADC_HandleTypeDef *hadc; // ADC句柄
    volatile uint16_t *dma_ptr; // 指向DMA缓冲数组中对应的电压数据

    // --- 物理参数 ---
    float r_up; // 分压上拉电阻 (欧姆), 例如 100k
    float r_down; // 分压下拉电阻 (欧姆), 例如 6.8k
    float voltage_ref; // ADC参考电压 (通常 3.3f)

    // --- 滤波配置 ---
    float lpf_alpha; // 低通滤波系数 [0.0 - 1.0]

    // --- 输出数据 ---
    float voltage_bus; // 计算出的实际母线电压 (V)
} Motor_Voltage_t;

void Motor_Voltage_Init(Motor_Voltage_t *motor, ADC_HandleTypeDef *hadc, volatile uint16_t *data_ptr,
                        float r_up, float r_down);

void Motor_Voltage_Enable(Motor_Voltage_t *motor);

void Motor_Current_Calculate(Motor_Voltage_t *motor);

void Motor_Voltage_Update(Motor_Voltage_t *motor);

void Motor_Voltage_DeInit(Motor_Voltage_t *motor);

#endif //SMARTCAR_BSP_VOLTAGE_H
