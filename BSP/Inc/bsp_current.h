//
// Created by linglitel on 2026/1/10.
//

#ifndef SMARTCAR_BSP_CURRENT_H
#define SMARTCAR_BSP_CURRENT_H

#include "main.h"

typedef struct {
    ADC_HandleTypeDef *hadc; // ADC句柄 (仅作记录，实际读取靠dma_ptr)
    volatile uint16_t *dma_ptr; // 指向DMA缓冲数组中该电机对应的数据位

    // --- 物理参数配置 ---
    float shunt_resistor; // 采样电阻阻值 (例如 0.001 Ohm)
    float amplifier_gain; // INA240增益 (例如 20, 50, 100, 200)
    float voltage_ref; // ADC参考电压 (通常为 3.3f)
    float offset_voltage; // 零电流时的电压偏置 (双向采样通常为 1.65f，单向为 0.0f)

    // --- 滤波配置 ---
    float lpf_alpha; // 低通滤波系数 [0.0 - 1.0], 1.0为不滤波, 0.1为强滤波

    // --- 运行时数据 ---
    float current_raw_volts; // 换算出的传感器输出电压 (V)
    float current_A; // 计算出的实际电流 (Amps) - 已滤波
} Motor_Current_t;

void Motor_Current_Init(Motor_Current_t *motor, ADC_HandleTypeDef *hadc, volatile uint16_t *data_ptr,
                        float r_shunt, float gain, float offset);

void Motor_Current_Enable(Motor_Current_t *motor);

void Motor_Current_Calculate(Motor_Current_t *motor);

void Motor_Current_Update(Motor_Current_t *motor);

void Motor_Current_DeInit(Motor_Current_t *motor);

#endif //SMARTCAR_BSP_CURRENT_H
