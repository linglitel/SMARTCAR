//
// Created by linglitel on 2026/1/10.
//

#include "bsp_current.h"

void Motor_Current_Init(Motor_Current_t *motor, ADC_HandleTypeDef *hadc, volatile uint16_t *data_ptr,
                        float r_shunt, float gain, float offset) {
    // 绑定硬件资源
    motor->hadc = hadc;
    motor->dma_ptr = data_ptr;

    // 设置物理参数
    motor->shunt_resistor = r_shunt;
    motor->amplifier_gain = gain;
    motor->offset_voltage = offset;

    // 默认参数
    motor->voltage_ref = 3.3f; // STM32G4通常参考电压
    motor->lpf_alpha = 1.0f; // 默认滤波系数 不滤波

    // 清零状态
    motor->current_A = 0.0f;
    motor->current_raw_volts = 0.0f;
}

/*
 * 初始化即启用,保持函数风格
 */
void Motor_Current_Enable(Motor_Current_t *motor) {
}

void Motor_Current_Update(Motor_Current_t *motor) {
    // 1. 获取原始ADC值 (0 - 4095)
    uint16_t raw_adc = *(motor->dma_ptr);

    // 2. 转换为电压值 (STM32G4 ADC默认12位，若开启过采样需调整分母)
    float measured_volts = ((float) raw_adc * motor->voltage_ref) / 4095.0f;
    motor->current_raw_volts = measured_volts;

    // 3. 计算电流: I = (V_out - V_offset) / (Gain * R_sense)
    // INA240公式: V_out = (I * R_shunt * Gain) + V_ref_pin
    float raw_current = (measured_volts - motor->offset_voltage) /
                        (motor->amplifier_gain * motor->shunt_resistor);

    // 4. 简单低通滤波 (Exponential Moving Average)
    // 有刷电机电流噪声大，建议加上滤波
    motor->current_A = (motor->lpf_alpha * raw_current) +
                       ((1.0f - motor->lpf_alpha) * motor->current_A);
}
