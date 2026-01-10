//
// Created by linglitel on 2026/1/10.
//

#include "bsp_voltage.h"


void Motor_Voltage_Init(Motor_Voltage_t *motor, ADC_HandleTypeDef *hadc, volatile uint16_t *data_ptr, float r_up,
                        float r_down) {
    motor->hadc = hadc;
    motor->dma_ptr = data_ptr;

    motor->r_up = r_up;
    motor->r_down = r_down;

    // 默认参数
    motor->voltage_ref = 3.3f;
    motor->lpf_alpha = 0.05f; // 电压通常变化较慢，滤波系数可以设得更小(平滑)

    motor->voltage_bus = 0.0f;
}

/*
 * 初始化即启用,保持函数风格
 */
void Motor_Voltage_Enable(Motor_Voltage_t *motor) {
}

void Motor_Voltage_Update(Motor_Voltage_t *motor) {
    // 1. 获取原始ADC值
    uint16_t raw_adc = *(motor->dma_ptr);

    // 2. 换算为ADC引脚电压
    // 注意：STM32G4是12位ADC (4095)。如果开启了过采样，分母需调整。
    float pin_voltage = ((float) raw_adc * motor->voltage_ref) / 4095.0f;

    // 3. 根据分压公式还原母线电压
    // V_pin = V_bus * (R_down / (R_up + R_down))
    // -> V_bus = V_pin * ((R_up + R_down) / R_down)
    float raw_bus_voltage = pin_voltage * ((motor->r_up + motor->r_down) / motor->r_down);

    // 4. 低通滤波
    motor->voltage_bus = (motor->lpf_alpha * raw_bus_voltage) +
                         ((1.0f - motor->lpf_alpha) * motor->voltage_bus);
}
